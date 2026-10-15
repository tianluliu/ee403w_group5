"""Tests for the Mac/Pi helper scripts.

Every invocation here is --help, --show-command, or an input-validation failure that exits
before any connection is attempted. Nothing contacts a host, installs software or touches
hardware. Needs only bash (macOS ships 3.2; the scripts are written for it).
"""

import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

from _bootstrap import REPO_ROOT

SCRIPTS = REPO_ROOT / "scripts"
RUNNABLE = ("test.sh", "check_env.sh", "deploy.sh", "remote.sh")
GOOD_ENV = "PI_HOST=raspberrypi.local\nPI_USER=ubuntu\nPI_REMOTE_DIR=lidar-mapping-robot\nPI_SSH_PORT=22\n"


def run(script, *args, env_text=None, env_missing=False):
    env = dict(os.environ)
    tmp = None
    if env_missing:
        env["PI_ENV_FILE"] = "/nonexistent/pi.env"
    elif env_text is not None:
        tmp = tempfile.TemporaryDirectory()
        path = Path(tmp.name) / "pi.env"
        path.write_text(env_text)
        env["PI_ENV_FILE"] = str(path)
    try:
        return subprocess.run(["bash", str(SCRIPTS / script), *args], capture_output=True, text=True,
                              env=env, timeout=60, cwd=tmp.name if tmp else None)
    finally:
        if tmp:
            tmp.cleanup()


class StaticScriptTests(unittest.TestCase):
    def test_syntax(self):
        for name in RUNNABLE + ("_common.sh",):
            with self.subTest(script=name):
                r = subprocess.run(["bash", "-n", str(SCRIPTS / name)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr)

    def test_executable_bits(self):
        for name in RUNNABLE:
            with self.subTest(script=name):
                self.assertTrue((SCRIPTS / name).stat().st_mode & stat.S_IXUSR)

    def test_scripts_contain_no_install_privilege_or_robot_commands(self):
        # Executable lines only: comments and usage text may legitimately say "uses no sudo".
        forbidden = ("sudo", "apt ", "apt-get", "pip install", "pip3 install", "brew ", "ros2 run",
                     "ros2 launch", "stty", "minicom", "picocom", "avrdude", "arduino-cli",
                     "docker run", "docker build", "docker pull")
        for path in sorted(SCRIPTS.glob("*.sh")):
            lines = path.read_text().splitlines()
            code, in_heredoc, terminator = [], False, ""
            for line in lines:
                stripped = line.strip()
                if in_heredoc:
                    in_heredoc = stripped != terminator
                    continue
                if "<<'" in line:
                    in_heredoc, terminator = True, line.split("<<'")[1].split("'")[0]
                if stripped.startswith("#"):
                    continue
                code.append(line)
            self.assertTrue(code, path.name)
            text = "\n".join(code)
            for word in forbidden:
                with self.subTest(script=path.name, word=word):
                    self.assertNotIn(word, text)

    def test_deploy_never_deletes_remote_files(self):
        self.assertNotIn("--delete", (SCRIPTS / "deploy.sh").read_text().replace("never deletes", ""))

    def test_pi_env_is_git_ignored_and_example_has_only_allowed_keys(self):
        self.assertIn("scripts/pi.env", (REPO_ROOT / ".gitignore").read_text().splitlines())
        keys = [l.split("=")[0] for l in (SCRIPTS / "pi.env.example").read_text().splitlines()
                if l and not l.startswith("#")]
        self.assertEqual(sorted(keys), ["PI_HOST", "PI_REMOTE_DIR", "PI_SSH_PORT", "PI_USER"])


class UsageTests(unittest.TestCase):
    def test_help_exits_zero_and_shows_usage(self):
        for name in ("test.sh", "deploy.sh", "remote.sh"):
            with self.subTest(script=name):
                r = run(name, "--help")
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertIn("Usage:", r.stdout)

    def test_unknown_option_fails(self):
        for name in ("test.sh", "deploy.sh", "remote.sh"):
            with self.subTest(script=name):
                self.assertNotEqual(run(name, "--bogus").returncode, 0)

    def test_remote_requires_a_subcommand(self):
        self.assertEqual(run("remote.sh", env_text=GOOD_ENV).returncode, 2)

    def test_check_env_reports_a_result(self):
        r = run("check_env.sh")
        self.assertIn("Result:", r.stdout)
        self.assertEqual(r.returncode == 0, "Result: OK" in r.stdout)
        self.assertIn("python3", r.stdout)


class DeployCommandTests(unittest.TestCase):
    def test_default_is_a_dry_run_without_delete(self):
        r = run("deploy.sh", "--show-command", env_text=GOOD_ENV)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn("rsync", out)
        self.assertIn("--dry-run", out)
        self.assertNotIn("--delete", out)
        self.assertIn("ubuntu@raspberrypi.local:lidar-mapping-robot/", out)
        self.assertIn("BatchMode=yes", out)
        for excluded in ("scripts/pi.env", ".git/", "__pycache__/"):
            self.assertIn(excluded, out)

    def test_apply_removes_dry_run(self):
        out = run("deploy.sh", "--show-command", "--apply", env_text=GOOD_ENV).stdout
        self.assertNotIn("--dry-run", out)
        self.assertIn("rsync", out)

    def test_port_is_passed_through(self):
        out = run("deploy.sh", "--show-command",
                  env_text=GOOD_ENV.replace("PI_SSH_PORT=22", "PI_SSH_PORT=2222")).stdout
        self.assertIn("-p 2222", out.replace("\\ ", " "))

    def test_missing_env_file_is_reported_without_connecting(self):
        r = run("deploy.sh", "--show-command", env_missing=True)
        self.assertEqual(r.returncode, 1)
        self.assertIn("missing", r.stderr)


class RemoteCommandTests(unittest.TestCase):
    def test_info_is_read_only(self):
        r = run("remote.sh", "info", "--show-command", env_text=GOOD_ENV)
        self.assertEqual(r.returncode, 0, r.stderr)
        for needle in ("ssh", "BatchMode=yes", "uname", "python3"):
            self.assertIn(needle, r.stdout)
        for banned in ("sudo", "ros2", "cmd_vel", "rm ", "launch", "kill"):
            self.assertNotIn(banned, r.stdout)

    def test_test_subcommand_runs_only_the_unit_tests(self):
        out = run("remote.sh", "test", "--show-command", env_text=GOOD_ENV).stdout
        self.assertIn("unittest", out)
        self.assertIn("tests/unit", out)
        self.assertIn("lidar-mapping-robot", out)
        for banned in ("sudo", "ros2", "launch", "ros2_integration"):
            self.assertNotIn(banned, out)

    def test_ssh_options_end_before_the_destination(self):
        words = run("remote.sh", "info", "--show-command", env_text=GOOD_ENV).stdout.replace("\\ ", "\x00").split()
        self.assertLess(words.index("--"), words.index("ubuntu@raspberrypi.local"))


class EnvFileHardeningTests(unittest.TestCase):
    def assertRejected(self, text, expect_in_stderr=""):
        for script, args in (("deploy.sh", ("--show-command",)), ("remote.sh", ("info", "--show-command"))):
            with self.subTest(script=script, env=text):
                r = run(script, *args, env_text=text)
                self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
                self.assertEqual(r.stdout, "")                      # nothing was printed to run
                self.assertIn(expect_in_stderr, r.stderr)

    def test_hostile_hosts_rejected(self):
        for host in ("-oProxyCommand=evil", "a b", "host;id", "host$(id)", "host`id`", "", "ho|st", "host\\"):
            self.assertRejected(GOOD_ENV.replace("raspberrypi.local", host), "PI_HOST")

    def test_hostile_users_rejected(self):
        for user in ("Ubuntu", "a;b", "-l", "", "us er", "$(id)"):
            self.assertRejected(GOOD_ENV.replace("ubuntu", user), "PI_USER")

    def test_hostile_directories_rejected(self):
        for d in ("../etc", "a/../b", "a;b", "a b", "$(id)", "", "-rf", "a'b", 'a"b'):
            self.assertRejected(GOOD_ENV.replace("lidar-mapping-robot", d), "PI_REMOTE_DIR")

    def test_bad_ports_rejected(self):
        for port in ("0", "65536", "22; id", "abc", "", "-1", "999999"):
            self.assertRejected(GOOD_ENV.replace("PI_SSH_PORT=22", f"PI_SSH_PORT={port}"), "PI_SSH_PORT")

    def test_unsupported_lines_rejected(self):
        for line in ("PI_EVIL=1", "rm -rf /", "export PI_HOST=x", "PI_HOST = x", "$(touch x)"):
            self.assertRejected(GOOD_ENV + line + "\n", "unsupported line")

    def test_env_file_is_parsed_not_executed(self):
        with tempfile.TemporaryDirectory() as tmp:
            sentinel = Path(tmp) / "executed"
            text = GOOD_ENV + f"touch {sentinel}\n" + f'PI_HOST=x$(touch {sentinel})\n'
            r = run("deploy.sh", "--show-command", env_text=text)
            self.assertEqual(r.returncode, 1)
            self.assertFalse(sentinel.exists())
            text = GOOD_ENV.replace("raspberrypi.local", f'ok"; touch {sentinel}; "')
            run("deploy.sh", "--show-command", env_text=text)
            self.assertFalse(sentinel.exists())

    def test_comments_blank_lines_and_quotes_are_accepted(self):
        text = '# a comment\n\nPI_HOST="pi.example"\nPI_USER=pi\nPI_REMOTE_DIR=/home/pi/robot\nPI_SSH_PORT=2200\n'
        r = run("remote.sh", "info", "--show-command", env_text=text)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("pi@pi.example", r.stdout)

    def test_last_line_without_trailing_newline_is_read(self):
        r = run("remote.sh", "info", "--show-command", env_text=GOOD_ENV.rstrip("\n"))
        self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main()
