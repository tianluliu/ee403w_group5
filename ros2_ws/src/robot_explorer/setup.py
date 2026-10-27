from setuptools import setup

package_name = "robot_explorer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LiDAR Robot Team",
    maintainer_email="team@example.invalid",
    description="Frontier-based autonomous exploration for the LiDAR mapping robot.",
    license="TODO: team to choose a license",
    entry_points={
        "console_scripts": [
            "explorer_node = robot_explorer.explorer_node:main",
        ],
    },
)
