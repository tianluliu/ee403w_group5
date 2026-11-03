"""Read-only TF probes. Lookup callbacks must be nonblocking (zero timeout)."""


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def pose_fields(msg):
    p = msg.pose
    return {"frame_id": msg.header.frame_id, "stamp_ns": stamp_ns(msg.header.stamp),
            "position": {k: getattr(p.position, k) for k in ("x", "y", "z")},
            "orientation": {k: getattr(p.orientation, k) for k in ("x", "y", "z", "w")}}


def result_fields(result):
    # Missing fields stay null; never synthesize an error message.
    return {"error_code": getattr(result, "error_code", None),
            "error_msg": getattr(result, "error_msg", None)}


def tf_snapshot(lookup, now_ns, at_ns, odometry_stamp_ns=None, plan=None):
    """Probe this observer's buffer, not the controller's private TF buffer."""
    def probe(parent, child, when):
        try:
            transform = lookup(parent, child, when)
            stamp = stamp_ns(transform.header.stamp)
            t = transform.transform
            return {"available": True, "stamp_ns": stamp,
                    "age_s": (now_ns - stamp) / 1e9,
                    "translation": {k: getattr(t.translation, k) for k in ("x", "y", "z")},
                    "rotation": {k: getattr(t.rotation, k) for k in ("x", "y", "z", "w")}}
        except Exception as exc:
            return {"available": False, "exception": type(exc).__name__, "error": str(exc)}

    edges = {}
    for parent, child in (("map", "odom"), ("odom", "base_link"), ("base_link", "laser")):
        static = child == "laser"
        edges[f"{parent}->{child}"] = {
            "static": static, "latest": probe(parent, child, 0),
            "at_now": probe(parent, child, now_ns)}
        if static:
            # A static transform's timestamp is not a freshness measurement.
            for value in edges[f"{parent}->{child}"].values():
                if isinstance(value, dict):
                    value.pop("age_s", None)
    odom_tf = edges["odom->base_link"]["latest"]
    checks = {"map_to_odom_at_robot_tf_stamp": (
        probe("map", "odom", odom_tf["stamp_ns"]) if odom_tf["available"] else None)}
    if plan is not None:
        checks["plan_to_odom_at_endpoint_stamp"] = probe(
            "odom", plan["frame_id"], plan["endpoint_stamp_ns"])
    return {"ros_now_ns": now_ns, "requested_stamp_ns": at_ns,
            "edges": edges, "robot_in_map_latest": probe("map", "base_link", 0),
            "map_to_odom_at_requested_stamp": probe("map", "odom", at_ns),
            "odometry_stamp_ns": odometry_stamp_ns,
            "odometry_age_s": None if odometry_stamp_ns is None else (now_ns - odometry_stamp_ns) / 1e9,
            "odom_tf_minus_odometry_s": None if odometry_stamp_ns is None or not odom_tf["available"]
            else (odom_tf["stamp_ns"] - odometry_stamp_ns) / 1e9,
            "plan": plan, "checks": checks}
