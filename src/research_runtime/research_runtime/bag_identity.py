"""Sealed raw CDR identity shared by catalog and owned player."""
import hashlib
import json
from pathlib import Path
import sqlite3

import yaml

RAW_TYPES = {"/livox/lidar": "livox_ros_driver2/msg/CustomMsg",
             "/livox/imu": "sensor_msgs/msg/Imu"}


def raw_fingerprint(directory: Path, info: dict) -> dict:
    paths = [directory / name for name in info["relative_file_paths"]]
    if any(not path.resolve().is_relative_to(directory.resolve()) for path in paths):
        raise ValueError("bag storage must stay within the cataloged directory")
    signatures = {name: hashlib.sha256() for name in RAW_TYPES}
    counts = {name: 0 for name in RAW_TYPES}
    identity = []
    for path in paths:
        before = path.stat()
        with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
            topics = {name: (identity, kind) for identity, name, kind in
                      db.execute("SELECT id,name,type FROM topics")}
            for name, kind in RAW_TYPES.items():
                if name not in topics or topics[name][1] != kind:
                    raise ValueError("raw sensor topic/type missing: " + name)
                for (blob,) in db.execute(
                        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                        (topics[name][0],)):
                    signatures[name].update(len(blob).to_bytes(8, "little"))
                    signatures[name].update(blob)
                    counts[name] += 1
        storage_hash=hashlib.sha256()
        with path.open("rb") as source:
            for block in iter(lambda:source.read(4*1024*1024),b""):storage_hash.update(block)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError("bag changed while hashing; not sealed")
        identity.append({"file": path.name, "size_bytes": after.st_size,
                         "mtime_ns": after.st_mtime_ns,"storage_sha256":storage_hash.hexdigest()})
    streams = {name: {"type": RAW_TYPES[name], "count": counts[name],
                       "ordered_cdr_sha256": signatures[name].hexdigest()}
               for name in RAW_TYPES}
    if not all(counts.values()):
        raise ValueError("empty raw sensor stream")
    declared={entry["topic_metadata"]["name"]:entry["message_count"] for entry in info["topics_with_message_count"]}
    if any(declared.get(name)!=counts[name] for name in RAW_TYPES):
        raise ValueError("raw storage count differs from sealed metadata")
    return {"streams": streams, "files": identity,
            "raw_input_sha256": hashlib.sha256(
                json.dumps(streams, sort_keys=True).encode()).hexdigest()}



def verify_cataloged_bag(row):
    directory=Path(row["bag_path"])
    metadata=directory/"metadata.yaml"
    content=metadata.read_bytes()
    if hashlib.sha256(content).hexdigest()!=row["metadata_sha256"]:
        raise ValueError("metadata identity changed since catalog")
    try:
        info=yaml.safe_load(content)["rosbag2_bagfile_information"]
        result=raw_fingerprint(directory,info)
    except (sqlite3.Error,yaml.YAMLError,KeyError,TypeError) as exc:
        raise ValueError("unreadable cataloged storage: "+str(exc)) from exc
    if result["files"]!=row.get("files"):
        raise ValueError("sealed bag storage identity changed since catalog; rebuild catalog if missing storage SHA")
    if metadata.read_bytes()!=content or result["raw_input_sha256"]!=row["raw_input_sha256"]:
        raise ValueError("raw input identity changed since catalog")
    return directory
