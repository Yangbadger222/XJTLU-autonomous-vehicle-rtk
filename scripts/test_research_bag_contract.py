import json
from pathlib import Path
import sqlite3
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from catalog_research_bags import catalog, RAW_TYPES
from raw_replay_contract import replay_contract
from research_runtime.bag_identity import verify_cataloged_bag


def fixture(path, duration=418.53, shift=0):
    path.mkdir(parents=True)
    with sqlite3.connect(path/"raw.db3") as db:
        db.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)")
        db.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
        for index, (name, kind) in enumerate(RAW_TYPES.items(), 1):
            db.execute("INSERT INTO topics VALUES(?,?,?)", (index, name, kind))
            db.execute("INSERT INTO messages VALUES(?,?,?,?)", (index, index, shift+index, b"same-payload"+bytes([index])))
    info={"storage_identifier":"sqlite3","duration":{"nanoseconds":int(duration*1e9)},
          "message_count":2,"relative_file_paths":["raw.db3"],"topics_with_message_count":[
              {"topic_metadata":{"name":name,"type":kind},"message_count":1} for name,kind in RAW_TYPES.items()]}
    meta=path/"metadata.yaml";meta.write_text(yaml.safe_dump({"rosbag2_bagfile_information":info}))
    return {"metadata_path":str(meta)}


def test_full_recording_budget_scales_past_old_420_seconds(tmp_path):
    fixture(tmp_path/"raw")
    contract=replay_contract(tmp_path/"raw")
    assert contract["wall_budget_s"] > 418.53+2+20
    assert contract["expected_raw_counts"]=={"/livox/lidar":1,"/livox/imu":1}


def test_recording_times_or_replay_rates_do_not_create_independent_raw_inputs(tmp_path):
    original=fixture(tmp_path/"original")
    copy=fixture(tmp_path/"replay_05x",duration=900.,shift=4000)
    result=catalog([original,copy])
    assert result["distinct_raw_input_count"]==1
    assert result["selected_original_bags"]==[str(tmp_path/"original")]
    assert result["bags"][1]["selected_original"] is False
    assert len(result["bags"][0]["identical_raw_input_paths"])==1


def test_missing_raw_storage_refuses_full_replay(tmp_path):
    fixture(tmp_path/"raw")
    (tmp_path/"raw/raw.db3").unlink()
    with pytest.raises(ValueError,match="incomplete"):replay_contract(tmp_path/"raw")


def test_catalog_refuses_incomplete_count_metadata(tmp_path):
    record=fixture(tmp_path/"raw")
    with sqlite3.connect(tmp_path/"raw/raw.db3") as db:db.execute("DELETE FROM messages WHERE topic_id=1")
    result=catalog([record])
    assert result["distinct_raw_input_count"]==0
    assert result["bags"][0]["category"]=="UNREADABLE"


def test_player_rechecks_full_cdr_identity_after_catalog(tmp_path):
    record=fixture(tmp_path/"raw")
    row=catalog([record])["bags"][0]
    assert verify_cataloged_bag(row)==tmp_path/"raw"
    with sqlite3.connect(tmp_path/"raw/raw.db3") as db:db.execute("UPDATE messages SET data=? WHERE id=1",(b"changed",))
    with pytest.raises(ValueError,match="identity changed"):verify_cataloged_bag(row)
