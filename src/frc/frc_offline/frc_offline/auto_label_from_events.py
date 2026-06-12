"""auto_label_from_events：人工复核结果回灌（标签质量闭环 §5.6 的治理步）。

流程：failure_miner 产出 events.jsonl -> contact_sheet 生成拼图 + review.csv ->
人工勾选（keep=n 表示误报、补录行表示漏报）-> 本脚本合成 cleaned_events.jsonl，
并输出 E1 表格需要的三个数字：自动挖掘 precision（按复核口径）、
归因过滤比例（由 dataset 导出时统计）、人工改动率。

review.csv 列：event_index, keep(y/n), comment
（contact_sheet 生成模板，复核人只改 keep / 追加行）
"""

import argparse
import csv
import json
from pathlib import Path


def apply_review(events_path, review_path, out_path):
    with open(events_path) as f:
        events = [json.loads(line) for line in f if line.strip()]

    kept, dropped, added = [], 0, 0
    if Path(review_path).exists():
        with open(review_path) as f:
            rows = list(csv.DictReader(f))
        decisions = {int(r["event_index"]): r for r in rows
                     if r.get("event_index", "").strip().isdigit()}
        for i, e in enumerate(events):
            d = decisions.get(i)
            if d is not None and d.get("keep", "y").strip().lower() == "n":
                dropped += 1
                continue
            if d is not None and d.get("comment", "").strip():
                e["review_comment"] = d["comment"].strip()
            kept.append(e)
        # 复核中人工补录的漏报（event_index 留空、带 stamp/type/severity 列）
        for r in rows:
            if not r.get("event_index", "").strip() and \
                    r.get("stamp", "").strip():
                kept.append({
                    "stamp": float(r["stamp"]), "type": r.get("type", "manual"),
                    "severity": r.get("severity", "gold"),
                    "note": r.get("comment", "review_added"),
                    "source": "review", "map_x": None, "map_y": None,
                })
                added += 1
    else:
        kept = events

    with open(out_path, "w") as f:
        for e in kept:
            f.write(json.dumps(e) + "\n")

    n_auto = sum(1 for e in events if e.get("source") == "miner")
    n_auto_kept = sum(1 for e in kept if e.get("source") == "miner")
    stats = {
        "total_mined": len(events),
        "kept": len(kept),
        "dropped_false_positive": dropped,
        "review_added_miss": added,
        "auto_precision": round(n_auto_kept / n_auto, 3) if n_auto else None,
        "human_edit_rate": round((dropped + added) / max(len(events), 1), 3),
    }
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--events", required=True)
    ap.add_argument("--review", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    out = args.out or str(Path(args.events).with_name("cleaned_events.jsonl"))
    stats = apply_review(args.events, args.review, out)
    print(json.dumps(stats, indent=2, ensure_ascii=False))
    print(f"cleaned events -> {out}")


if __name__ == "__main__":
    main()
