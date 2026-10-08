import argparse,hashlib,importlib.util,json,platform
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--source",type=Path,required=True);p.add_argument("--output",type=Path,required=True);p.add_argument("--runtime-commit",required=True);a=p.parse_args()
checks=[];resolved={}
for name in ("research_runtime","super_lio_vehicle_adapter"):
 spec=importlib.util.find_spec(name)
 if spec is None or not spec.submodule_search_locations:raise RuntimeError("package unresolved: "+name)
 installed=Path(list(spec.submodule_search_locations)[0]);resolved[name]=str(installed)
 source=a.source/name/name
 for file in sorted(source.glob("*.py")):
  actual=installed/file.name
  sha=hashlib.sha256(file.read_bytes()).hexdigest()
  got=hashlib.sha256(actual.read_bytes()).hexdigest() if actual.is_file() else None
  checks.append({"package":name,"file":file.name,"source_sha256":sha,"installed_sha256":got,
                 "status":"PASS" if sha==got else "FAIL"})
d={"status":"PASS" if checks and all(c["status"]=="PASS" for c in checks) else "FAIL",
   "runtime_commit":a.runtime_commit,"architecture":platform.machine(),"package_paths":resolved,
   "fresh_sdk14_source_commit":"8c937ed5c242fc3e8a08cedcd35c736b957ac43b",
   "fresh_before_build":False,"scope":"previous qualified SDK14 foundation plus isolated Python package overlays; no all14 clean build at current tip",
   "checks":checks}
with a.output.open("x") as out:out.write(json.dumps(d,indent=2)+"\n")
print(json.dumps({"status":d["status"],"architecture":d["architecture"],"file_count":len(checks)}))
raise SystemExit(0 if d["status"]=="PASS" else 1)
