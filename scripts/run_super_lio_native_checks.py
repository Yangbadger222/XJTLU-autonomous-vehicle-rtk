#!/usr/bin/env python3
"""Compile independent probes against this workspace's actual native objects."""
import argparse
import hashlib
import json
import shlex
import subprocess
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--only',choices=('information','failed_update','deskew','packet'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    build=args.workspace/'build/super_lio'
    make=(build/'CMakeFiles/lio.dir/flags.make').read_text().splitlines()
    flags={line.split(' = ',1)[0]:shlex.split(line.split(' = ',1)[1]) for line in make if ' = ' in line}
    link=shlex.split((build/'CMakeFiles/super_lio_node.dir/link.txt').read_text())
    object_index=next(i for i,part in enumerate(link) if part.endswith('src/apps/super_lio_node.cpp.o'))
    output_index=link.index('-o')+1
    checks=[]
    for name in ((args.only,) if args.only else ('information','failed_update','deskew','packet')):
        source=args.repo/f'scripts/validate_super_lio_{name}.cpp'
        object_file=args.output/f'{name}.o';binary=args.output/name
        compile_cmd=[link[0],*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],
                     '-std=c++20','-O1','-UNDEBUG','-c',str(source),'-o',str(object_file)]
        link_cmd=list(link);link_cmd[object_index]=str(object_file);link_cmd[output_index]=str(binary)
        log=args.output/f'{name}.log'
        with log.open('w') as stream:
            subprocess.run(compile_cmd,cwd=build,stdout=stream,stderr=subprocess.STDOUT,check=True)
            subprocess.run(link_cmd,cwd=build,stdout=stream,stderr=subprocess.STDOUT,check=True)
            subprocess.run([str(binary)],stdout=stream,stderr=subprocess.STDOUT,check=True)
        checks.append({'name':name,'status':'PASS','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                       'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'log':str(log),
                       'link_scope':'actual built liblio and original CMake native dependencies'})
        print(log.read_text()[-1800:])
    result={'status':'PASS','checks':checks,'workspace':str(args.workspace),'scope':'native software equations and failure paths; synthetic inputs do not prove physical acceptance'}
    (args.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
