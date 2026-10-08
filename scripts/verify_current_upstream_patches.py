#!/usr/bin/env python3
"""Apply every current patch to fresh pinned clones and compare runtime source."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def git(directory, *args):
    return subprocess.check_output(['git', '-C', str(directory), *args], text=True).strip()


def source_manifest(directory):
    paths = set(git(directory, 'ls-files').splitlines())
    paths.update(git(directory, 'ls-files', '--others', '--exclude-standard').splitlines())
    return {path: hashlib.sha256((directory/path).read_bytes()).hexdigest()
            for path in sorted(paths) if (directory/path).is_file()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--verification-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    # New audit assets only; never reset or clean an existing dependency.
    args.verification_root.mkdir(parents=True, exist_ok=False)
    recipe = (
        ('super_lio', 'f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2', 'super_lio', 2),
        ('ego_planner_2d_ros2', '7f5be6d4cee34871e85aa1f15285cfaf17b23877', 'ego_planner_2d', 10))
    results = {}
    for name, pinned, patch_directory, count in recipe:
        built = args.repo/'src'/name
        checkout = args.verification_root/name
        if git(built, 'rev-parse', 'HEAD') != pinned:
            raise RuntimeError('runtime dependency has wrong pin')
        subprocess.run(['git', 'clone', '--shared', '--no-checkout', str(built), str(checkout)], check=True)
        subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', pinned], check=True)
        patches = sorted((args.repo/'patches'/patch_directory).glob('*.patch'))
        if len(patches) != count or [p.name[:4] for p in patches] != [f'{index:04}' for index in range(1, count+1)]:
            raise RuntimeError('missing, extra or unordered current patch')
        entries = []
        for patch in patches:
            subprocess.run(['git', '-C', str(checkout), 'apply', '--check', str(patch)], check=True)
            subprocess.run(['git', '-C', str(checkout), 'apply', str(patch)], check=True)
            entries.append({'path': str(patch.relative_to(args.repo)),
                            'sha256': hashlib.sha256(patch.read_bytes()).hexdigest(), 'check_apply': 'PASS'})
        expected, actual = source_manifest(checkout), source_manifest(built)
        differences = [path for path in sorted(set(expected)|set(actual)) if expected.get(path) != actual.get(path)]
        results[name] = {'pinned_commit': pinned, 'patches': entries, 'sequential_apply': 'PASS',
                         'matches_built_source': not differences, 'differences': differences,
                         'source_sha256': expected}
    result = {'status': 'PASS' if all(item['matches_built_source'] for item in results.values()) else 'FAIL',
              'research_source_commit': git(args.repo, 'rev-parse', 'HEAD'), 'dependencies': results,
              'scope': 'fresh exact-pin sequential patches and actual runtime checkout byte equality; compile/replay are separate receipts'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'status': result['status'], 'research_source_commit': result['research_source_commit'],
                      'dependencies': {name: {'patch_count': len(value['patches']), 'differences': value['differences']}
                                       for name, value in results.items()}}, indent=2))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
