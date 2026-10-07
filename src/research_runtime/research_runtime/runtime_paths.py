"""Task-specific persistence; never writes current_scene or production latest."""
import os
from pathlib import Path


def research_path(value):
    path = Path(value)
    if path.is_absolute():
        return path
    runtime_root = os.environ.get('FYP_RUNTIME_ROOT')
    if runtime_root:
        # Existing configs retain their conventional relative names, while a
        # test/session runtime root owns the actual writes.
        parts = path.parts
        if parts[:3] == ('runtime-data','research','active_road'):
            path = Path(*parts[3:])
        return Path(runtime_root)/'research/active_road'/path
    return path
