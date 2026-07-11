from __future__ import annotations


def parse_gga_quality(sentence: str) -> int | None:
    """Return the GGA quality field only for a complete, checksum-valid sentence."""
    if not isinstance(sentence, str):
        return None
    clean = sentence.strip()
    if not clean.startswith("$") or clean.count("*") != 1:
        return None
    payload, transmitted = clean[1:].split("*", 1)
    if len(transmitted) != 2:
        return None
    try:
        expected = int(transmitted, 16)
    except ValueError:
        return None
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    if checksum != expected:
        return None
    fields = payload.split(",")
    if len(fields) < 7 or not fields[0].endswith("GGA") or not fields[6]:
        return None
    try:
        quality = int(fields[6])
    except ValueError:
        return None
    return quality if 0 <= quality <= 9 else None
