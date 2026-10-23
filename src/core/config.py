"""Lightweight configuration loaders (stdlib only)."""

import json
import os


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def save_json(path, data):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def parse_ini(text):
    result = {}
    section = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith(";"):
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip()
            result.setdefault(section, {})
            continue
        if "=" in line:
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip()
            target = result if section is None else result[section]
            # type coercion
            if val.lower() in ("true", "false"):
                val = val.lower() == "true"
            else:
                try:
                    if "." in val:
                        val = float(val)
                    else:
                        val = int(val)
                except ValueError:
                    pass
            target[key] = val
    return result


def load_ini(path):
    with open(path, "r") as f:
        return parse_ini(f.read())
