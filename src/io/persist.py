"""Persistence helpers for DegeFS (stdlib only)."""

import json
import os


STATE_DIR = os.path.join(os.path.expanduser("~"), ".degefs")
STATE_FILE = os.path.join(STATE_DIR, "state.json")


def load_state():
    try:
        with open(STATE_FILE, "r") as f:
            return json.load(f)
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def save_state(data):
    try:
        os.makedirs(STATE_DIR, exist_ok=True)
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, STATE_FILE)
        return True
    except OSError:
        return False
