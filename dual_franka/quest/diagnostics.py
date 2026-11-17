"""Small connection journal: state changes only, never controller poses."""

import json
import threading
from datetime import datetime, timezone
from pathlib import Path


class ConnectionJournal:
    def __init__(self, path=None):
        self.lock = threading.Lock()
        self.path = None if path is None else Path(path)
        self.previous = {}
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def event(self, kind, **details):
        with self.lock:
            if self.previous.get(kind) == details:
                return
            self.previous[kind] = details
            event = {"time": datetime.now(timezone.utc).isoformat(), "event": kind, **details}
            if self.path:
                with self.path.open("a") as stream:
                    stream.write(json.dumps(event) + "\n")
            print("Quest: " + kind + " · " + json.dumps(details), flush=True)
