"""Thread-safe routing of camera/Quest callbacks to the current session recorder."""

import threading


class RecordingSink:
    def __init__(self):
        self.lock = threading.Lock()
        self.recorder = None

    def attach(self, recorder):
        with self.lock:
            self.recorder = recorder

    def record(self, stream, data, received_ns=None):
        with self.lock:
            if self.recorder is not None:
                self.recorder.record(stream, data, received_ns)

    def image(self, role, color, depth, metadata):
        with self.lock:
            if self.recorder is not None:
                self.recorder.image(role, color, depth, metadata)

    def close(self):
        with self.lock:
            recorder, self.recorder = self.recorder, None
        if recorder is not None:
            recorder.close()
