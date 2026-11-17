"""Background camera restart/acquisition; missing devices never gate teleoperation."""

import threading


class CameraManager:
    def __init__(self, config, sink, reset=True):
        self.config, self.sink = config, sink
        self.capture = None
        self.message = "Preparing cameras"
        self.error = ""
        self.stop_event = threading.Event()
        self.restart_event = threading.Event()
        self.reset_enabled = reset
        self.thread = None
        self.restart_event.set()

    def start(self):
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._run, name="camera-manager", daemon=True)
        self.thread.start()

    @property
    def metadata(self):
        return {} if self.capture is None else self.capture.metadata

    @property
    def latest(self):
        return {} if self.capture is None else self.capture.latest

    @property
    def received(self):
        return {} if self.capture is None else self.capture.received

    @property
    def errors(self):
        return (
            {role: self.error for role in self.config.cameras}
            if self.capture is None
            else self.capture.errors
        )

    def restart(self):
        self.restart_event.set()

    def _hardware_reset(self):
        import pyrealsense2 as rs

        context = rs.context()
        devices = {
            device.get_info(rs.camera_info.serial_number): device
            for device in context.query_devices()
        }
        restarted = 0
        for role, serial in self.config.cameras.items():
            if serial in devices:
                self.message = f"Restarting {role} camera"
                devices[serial].hardware_reset()
                restarted += 1
        devices.clear()
        if restarted:
            self.message = "Waiting 10 seconds for cameras to restart"
            self.stop_event.wait(10)

    def _run(self):
        while not self.stop_event.is_set():
            if not self.restart_event.wait(0.2):
                continue
            self.restart_event.clear()
            if self.capture is not None:
                self.capture.close()
                self.capture = None
            try:
                from dual_franka.capture.cameras import Cameras

                if self.reset_enabled and self.config.cameras:
                    self._hardware_reset()
                if self.stop_event.is_set():
                    return
                self.capture = Cameras(
                    self.config.cameras,
                    self.sink,
                    self.config.camera_width,
                    self.config.camera_height,
                    self.config.camera_fps,
                )
                self.message, self.error = (
                    "Camera streams requested; offline views do not block control",
                    "",
                )
            except Exception as error:
                self.error = str(error)
                self.message = f"Cameras unavailable: {error}"
                print(self.message, flush=True)

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(5)
        if self.capture is not None:
            self.capture.close()
