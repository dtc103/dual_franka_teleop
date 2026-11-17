"""Maintain USB forwarding across unplug/replug and ADB transport changes."""

import subprocess
import threading


class USBConnection:
    def __init__(self, port, journal, serial=None, open_browser=False):
        self.port, self.journal, self.serial = port, journal, serial
        self.open_browser = open_browser
        self.opened = False
        self.transport = None
        self.status = "Checking USB…"
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name="quest-usb")

    def start(self):
        self.thread.start()

    def adb(self, *args, transport=None):
        if self.stopped.is_set():
            raise RuntimeError("USB monitor stopping")
        prefix = ["adb"] if transport is None else ["adb", "-t", transport]
        result = subprocess.run(prefix + list(args), capture_output=True, text=True, timeout=4)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        return result.stdout

    def check(self):
        devices = self.adb("devices", "-l")
        ready = []
        for line in devices.splitlines()[1:]:
            fields = line.split()
            if len(fields) < 2 or fields[1] != "device":
                continue
            if self.serial and fields[0] != self.serial:
                continue
            transport = next(
                (f.split(":", 1)[1] for f in fields if f.startswith("transport_id:")), None
            )
            if transport:
                ready.append((fields[0], transport))
        if not ready:
            self.transport = None
            if "no permissions" in devices:
                raise RuntimeError(
                    "Linux USB permission denied. Install setup/51-meta-quest.rules and reconnect USB."
                )
            if "unauthorized" in devices:
                raise RuntimeError(
                    "USB debugging unauthorized. Accept 'Always allow from this computer' in the Quest."
                )
            raise RuntimeError(
                "No authorized headset. Check USB debugging, cable and Linux USB permissions."
            )
        if len(ready) != 1:
            raise RuntimeError(
                "Multiple authorized Android transports; select the Quest with --quest-serial."
            )
        serial, transport = ready[0]
        # Select transport ID, not serial: stale/permission-denied transports can
        # appear alongside the working device with exactly the same serial.
        mappings = self.adb("reverse", "--list", transport=transport)
        expected = [f"tcp:{self.port}", f"tcp:{self.port}"]
        forwarded = any(line.split()[-2:] == expected for line in mappings.splitlines())
        changed = transport != self.transport
        if not forwarded:
            self.adb("reverse", *expected, transport=transport)
            self.journal.event(
                "usb_forward_restored", serial=serial, port=self.port, transport=transport
            )
        if changed or not forwarded:
            response = self.adb(
                "shell",
                "am",
                "broadcast",
                "-a",
                "com.oculus.vrpowermanager.prox_close",
                transport=transport,
            )
            # A successful broadcast request does not prove the XR session
            # stayed focused; record the response without claiming it did.
            self.journal.event("proximity_override_requested", response=response.strip())
        self.transport = transport
        if self.open_browser and not self.opened:
            self.adb(
                "shell",
                "am",
                "start",
                "-a",
                "android.intent.action.VIEW",
                "-d",
                f"http://localhost:{self.port}/",
                "-p",
                "com.oculus.browser",
                transport=transport,
            )
            self.opened = True
        self.status = f"USB ready · {serial} · port {self.port}"

    def run(self):
        while not self.stopped.is_set():
            try:
                self.check()
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                if self.stopped.is_set():
                    break
                self.status = f"USB unavailable: {error}"
            self.journal.event("usb_status", detail=self.status)
            self.stopped.wait(2)

    def stop(self):
        self.stopped.set()
        self.thread.join(timeout=5)
