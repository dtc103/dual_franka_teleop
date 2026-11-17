"""Connection recovery tests; no ROS, Quest or robot required."""

import contextlib
import io
import json
import socket
import time
import unittest
import urllib.error
import urllib.request
from unittest.mock import Mock

from dual_franka.quest.diagnostics import ConnectionJournal
from dual_franka.quest.reader import WebXRReader
from dual_franka.quest.setup import USBConnection


def frame(sequence=0, source="headset"):
    return {
        "reference_space": "local-floor",
        "source": source,
        "epoch": 0,
        "sequence": sequence,
        "controllers": {"left": None, "right": None},
    }


class RelayTests(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.reader = WebXRReader(port=0)
        self.port = self.reader.server.server_port
        self.base = f"http://127.0.0.1:{self.port}"
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def tearDown(self):
        self.reader.stop()
        self.output.__exit__(None, None, None)

    def get(self, path):
        with self.http.open(self.base + path, timeout=2) as response:
            return json.load(response)

    def post(self, path, data):
        request = urllib.request.Request(
            self.base + path, json.dumps(data).encode(), {"Content-Type": "application/json"}
        )
        with self.http.open(request, timeout=2) as response:
            return response.status

    def test_restart_rebinds_same_port_and_accepts_existing_browser_sequence(self):
        self.post("/pose", frame(500))
        self.reader.stop()
        self.reader = WebXRReader(port=self.port)
        self.post("/pose", frame(501))
        self.assertEqual(self.reader.snapshot()["sequence"], 501)

    def test_idle_browser_preconnections_do_not_block_page_or_frames(self):
        connections = [
            socket.create_connection(("127.0.0.1", self.port), timeout=1) for _ in range(12)
        ]
        try:
            self.assertEqual(self.post("/pose", frame()), 204)
            self.assertTrue(self.get("/health")["streaming"])
        finally:
            for connection in connections:
                connection.close()

    def test_browser_heartbeat_is_distinct_from_controller_stream(self):
        self.post(
            "/client-status", {"client": "test", "session": "not-started", "visibility": "visible"}
        )
        health = self.get("/health")
        self.assertFalse(health["streaming"])
        self.assertEqual(health["browser"]["session"], "not-started")
        self.post("/pose", frame())
        health = self.get("/health")
        self.assertTrue(health["streaming"])
        self.assertEqual(health["tracked"], [])

    def test_stale_frames_are_never_reported_as_tracking(self):
        self.reader.state.stale_seconds = 0.001
        self.post("/pose", frame())
        time.sleep(0.01)
        self.assertFalse(self.get("/health")["streaming"])
        self.assertIsNone(self.reader.snapshot())

    def test_conflicting_live_browser_is_reported_and_cannot_replace_input(self):
        self.post("/pose", frame())
        with self.assertRaises(urllib.error.HTTPError) as error:
            self.post("/pose", frame(source="other-tab"))
        self.assertEqual(error.exception.code, 400)
        self.assertEqual(self.reader.snapshot()["source"], "headset")

    def test_head_request_and_clock_cache_buster(self):
        request = urllib.request.Request(self.base + "/app.js", method="HEAD")
        with self.http.open(request, timeout=2) as response:
            self.assertGreater(int(response.headers["Content-Length"]), 100)
            self.assertEqual(response.read(), b"")
        self.assertGreater(self.get("/clock?probe=1")["host_monotonic_ns"], 0)


class USBTests(unittest.TestCase):
    def connection(self):
        connection = USBConnection(8443, Mock(spec=ConnectionJournal))
        connection.adb = Mock()
        return connection

    def devices(self, transport="4", duplicate=True):
        lines = f"List of devices attached\nQUEST device usb:2-1 transport_id:{transport}\n"
        if duplicate:
            lines += "QUEST no permissions usb:2-1 transport_id:3\n"
        return lines

    def test_uses_authorized_transport_despite_duplicate_permission_error(self):
        connection = self.connection()
        connection.adb.side_effect = [self.devices(), "UsbFfs tcp:8443 tcp:8443\n", ""]
        connection.check()
        self.assertEqual(connection.transport, "4")
        self.assertEqual(connection.adb.call_args_list[1].kwargs, {"transport": "4"})

    def test_removed_forward_is_restored_without_script_restart(self):
        connection = self.connection()
        connection.transport = "4"
        connection.adb.side_effect = [self.devices(), "", "8443", ""]
        connection.check()
        connection.adb.assert_any_call("reverse", "tcp:8443", "tcp:8443", transport="4")

    def test_replug_selects_new_transport_and_restores_forwarding(self):
        connection = self.connection()
        connection.transport = "4"
        connection.adb.side_effect = [self.devices("5"), "", "8443", ""]
        connection.check()
        self.assertEqual(connection.transport, "5")
        connection.adb.assert_any_call("reverse", "tcp:8443", "tcp:8443", transport="5")

    def test_healthy_usb_is_not_reconfigured_or_browser_reopened(self):
        connection = self.connection()
        connection.transport = "4"
        connection.adb.side_effect = [self.devices(), "UsbFfs tcp:8443 tcp:8443\n"]
        connection.check()
        self.assertEqual(connection.adb.call_count, 2)

    def test_unauthorized_device_has_clear_message(self):
        connection = self.connection()
        connection.adb.return_value = (
            "List of devices attached\nQUEST unauthorized transport_id:3\n"
        )
        with self.assertRaisesRegex(RuntimeError, "USB debugging unauthorized"):
            connection.check()

    def test_linux_permissions_are_distinguished_from_headset_authorization(self):
        connection = self.connection()
        connection.adb.return_value = (
            "List of devices attached\nQUEST no permissions usb:2-1 transport_id:3\n"
        )
        with self.assertRaisesRegex(RuntimeError, "51-meta-quest.rules"):
            connection.check()

    def test_multiple_devices_require_selection(self):
        connection = self.connection()
        connection.adb.return_value = (
            self.devices(duplicate=False) + "OTHER device transport_id:8\n"
        )
        with self.assertRaisesRegex(RuntimeError, "--quest-serial"):
            connection.check()
