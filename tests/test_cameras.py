"""Missing configured cameras must never enter blocking SDK startup."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from dual_franka.capture.cameras import Cameras


class CameraDiscoveryTests(unittest.TestCase):
    def sdk(self, serials):
        devices = [SimpleNamespace(get_info=Mock(return_value=serial)) for serial in serials]
        return SimpleNamespace(
            context=Mock(return_value=SimpleNamespace(query_devices=lambda: devices)),
            camera_info=SimpleNamespace(serial_number="serial"),
            pipeline=Mock(),
        )

    def test_missing_cameras_are_reported_without_pipeline_startup(self):
        sdk = self.sdk([])
        with (
            patch.dict("sys.modules", {"pyrealsense2": sdk}),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            cameras = Cameras({"left": "missing-left", "right": "missing-right"}, Mock())
            cameras.close()
        sdk.pipeline.assert_not_called()
        self.assertEqual(set(cameras.errors), {"left", "right"})
        self.assertIn("not connected", cameras.errors["left"])

    def test_available_camera_starts_even_when_another_is_missing(self):
        sdk = self.sdk(["present"])
        with (
            patch.dict("sys.modules", {"pyrealsense2": sdk}),
            patch.object(Cameras, "_capture") as capture,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            cameras = Cameras({"left": "absent", "right": "present"}, Mock())
            cameras.close()
        capture.assert_called_once_with("right", "present", 1280, 720, 30)
        self.assertEqual(set(cameras.errors), {"left"})

    def test_no_camera_configuration_needs_no_sdk(self):
        with patch.dict("sys.modules", {"pyrealsense2": None}):
            cameras = Cameras({}, Mock())
            cameras.close()
        self.assertEqual(cameras.errors, {})
