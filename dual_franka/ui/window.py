"""Desktop controls; all ROS work and recording happen in Runtime's worker."""

from PySide6 import QtCore, QtGui, QtWidgets

STYLE = """
QWidget { background:#101720; color:#e7edf5; font:14px 'DejaVu Sans'; }
QGroupBox { border:1px solid #344357; border-radius:10px; margin-top:16px; padding:14px; }
QGroupBox::title { subcontrol-origin:margin; left:14px; }
QLineEdit { background:#1d2938; border:1px solid #344357; border-radius:6px; padding:8px; }
QPushButton { background:#293b50; border:0; border-radius:7px; padding:10px 15px; }
QPushButton:hover { background:#38536f; }
QPushButton:disabled { color:#64748b; background:#1b2734; }
QPushButton#primary { color:#10202b; background:#72d4ed; font-weight:bold; }
QLabel#heading { font-size:26px; font-weight:bold; }
"""


class CollectionWindow(QtWidgets.QWidget):
    def __init__(self, runtime, port):
        super().__init__()
        self.runtime = runtime
        self.closing = False
        teleop = runtime.teleop_only
        self.setWindowTitle(
            "Dual Panda teleoperation" if teleop else "Dual Panda demonstration collection"
        )
        self.setStyleSheet(STYLE)
        self.resize(1240, 520 if teleop else 900)
        self.setMinimumSize(940, 440 if teleop else 760)
        layout = QtWidgets.QVBoxLayout(self)
        heading = QtWidgets.QLabel(
            "Dual Panda · teleoperation" if teleop else "Dual Panda · demonstration collection"
        )
        heading.setObjectName("heading")
        layout.addWidget(heading)
        layout.addWidget(
            QtWidgets.QLabel(
                f"{'LIVE CONTROL' if runtime.execute else 'PREVIEW — no robot commands'}  ·  Quest: http://localhost:{port}/"
            )
        )
        self.quest_status = QtWidgets.QLabel("Quest: waiting for browser connection…")
        self.quest_status.setWordWrap(True)
        layout.addWidget(self.quest_status)
        task_box = QtWidgets.QGroupBox("Session")
        form = QtWidgets.QFormLayout(task_box)
        self.name = QtWidgets.QLineEdit()
        self.name.setPlaceholderText("Your session folder name")
        self.prompt = QtWidgets.QLineEdit()
        self.prompt.setPlaceholderText("Describe the task; optional alternative description")
        form.addRow("Session name", self.name)
        form.addRow("Task descriptions", self.prompt)
        self.paths = {}
        self.browse = []
        for hand, config in runtime.config.arms.items():
            row = QtWidgets.QHBoxLayout()
            field = QtWidgets.QLineEdit(str(config.reset_sequence or ""))
            self.paths[hand] = field
            button = QtWidgets.QPushButton("Choose…")
            button.clicked.connect(lambda _, hand=hand: self.choose_reset(hand))
            self.browse.append(button)
            row.addWidget(field)
            row.addWidget(button)
            form.addRow(f"{hand.capitalize()} reset path", row)
        self.prepare = QtWidgets.QPushButton("Prepare session")
        self.prepare.setObjectName("primary")
        self.prepare.clicked.connect(self.prepare_session)
        form.addRow(self.prepare)
        layout.addWidget(task_box)
        task_box.setVisible(not teleop)
        self.status = QtWidgets.QLabel("Connecting to robot drivers…")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        connection = QtWidgets.QHBoxLayout()
        self.reconnect = QtWidgets.QPushButton("Reconnect robots")
        self.reconnect.clicked.connect(self.reconnect_robots)
        connection.addWidget(self.reconnect)
        close = QtWidgets.QPushButton("Close program")
        close.clicked.connect(self.close)
        connection.addWidget(close)
        layout.addLayout(connection)
        self.counts = QtWidgets.QLabel("Episodes: 0")
        layout.addWidget(self.counts)
        self.counts.setVisible(not teleop)
        arms = QtWidgets.QHBoxLayout()
        self.arm_labels = {}
        for hand in runtime.config.arms:
            group = QtWidgets.QGroupBox(f"{hand.capitalize()} arm · {runtime.config.arms[hand].ip}")
            column = QtWidgets.QVBoxLayout(group)
            label = QtWidgets.QLabel("Waiting for state")
            label.setWordWrap(True)
            self.arm_labels[hand] = label
            column.addWidget(label)
            arms.addWidget(group)
        layout.addLayout(arms)
        buttons = QtWidgets.QHBoxLayout()
        self.actions = {}
        for key, text in [
            ("A", "A · Reset / start / success"),
            ("X", "X · Unsuccessful"),
            ("Y", "Y · Interrupted"),
        ]:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda _, key=key: runtime.command(key))
            self.actions[key] = button
            buttons.addWidget(button)
        self.close_session = QtWidgets.QPushButton("Close session")
        self.close_session.clicked.connect(lambda: runtime.command("close_session"))
        buttons.addWidget(self.close_session)
        layout.addLayout(buttons)
        for button in [*self.actions.values(), self.close_session]:
            button.setVisible(not teleop)
        self.location = QtWidgets.QLabel(str(runtime.output))
        self.location.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.location)
        layout.addWidget(
            QtWidgets.QLabel(
                "Own grip = clutch · own trigger = gripper · right thumbstick = both references · hold right B = precision for both"
            )
        )
        self.images = {}
        if not teleop:
            self.add_camera_previews(layout)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def add_camera_previews(self, layout):
        preview = QtWidgets.QGridLayout()
        for index, role in enumerate(self.runtime.config.cameras):
            label = QtWidgets.QLabel(f"{role}: waiting for camera")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumSize(260, 140)
            label.setStyleSheet("background:#192330; border-radius:8px;")
            self.images[role] = label
            preview.addWidget(label, index // 2, index % 2)
        layout.addLayout(preview, 1)

    def reconnect_robots(self):
        self.runtime.start()
        self.refresh()

    def choose_reset(self, hand):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, f"{hand} reset sequence", "", "JSON (*.json)"
        )
        if path:
            self.paths[hand].setText(path)

    def prepare_session(self):
        self.runtime.command(
            "prepare",
            name=self.name.text(),
            prompt=self.prompt.text(),
            reset_paths={hand: field.text().strip() for hand, field in self.paths.items()},
        )

    def refresh(self):
        reader = getattr(self.runtime, "reader", None)
        if reader is not None:
            health = reader.state.health()
            browser = health["browser"]
            usb = getattr(self, "quest_usb", None)
            connection = usb.status if usb else "USB monitor disabled"
            tracking = ", ".join(health["tracked"]) or "no controller poses"
            if (
                browser
                and health["browser_age_seconds"] < 5
                and browser.get("session") in ("hidden", "visible-blurred")
            ):
                detail = (
                    f"XR {browser['session']} · controller input paused by the headset; "
                    "check headset sleep/proximity and close system overlays"
                )
            elif health["streaming"]:
                detail = f"streaming · {tracking}"
            elif browser and health["browser_age_seconds"] < 5:
                detail = f"browser connected · XR {browser.get('session')} · {browser.get('error') or 'waiting for frames'}"
            else:
                detail = "no browser heartbeat · open the Quest page and start tracking"
            self.quest_status.setText(f"Quest: {connection}\n{detail}")
        state = self.runtime.status
        phase = state["phase"]
        alive = self.runtime.thread is not None and self.runtime.thread.is_alive()
        if self.closing:
            self.status.setText("Stopping both arms and flushing recordings…")
            if not alive:
                self.close()
            return
        self.status.setText(f"{phase.upper()} · {state['detail']}")
        self.reconnect.setEnabled(not alive)
        self.reconnect.setToolTip(
            "Reconnect after restoring robot readiness; this does not run error recovery"
        )
        editable = phase in ("unprepared", "closed", "connecting", "fault")
        for widget in [self.name, self.prompt, *self.paths.values(), *self.browse]:
            widget.setEnabled(editable)
        self.prepare.setEnabled(alive and phase in ("unprepared", "closed"))
        self.actions["A"].setEnabled(alive and phase in ("needs_reset", "ready", "recording"))
        self.actions["X"].setEnabled(alive and phase == "recording")
        self.actions["Y"].setEnabled(alive and phase == "recording")
        self.close_session.setEnabled(alive and phase in ("needs_reset", "reference", "ready"))
        counts = state.get("counts", {})
        self.counts.setText(
            "  ·  ".join(
                f"{key.capitalize()}: {counts.get(key, 0)}"
                for key in ("total", "success", "failure", "interrupted")
            )
        )
        for hand, label in self.arm_labels.items():
            arm = state.get("arms", {}).get(hand)
            if not arm:
                label.setText(
                    "Disconnected — see status above"
                    if phase == "fault"
                    else "Waiting for robot state"
                )
            else:
                label.setText(
                    f"{arm['status']}\nScale {arm['scale']:g} · gripper {arm['width'] * 1000:.0f} mm\nTCP: "
                    + ", ".join(f"{v:.3f}" for v in arm["target"])
                )
        self.location.setText(state.get("location") or str(self.runtime.output))
        for role, label in self.images.items():
            image = self.runtime.cameras.latest.get(role)
            receipt = self.runtime.cameras.received.get(role, 0)
            import time

            if image is None or time.monotonic() - receipt > 1.5:
                label.setText(
                    f"{role}: offline · " + self.runtime.cameras.errors.get(role, "waiting")
                )
                continue
            rgb = image[..., ::-1].copy()
            h, w = rgb.shape[:2]
            qimage = QtGui.QImage(rgb.data, w, h, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
            label.setPixmap(
                QtGui.QPixmap.fromImage(qimage).scaled(
                    label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
                )
            )

    def closeEvent(self, event):
        if not self.closing:
            reader = getattr(self.runtime, "reader", None)
            if reader is not None:
                reader.journal.event("window_close_requested", phase=self.runtime.status["phase"])
        self.closing = True
        self.runtime.stop_event.set()
        if self.runtime.thread is not None and self.runtime.thread.is_alive():
            self.status.setText("Stopping both arms and flushing recordings…")
            event.ignore()
            return
        self.timer.stop()
        event.accept()
