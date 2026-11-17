# Dual Franka teleoperation and demonstration collection

Two Quest controllers operate two Panda arms. The right thumbstick sets both
references using each controller's own tracked pose; the left thumbstick is unused.
Both controllers must be tracked when setting the references. Both arms share one recording
session and episode clock, while keeping independent clutch, reference, gripper,
IK and joint trajectories. The project is self-contained; it does not import or
modify the single-arm code in `../neura_demonstrator`.

Default assignment:

| Controller | Robot IP | ROS namespace |
| --- | --- | --- |
| Left | `172.16.0.2` | `/left` |
| Right | `172.16.0.3` | `/right` |

Edit `config/robots.json` to change the IPs, assignment, per-arm yaw, control
settings or camera serials. Copying that file gives you a second setup selectable
with `--config`. The existing three camera serials are configured as
`wrist_left`, `side`, `opposite`; confirm which arm actually carries that wrist
camera. Add `wrist_right` with its own serial if you have a fourth camera.
Camera roles apply to demonstration collection. Standalone teleoperation does not access cameras.

## Repository checkout

```bash
git clone https://github.com/dtc103/dual_franka_teleop.git ~/dev/dual_franka_teleoperation
```

The checkout name above preserves the paths used in the commands below. The
Python entry points also work from another checkout location using its actual
path. This repository contains the dual-robot application, controller, Quest
web client, collection tools, tests, Linux USB rule and driver compatibility
patch. ROS/libfranka and Python environments are dependencies, not vendored
binaries. Generated datasets, logs and last-applied collision-limit caches are
ignored by Git.

## Environment

Use the existing **Jazzy/Pixi environment** in `~/install/pixi_panda_ros2`.
ROS, the Panda driver, Pink, Pinocchio, PySide6 and RealSense dependencies were
found there during setup. On the existing PC, no replacement driver library or new ROS build is
required. This project provides its own namespaced bringup using those installed
packages. `requirements.txt` lists Python dependencies if rebuilding an environment;
ROS and `franka_msgs` still come from the robot workspace. The existing
workspace is based on [lvjonok/pixi_panda_ros2](https://github.com/lvjonok/pixi_panda_ros2);
a fresh machine must install/build that compatible driver environment. `requirements.txt` alone does not install
ROS or the Franka hardware driver.

Use the RT kernel and the realtime user permissions already configured for the
single-arm setup. Close the old single-arm driver/teleoperator first. There must
be only one FCI control connection per robot.

Read-only diagnostics:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/check_setup.py
```

## 1. Teach reset paths once

No reset positions are invented or copied from the old setup. Each arm needs
its own taught clearance/start sequence. If you only want standalone
teleoperation, skip this step and use `teleoperate.py` below.

Start both drivers **with state broadcasters only**, so no arm motion controller
is loaded:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/start_robots.py \
  --execute --state-only
```

In another terminal:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/teach_reset.py
```

Use native hand-guiding. Position both arms, release the guiding controls and
enter `S` to save a pair of joint poses. Repeat for clearance poses; the **last
pair is the starting pose**. `D` removes the last pair. `Q` writes
`reset_sequences/left.json` and `right.json` and updates the configuration paths.
Existing files are replaced when you finish a new teaching session.

At reset, both grippers open first. The left arm traverses its entire sequence,
then the right arm traverses its sequence. Teach paths for that ordering; the
planner does not check collisions between the arms or the environment.
Stop the state-only driver with Ctrl+C when teaching is finished.

## 2. Start the normal two-arm drivers

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/start_robots.py --execute
```

The launch starts `/left/controller_manager` and `/right/controller_manager` at
1000 Hz, their joint trajectory/state controllers, and two gripper servers.
Arm gains and joint-trajectory generation match the single-arm joint backend.
No CRISP impedance controller is loaded. If a controller manager exits, the
launch shuts down both driver processes.

The driver launch activates controllers but sends no reset/movement goal and
never changes collision thresholds. It does not perform automatic error recovery.

## 3. Start collection

In another terminal:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/run_collection.py \
  --port 8444 --execute
```

For a read-only preview, omit `--execute`. Preview recordings are explicitly
marked and should not be used as executed demonstrations.

The Quest HTTP relay starts immediately. The application automatically runs ADB
port forwarding and requests the proximity override. In the **Quest browser**,
open **http://localhost:8444/** and press **Start controller tracking**. Accept USB
debugging in the headset if `adb devices` shows `unauthorized`.

Enter a session name and task description, select both reset JSON files, and
click **Prepare session**. Multiple prompts can be separated by semicolons.
Camera reset/acquisition runs in the background at startup and preparation.
Missing cameras are shown as offline and do not block either arm.

1. Press right **A** to open grippers and follow both reset paths.
2. Release both grips/triggers, then click the **right controller's thumbstick** to
   establish both references together. World-aligned mode uses each arm's configured yaw.
3. Press right **A** again to begin the paired episode.
4. Hold a controller's **grip** to move that arm; release to reposition your hand.
5. Click its **trigger** to toggle its gripper closed/open.
6. Hold right **B** to reduce translation **and rotation** on **both** arms to
   `precision_scale` (default 0.5). Release B to return to normal scale.
7. Right **A** ends successfully; left **X** ends unsuccessfully; left **Y** marks
   interruption. Both arms stop accepting movement immediately. After bounded
   braking/rest, the episode finalizes and both reset paths execute again.

The GUI has matching episode buttons and counters for each outcome. A missing
controller pose behaves like clutch release for that arm, keeping the episode.
An actual robot fault, stale robot state or disk failure interrupts the paired
episode and stops both arms; a fault does not launch a reset or recovery motion.

Close the GUI and wait for recordings to flush; if control is running it asks
you to close again after the worker stops. Then stop the driver terminal.
Restore normal Quest proximity behavior if desired:

```bash
adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable
```

### Standalone teleoperation

No session name, camera captures or reset sequences are required:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/teleoperate.py \
  --port 8444 --execute
```

Click the right thumbstick to set both references, release grips/triggers, then grip to move. Add `--headless`
to print status without opening Qt. The GUI's recording buttons remain disabled
in standalone mode. The Quest server remains available. Camera SDK imports, resets, acquisition and previews are completely disabled in standalone teleoperation.

### Control options

Settings in `config/robots.json` use the same semantics as the single-arm joint
backend. CLI overrides include `--scale`, `--precision-scale`, `--frame-alignment`,
`--yaw-left`, `--yaw-right`, `--workspace-radius`, `--max-speed`,
`--max-angular-speed`, `--joint-speed`, `--joint-acceleration`, `--joint-jerk`,
`--smoothing-seconds`, `--trajectory-horizon`, `--stale-seconds`, `--grasp-force`,
`--gripper-speed`, `--reset-speed`, and `--disable-reset-path-tolerance`.
Joint speeds remain bounded by the URDF. Default mapping at yaw zero is Quest
forward → robot +X, right → −Y, up → +Z. Each robot's local base is independent;
yaw offsets must reflect the physical setup. This is not a shared-world
calibration or coordinated collision-avoidance planner.

Use `--reset-left PATH --reset-right PATH` to override the default sequences.
Use `--no-usb-setup` to skip ADB forwarding/proximity configuration.

## Recorded data and synchronization

A named session is created under `datasets/demonstrations/` (or `--output`).
Each episode contains both arms:

```text
session/
  session.json                    # settings, arm identities, clock pair, gains, resets
  left_robot.urdf
  right_robot.urdf
  episode_000001/
    episode.json                  # outcome, prompts, timing, camera intrinsics
    quest.jsonl                   # both raw controllers, original source clock mapping
    control.jsonl                 # paired target/control snapshots
    left_control.jsonl
    right_control.jsonl
    left_robot_state.jsonl        # full proprioception, errors, forces/torques
    right_robot_state.jsonl
    left_sent_command.jsonl       # actual published joint trajectory messages
    right_sent_command.jsonl
    left_gripper_state.jsonl
    right_gripper_state.jsonl
    left_gripper_command.jsonl    # appears when commanded during the episode
    right_gripper_command.jsonl
    clock_samples.jsonl           # ROS/host monotonic/host wall-clock pairs
    camera_info.jsonl              # late camera intrinsics when reconnecting
    camera_<role>.jsonl
    cameras/<role>/*_rgb.npy
    cameras/<role>/*_depth.npy
```

Both arm targets are computed from the **same Quest snapshot and control-loop
clock**, nominally 50 Hz, then published separately. Receipt times and ROS
headers remain available for each stream. This does not claim simultaneous
hardware execution or measure the command-to-actuation delay. RGB/depth remain
lossless uint8/uint16 arrays. Camera exposure timestamps and clock domains are
retained, including a separate `depth_timing` field. USB timing estimates have
unknown minimum transport latency; cameras are not hardware-trigger synchronized.

Clutch pauses remain inside the episode and are labeled per arm. A later exporter
can distinguish left/right pauses and gripper actions. The single-arm LeRobot
exporter in the other repository does **not** support this dual-arm schema yet;
these are raw recordings, not a ready-made bimanual LeRobot dataset.

## Offline verification

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy bash -c \
  'cd ~/dev/dual_franka_teleoperation && python -m unittest discover -s tests -v'
```

To check namespace/configuration loading without connecting to either robot:

```bash
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/start_robots.py --fake
```

Fake hardware is a bringup check, not a physical dynamics simulation. It lacks
the real Franka state interfaces; running the collector against it additionally
needs a synthetic state source. Unit tests and development smoke tests use
synthetic states without sending motion commands to either physical Panda.

### Edit force/torque collision thresholds by robot IP

Keep both drivers running and stop teleoperation first:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/configure_collision.py --robot-ip 172.16.0.2
# Or the other arm:
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/configure_collision.py --robot-ip 172.16.0.3
```

The editor selects the matching ROS namespace from `config/robots.json`, checks
its fresh stationary/error-free state, and verifies the running driver's IP in
its published URDF before applying. Press Enter to keep a field or `q` to cancel;
type `APPLY` after reviewing the complete request. `--dry-run` never connects to
ROS or changes thresholds. Per-IP cached values are last-applied records, not
live readback. Collision thresholds are not a commanded contact-force cap.

## Quest connection recovery and diagnostics

USB forwarding is now monitored for the lifetime of the GUI, including after a
robot worker faults. Unplug/replug and missing `adb reverse` rules are repaired
automatically once Linux USB access and headset debugging authorization allow
ADB to connect. The right/left controller input and controller parameters are
unchanged. The GUI shows USB, browser/session and controller tracking separately.
Connection events are saved to `logs/quest_connection_*.jsonl`, independently of
any episode recording. See [CONNECTIONS.md](CONNECTIONS.md) for the investigation
and verified recovery tests.

One-time Linux USB permissions fix (needed on this PC for Quest USB mode 5013):

```bash
sudo install -m 644 ~/dev/dual_franka_teleoperation/setup/51-meta-quest.rules /etc/udev/rules.d/51-meta-quest.rules
sudo udevadm control --reload-rules
```

Then unplug/replug USB. The rule grants `plugdev` access only to Meta/Oculus USB
devices, vendor `2833`, across USB modes. The user must belong to `plugdev` (`jan`
already does). USB debugging still requires headset authorization.

To test tracking without ROS, cameras or robot control:

```bash
cd ~/install/pixi_panda_ros2
pixi run -e jazzy python ~/dev/dual_franka_teleoperation/quest_relay.py --port 8443
```

Open `http://localhost:8443/` in Quest Browser and start tracking. Add
`--open-quest-browser` to open the page from the PC once; it does not click Start
or reopen the browser after a USB interruption. `--quest-serial SERIAL` selects
one headset if other Android devices are connected. Both options also work with
`teleoperate.py` and `run_collection.py`.

Inspect live connection health from the PC:

```bash
curl --noproxy '*' http://127.0.0.1:8443/health
```

Stop `quest_relay.py` before starting teleoperation on the same port. It is a
standalone diagnostic server, not an additional server to run alongside it.
Ctrl+C, SIGTERM and a normal window close release the application server; a hard
process crash releases sockets through the OS. The browser can reconnect to a
new server while its VR session remains active. Robot reconnection remains
explicit through **Reconnect robots**; transport recovery never runs robot error
recovery. A suspended/ended VR session, sleeping or occluded controllers and
Linux permission failures are distinct from HTTP connectivity.

Startup order: the Quest server and USB monitor start first, the GUI opens, then
robot connections initialize in the background. Camera initialization runs only in demonstration collection. You can open the
Quest page/start tracking while the GUI reports that robot connections are
pending. No robot state or running driver is required for the page or window
to appear. Motion becomes available once both robot connections are ready.

## Open or close one gripper

Keep the dual drivers running and stop teleoperation before issuing standalone
gripper commands. This CLI automatically invokes the existing Jazzy/Pixi
workspace; it does not command arm motion.

```bash
# Left robot (172.16.0.2 with the default configuration):
python3 ~/dev/dual_franka_teleoperation/gripper.py --open --action /left/panda_gripper/move
# Right robot (172.16.0.3):
python3 ~/dev/dual_franka_teleoperation/gripper.py --open --action /right/panda_gripper/move
```

Replace `--open` with `--close` to request zero finger separation. This uses a
Move action, not force-controlled grasping. Namespace selection follows your
running driver; if you change the namespaces, update `--action` accordingly.
Use `--pixi-project PATH` if the ROS workspace lives elsewhere.
