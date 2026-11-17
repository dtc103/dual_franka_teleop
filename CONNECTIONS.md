# Quest connection investigation — 17 November 2026

Scope: `dual_franka_teleoperation`, Quest 3 `2G0YC5ZH2Z010C`, USB loopback HTTP
port 8443. All interactive tests used a tracking-only relay. No robot motion,
controller switching, force-limit changes or robot recovery was performed.

## Reproduced failure: Linux USB permissions depend on Quest USB mode

During the physical unplug/replug test:

- 17:36:50 CET: USB disconnected.
- 17:37:00: the Quest enumerated as vendor/product `2833:5013`, device
  `/dev/bus/usb/002/006`.
- Device permissions were `root:root`, mode `0664`, with no user ACL. `jan` had
  no write access. `adb devices -l` reported **no permissions**. The installed
  `/usr/lib/udev/rules.d/51-android.rules` had no rule for vendor `2833`.
- The system journal and `udevadm info` confirmed no `uaccess` tag for this mode.
  The current Wayland desktop session was active and `jan` already belonged to
  `plugdev`; group membership was not the missing part.
- After the operator accepted the headset connection prompt, the device
  re-enumerated at 17:38:15 as `2833:5012`, `/dev/bus/usb/002/007`. This mode had
  `uaccess` and a user ACL. ADB became usable again.
- The new monitor restored port forwarding and browser packets resumed, with
  the same browser source ID, without restarting the PC server.

This explains why accepting prompts/reconnecting can appear to randomly fix the
connection. An ADB authorization prompt and Linux's `no permissions` error are
different layers. Restarting Python cannot grant Linux access to the device.
The exact event that caused the investor-demo failure was not recorded; the
permission failure was reproduced today and is an established contributing
problem, not proof of every earlier incident.

Prepared permanent fix: `setup/51-meta-quest.rules` covers vendor `2833` across
USB modes and grants `plugdev` read/write access, plus desktop `uaccess`.
Installation requires sudo; the running agent cannot supply the operator's
password. README contains the two installation commands. Replug after installing.
ADB must still be authorized on the headset.

## Confirmed application gaps and changes

1. **One-shot USB setup.** Previously `quest/setup.py` ran `adb reverse` and the
   proximity broadcast once at startup, then returned. Lost mappings/transport
   changes were never repaired. `USBConnection` now checks every two seconds,
   selects an authorized transport ID explicitly, and restores missing mappings.
   This also avoids accidentally addressing a stale permission-denied transport
   with the same serial. It retries after authorization becomes available.
2. **Incomplete Ctrl+C shutdown in the GUI.** The previous signal handlers only
   stopped the robot worker. The Qt event loop and HTTP listener could remain
   alive, making the next startup contend with an old process. SIGINT/SIGTERM
   now request window closure and run the worker/USB/server cleanup. Window
   closure remains asynchronous while the robot worker finishes.
3. **Insufficient diagnostics.** A quiet HTTP server could not tell us whether
   the browser loaded, VR started, frames stopped, controllers disappeared or
   input was rejected. `/health`, periodic browser status and a connection
   journal now expose these states separately. GUI status displays them without
   depending on robot readiness.
4. **Server robustness.** Connection socket timeouts and a larger accept backlog
   bound idle browser preconnections/partial requests. HEAD requests work, and
   browser-cancelled requests do not produce misleading BrokenPipe tracebacks.
   Idle preconnections were tested; they have not been established as the cause
   of an earlier headset freeze.
5. **Browser lifecycle.** Requests already had one-second timeouts in the old
   client; an absent timeout was not the bug. The updated client additionally
   cancels outstanding requests on visibility/resume/end changes and reports
   paused XR frame callbacks and session end. The client retries transport
   errors without restarting the VR session or buffering old controller motion.
   The updated client ID is `20261117-connection-recovery`.

The relay already lived outside the robot worker in GUI mode; that separation
is preserved. A robot fault stops control, while the GUI and Quest relay remain
available. Use **Reconnect robots** after restoring robot readiness. In headless
mode, the process still exits when its robot worker exits.

## What the live inspection established

- Initially no PC listener existed on either 8443 or 8444. Nevertheless the
  browser had a visible, active immersive session and was trying to send poses;
  its message was `Relay error: Failed to fetch`.
- Starting the PC relay restored packet flow in the existing browser session.
  This specific case did not require restarting the browser or VR session.
- The new browser version loaded and reported successful POST request durations
  around 3.5–3.8 ms in inspected samples. This is HTTP request duration, not an
  end-to-end motion latency measurement.
- Both controller poses arrived during the operator's movement tests. Other
  intervals contained one or neither controller. A live HTTP stream with no
  tracked controller poses is not a failed server connection.
- The browser reported `visible-blurred` while system UI was present. The
  controller input path deliberately yields no controller poses when XR is not
  fully visible/focused. That holds motion; it must not reuse old tracked poses.
- Kernel logs showed several physical USB disconnect/re-enumeration events
  before the controlled test too. Their cause (cable, USB mode changes or user
  actions) cannot be inferred from the kernel entries alone.

## Verification

- Removed the live ADB reverse rule: it was restored and browser packets resumed
  in approximately two seconds, without page reload or script restart.
- Sent SIGINT to the standalone tracking-only relay: the listener stopped, then
  a new process rebound port 8443 and received packets from the same active
  browser source, without a headset action.
- Physically unplugged/replugged: reproduced the Linux permission failure; after
  the usable USB mode/authorization returned, automatic forwarding and packet
  recovery worked. Repeating this after installing the permanent permission
  rule is the remaining hardware acceptance step.
- Automated tests cover same-port rebinding, idle browser connections, stale
  frames, conflicting browser ownership, heartbeat/pose separation, HEAD/clock
  routes, authorized-transport selection, forwarding repair and replug.
- Process tests cover SIGINT and SIGTERM on the real Qt application with a fake
  robot runtime, and socket rebinding after a forced process kill. No robot
  driver is involved.

## Reading `/health`

`streaming` means a valid frame arrived within the configured stale interval.
`tracked` lists the physical controllers with a valid tracked pose in that
frame. `accepted_frames` is a cumulative counter. `frame_age_ms` shows gaps.
`browser.session` distinguishes not-started/visible/visible-blurred/hidden;
`browser_age_seconds` tells whether browser status itself is recent. The
`active_source` identifies the stream owner. Heartbeats do not extend pose
freshness or authorize robot motion.

Connection journals live in `logs/quest_connection_*.jsonl`; they record changes
and errors, not every pose, and are separate from recorded demonstrations.

## Later alternatives (no migration implemented)

A native OpenXR Android app can be launched from ADB and stream controller poses
without opening a web page or clicking the WebXR Start button. It still needs a
running/focused XR session, valid tracking and initial Android authorization.
This does not remove USB permissions or headset sleep/tracking constraints.

Existing source prototype:
`../neura_demonstrator/tests/quest_tracking_alternative/android/`. Its README
states the APK has not been built or physically verified; it is not a tested
replacement yet. It uses STAGE/LOCAL reference space rather than headset-relative
poses. The earlier build attempt was blocked by SDK downloads.

References checked:

- Meta OpenXR input: https://developers.meta.com/vr/documentation/native/android/mobile-openxr-input/
- WebXR session user-activation requirements: https://developer.mozilla.org/en-US/docs/Web/API/WebXR_Device_API/Permissions_and_security
- Meta's proximity override documentation: https://developers.meta.com/vr/documentation/unity/meta-xr-operator/quest/
- Android Linux USB setup: https://developer.android.com/studio/run/device
- PickNik native Unity/ROS 2 input app: https://github.com/PickNikRobotics/meta_quest_teleoperation
  Publishes poses/buttons and remembers the host address; our input adapter and
  coordinate-frame behaviour would need validation rather than copying its
  robot control stack.
- Quest2ROS2: https://github.com/Taokt/Quest2ROS2
  Documents Quest 2/3, ROS 2 Humble, a companion app and ROS–TCP setup. It is not
  a drop-in Jazzy input replacement and still has headset-side setup steps.

Per the operator's latest instruction, current work remains focused on testing
and stabilizing the existing browser tracking path.

## Follow-up: off-head tracking and reference buttons

The operator installed `setup/51-meta-quest.rules`. A subsequent replug in USB
mode `5013` produced `root:plugdev` mode `0660` with `uaccess`, and an authorized
ADB transport. This verifies the permission fix in the previously failing mode.
A stale ADB `no permissions` entry with transport ID 3 still appeared alongside
the working transport; the application selects the working transport ID.

The failed 17:46–17:47 run was also inspected on the headset. The power manager
confirmed receipt of `prox_close`; Android logged a sleep at 17:47:02 with reason
`power_button`. The operator could not recall whether the physical power button
was pressed. This OS classification does not establish who/what generated the
sleep request. It does establish a system sleep distinct from losing HTTP.
The proximity override was enabled; a request acknowledgement is not proof
that all other ways of sleeping the device are disabled.

After reboot, the next tracking-only run kept XR `visible` and the headset
awake while off-head. Both physical controllers appeared in WebXR. Live polling
confirmed right thumbstick press/release and right grip press/release reaching
the PC. At that time only the right controller had a non-emulated position;
direct WebXR inspection reported `emulatedPosition=true` for the left. These
estimated positions are deliberately excluded from robot motion input.
The shared reference requires both controller poses, so a missing left pose
can block setting either reference even when the right thumbstick is received.
The teleoperation status now explains which controller blocks the reference,
instead of only asking the operator to click the thumbstick again.

Current automated validation: 35 tests pass, including GUI SIGINT/SIGTERM,
forced-process-kill rebinding, independent arm/reference behaviour and connection
recovery. Targeted lint checks pass. Live tracking coverage is recorded above;
this is not a guarantee of optical tracking under occlusion or system sleep.

The final positioning test recovered **both** non-emulated controller poses.
With XR still `visible`, the live observer recorded both grip presses/releases
and a right thumbstick press/release while both controllers were tracked.
Evidence: `logs/quest_buttons_verified_20261117.jsonl`. The source/browser session
was unchanged, and no server restart was required for this recovery. This
verifies the off-head input path with both controllers in the headset cameras'
view. It does not test robot motion or guarantee tracking outside that view.

## VR Start button after stopping/reloading

The old client made the initial `isSessionSupported()` probe a permanent gate:
a false/rejected/unresolved result left Start disabled with no retry path.
It also let that asynchronous probe change buttons independently of an active
session. The observed disabled-button episode was not captured live; these
failure paths were found in code and reproduced with mocked WebXR objects.

Client `20261117-vr-restart` makes the probe advisory. Secure context and the
presence of WebXR are still required. Start is disabled during startup or an
active session; session end and failed startup restore it. Setup registers the
end handler before asynchronous reference-space initialization. Navigation
requests teardown of the old session. A script-load failure now gives an
explicit server/reload message instead of silently showing a grey button.

Eleven lifecycle cases passed in the connected Quest browser's JavaScript engine
using fake XR/DOM/network objects; no real VR session or robot commands were
started by these tests. The test harness is `tests/quest_client_lifecycle.js`
and can also run under Node. Real stop/reload/start remains an operator test.
Keep the PC server running while reloading, so the new JavaScript can load.

## Startup no longer waits for robot inputs

The entry point now binds the Quest HTTP server and starts USB forwarding before
constructing QApplication. Qt library selection/re-exec still happens first,
so it cannot discard an already-running server. Runtime construction is inert:
robot/IK imports, ROS initialization, camera reset/acquisition and robot service
waits run in workers. The GUI is shown and its event loop begins before robot
connection work is scheduled. Reconnect also creates camera state without
starting camera hardware work in the GUI callback.

Startup/shutdown cleanup covers a window closed before its worker starts and
ROS import/initialization failures. The relay remains independent of robot
readiness in GUI mode. Terminal messages explicitly announce when the Quest
server is listening and when the GUI is opened.

Verification: `tests/test_startup.py` runs the real GUI and ROS runtime in an
isolated ROS domain with no robot publishers, no camera devices and no execute
flag. It verifies the visible window and HTTP response while robot connection
is pending. The existing signal tests also check that HTTP is available during
window construction and that robot startup begins after the window is visible.
The 35 existing tests and the new startup test pass.

## Reproduced 45-second freeze and standalone camera removal

A later live report exposed a gap in the earlier startup test: its camera
configuration was empty. The real configuration listed three cameras, while
`lsusb` and RealSense enumeration showed none connected. Nevertheless the old
code called `pipeline.start(config)` for each configured serial.

An isolated SDK reproduction with one absent configured serial and no robot
connection produced a **15.012-second main-thread scheduling gap**, despite
pipeline startup running in another Python thread. Native faulthandler timed
thread dumps repeatedly showed that worker inside `pipeline.start` while the
main thread could not resume its 100 ms heartbeat. Three such attempts explain
the approximately 45-second application-wide stalls. Threading/startup order
alone did not isolate this native SDK wait.

Per the operator's clarification, `teleoperate.py` now has **no camera path**:
no CameraManager construction (including on robot reconnection), no camera SDK
import, no hardware resets, no streams and no camera preview widgets. Camera
handling remains in `run_collection.py`. Its discovery now skips absent serials
before SDK pipeline startup, so missing collection cameras do not incur this
known blocking wait either.

The GUI/server startup integration test now has a nonempty camera configuration
and checks that standalone teleoperation creates neither camera handling nor
preview widgets and does not import `pyrealsense2`. Camera discovery unit tests
cover all missing, a mixture of available/missing, and an empty configuration.
The GUI also journals window-close requests, termination signals and event-loop
exit to distinguish a later shutdown from the freeze. The prior terminal output
contained no crash traceback, so its exact exit trigger was not established.

Final validation after standalone camera removal: 39 Python tests pass and
changed-file lint checks pass. Ten isolated Qt window lifecycle runs also passed
after correcting the test fixture to dispose of its windows before interpreter
shutdown. The startup test confirms no RealSense import/camera handling in the
standalone teleoperation process. No live robot execution was performed.
