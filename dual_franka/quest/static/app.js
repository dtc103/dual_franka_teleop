const status = document.getElementById('status');
const start = document.getElementById('start');
const stop = document.getElementById('stop');
const diagnostics = document.getElementById('diagnostics');
const CLIENT = '20261117-vr-restart';
diagnostics.textContent = `Client: ${CLIENT}\nURL: ${location.href}\nSecure context: ${window.isSecureContext}\nWebXR API: ${!!navigator.xr}\nBrowser: ${navigator.userAgent}`;
let session = null, space = null, gl = null, source = '', epoch = 0, sequence = 0;
let lastSend = 0, sending = false;
let starting = false, stopping = false;
const canStartXR = window.isSecureContext && !!navigator.xr;

function updateButtons() {
  start.disabled = !canStartXR || starting || !!session;
  stop.disabled = !session || stopping;
  start.textContent = starting ? 'Starting VR…' : session ? 'Tracking active' : 'Start controller tracking';
}

let lastFrameAt = 0, lastAckAt = 0, lastError = '', sendRequest = null;
const requests = new Set();

async function request(url, options = {}) {
  const controller = new AbortController();
  requests.add(controller);
  const timer = setTimeout(() => controller.abort(), 1000);
  try {
    const response = await fetch(url, {...options, cache:'no-store', signal:controller.signal});
    const body = await response.text();
    if (!response.ok) throw new Error(body || `HTTP ${response.status}`);
    return body;
  } finally {
    clearTimeout(timer);
    requests.delete(controller);
  }
}

function cancelRequests() {
  for (const controller of requests) controller.abort();
}

async function reportStatus() {
  const now = Date.now();
  const data = {client:CLIENT, source, session:session?.visibilityState ?? 'not-started',
    visibility:document.visibilityState, error:lastError,
    inputs:session ? Array.from(session.inputSources, input=>({hand:input.handedness,
      hand_tracking:!!input.hand, grip_space:!!input.gripSpace,
      mapping:input.gamepad?.mapping ?? null, profiles:Array.from(input.profiles)})) : [],
    last_frame_age_ms:lastFrameAt ? now-lastFrameAt : null,
    last_ack_age_ms:lastAckAt ? now-lastAckAt : null};
  try {
    await request('/client-status', {method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify(data)});
  } catch (_) { /* USB repair runs on the PC, independently of this page. */ }
  if (session && lastFrameAt && now-lastFrameAt > 2000)
    status.textContent='XR frame loop paused. Wake the headset and close system overlays. The PC relay is still running.';
}
setInterval(reportStatus, 2000);
document.addEventListener('visibilitychange', () => { cancelRequests(); reportStatus(); });
window.addEventListener('pageshow', () => { cancelRequests(); syncClock(); reportStatus(); });
window.addEventListener('pagehide', cancelRequests);
window.addEventListener('error', event => { lastError=event.message; reportStatus(); });


let clockSync = null;
async function syncClock() {
  const before = performance.now();
  try {
    const sample = JSON.parse(await request('/clock'));
    const after = performance.now();
    const candidate = {host_offset_ns:sample.host_monotonic_ns - (before+after)*5e5,
      round_trip_ms:after-before, at:after};
    if (!clockSync || after-clockSync.at > 10000 || candidate.round_trip_ms < clockSync.round_trip_ms)
      clockSync=candidate;
  } catch (_) { /* Missing synchronization is explicit in recorded timing. */ }
}
setInterval(syncClock, 2000);
syncClock();

function payload(frame, frameTime) {
  const controllers = {left:null, right:null};
  const controller_buttons = {left:null, right:null};
  if (session.visibilityState === 'visible') {
    for (const input of session.inputSources) {
      // Do not let a hand-tracking source replace a physical controller.
      if (input.hand || !input.gamepad || !input.gripSpace ||
          input.gamepad.mapping !== 'xr-standard' || !(input.handedness in controllers)) continue;
      // Buttons remain usable when the physical controller has no tracked pose.
      controller_buttons[input.handedness] = {
        input_type:'controller',
        select:input.gamepad.buttons[4]?.pressed ?? false,
        secondary:input.gamepad.buttons[5]?.pressed ?? false
      };
      const pose = frame.getPose(input.gripSpace, space);
      if (!pose || pose.emulatedPosition) continue;
      const p = pose.transform.position, q = pose.transform.orientation;
      controllers[input.handedness] = {
        axes:Array.from(input.gamepad.axes),
        buttons:Array.from(input.gamepad.buttons, b=>({value:b.value, pressed:b.pressed, touched:b.touched})),
        input_type:'controller', position:[p.x,p.y,p.z], orientation:[q.x,q.y,q.z,q.w],
        trigger:input.gamepad.buttons[0]?.value ?? 0,
        squeeze:input.gamepad.buttons[1]?.value ?? 0,
        thumbstick:input.gamepad.buttons[3]?.pressed ?? false,
        select:input.gamepad.buttons[4]?.pressed ?? false,
        secondary:input.gamepad.buttons[5]?.pressed ?? false,
        emulatedPosition:!!pose.emulatedPosition
      };
    }
  }
  return {reference_space:'local-floor', source, epoch, sequence:sequence++, controllers, controller_buttons,
    timing:{frame_time_ms:frameTime, sample_time_ms:performance.now(), time_origin_ms:performance.timeOrigin,
      host_offset_ns:clockSync?.host_offset_ns ?? null,
      round_trip_ms:clockSync?.round_trip_ms ?? null,
      sync_age_ms:clockSync ? performance.now()-clockSync.at : null}};
}

async function send(data) {
  if (sending) return;
  sending = true;
  const attempt = {};
  sendRequest = attempt;
  try {
    await request('/pose', {method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify(data)});
    lastAckAt=Date.now(); lastError='';
    const hands = Object.entries(data.controllers).filter(([,pose])=>pose).map(([hand])=>hand);
    if (session) status.textContent = `Streaming local-floor controller poses\nTracked: ${hands.join(', ') || 'none'}\nReference epoch: ${epoch}\nRight B: ${data.controllers.right?.secondary ?? "not tracked"}`;
  } catch (error) {
    lastError=error.message;
    status.textContent = `Relay disconnected: ${error.message}. Retrying automatically; no script restart needed.`;
  } finally {
    if (sendRequest === attempt) sending = false;
  }
}

function onFrame(time, frame) {
  if (!session) return;
  lastFrameAt=Date.now();
  session.requestAnimationFrame(onFrame);
  gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
  gl.clearColor(.05,.08,.15,1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  if (time-lastSend >= 1000/30 && !sending) {
    lastSend=time;
    send(payload(frame, time));
  }
}

function sessionEnded(ended) {
  if (session !== ended) return;
  cancelRequests();
  const endedSource = source;
  session = null; space = null; stopping = false;
  updateButtons();
  status.textContent = 'VR session ended. Click Start controller tracking to resume.';
  reportStatus();
  send({reference_space:'local-floor', source:endedSource, epoch, sequence:sequence++,
    controllers:{left:null,right:null}});
}

start.onclick = async () => {
  if (!canStartXR || starting || session) return;
  starting = true;
  updateButtons();
  status.textContent = 'Starting VR. Accept or cancel any headset permission prompt.';
  try {
    const canvas = document.createElement('canvas');
    gl = canvas.getContext('webgl', {xrCompatible:true});
    if (!gl) throw new Error('WebGL unavailable');
    const opened = await navigator.xr.requestSession('immersive-vr', {
      requiredFeatures:['local-floor'], optionalFeatures:['dom-overlay'], domOverlay:{root:document.body}
    });
    session = opened;
    source = crypto.randomUUID(); epoch=0; sequence=0; lastSend=0;
    lastFrameAt=0; lastAckAt=0;
    // Register teardown before reference-space setup, which can itself fail.
    opened.addEventListener('end', () => sessionEnded(opened));
    opened.addEventListener('visibilitychange', () => { cancelRequests(); reportStatus(); });
    opened.updateRenderState({baseLayer:new XRWebGLLayer(opened,gl)});
    const reference = await opened.requestReferenceSpace('local-floor');
    if (session !== opened) return; // Ended while reference-space setup was pending.
    space = reference;
    space.addEventListener('reset', () => {epoch++;});
    lastError=''; reportStatus();
    opened.requestAnimationFrame(onFrame);
  } catch (error) {
    const failed = session;
    session = null; space = null;
    if (failed) failed.end().catch(() => {});
    lastError = `${error.name}: ${error.message}`;
    status.textContent = `Cannot start: ${lastError}. You can try Start again.`;
    reportStatus();
  } finally {
    starting = false;
    updateButtons();
  }
};

stop.onclick = async () => {
  if (!session || stopping) return;
  stopping = true;
  updateButtons();
  try {
    await session.end();
  } catch (error) {
    lastError = `${error.name}: ${error.message}`;
    status.textContent = `Cannot stop VR: ${lastError}`;
    reportStatus();
  } finally {
    stopping = false;
    updateButtons();
  }
};

window.addEventListener('pageshow', updateButtons);
window.addEventListener('pagehide', () => {
  // Navigation/reload should release this page's immersive session.
  if (session) session.end().catch(() => {});
});
updateButtons();
if (!canStartXR) {
  status.textContent = 'WebXR needs Quest Browser and a secure context. Open this server through its localhost URL.';
} else {
  status.textContent = 'Ready. Click Start controller tracking.';
  // Advisory only: a failed or pending availability probe must not lock out
  // a user-initiated requestSession(), which reports the actual startup error.
  navigator.xr.isSessionSupported('immersive-vr').then(supported => {
    diagnostics.textContent += `\nImmersive VR support probe: ${supported}`;
  }).catch(error => {
    diagnostics.textContent += `\nSupport probe error: ${error.name}: ${error.message}`;
  });
}
