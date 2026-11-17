// Run in a JavaScript engine with the client source passed to this function.
// All browser, XR and network objects are fakes; no real session is started.
async function testQuestClientLifecycle(clientSource) {
  const passed = [];
  function check(condition, message) {
    if (!condition) throw new Error(message);
  }
  function fixture(probe = () => Promise.resolve(true), secure = true) {
    const elements = Object.fromEntries(['start','stop','status','diagnostics'].map(id =>
      [id, {disabled:true, textContent:''}]));
    const pageEvents = {};
    const xr = {isSessionSupported:probe};
    let requested = 0;
    class Session {
      constructor() { this.events={}; this.inputSources=[]; this.visibilityState='visible'; }
      addEventListener(name, callback) { this.events[name]=callback; }
      updateRenderState(state) { this.renderState=state; }
      async requestReferenceSpace() { return {addEventListener(){}}; }
      requestAnimationFrame() {}
      async end() { this.events.end?.(); }
    }
    xr.requestSession = async () => {requested++; return new Session();};
    const document = {getElementById:id=>elements[id], body:{}, visibilityState:'visible',
      createElement:()=>({getContext:()=>({})}), addEventListener(){}};
    const window = {isSecureContext:secure, addEventListener:(name, callback)=> {
      (pageEvents[name] ??= []).push(callback);
    }};
    const load = new Function('window','navigator','document','location','crypto','performance',
      'fetch','setInterval','setTimeout','clearTimeout','XRWebGLLayer',
      clientSource + '\nreturn {getSession:()=>session};');
    const api = load(window, {xr,userAgent:'test'}, document, {href:'http://localhost/',origin:'http://localhost'},
      {randomUUID:()=>String(requested)}, {now:()=>0,timeOrigin:0},
      async()=>({ok:true,text:async()=>''}), ()=>0, ()=>0, ()=>{}, class Layer {});
    return {elements,xr,api,requested:()=>requested,
      event:name=>pageEvents[name]?.forEach(callback=>callback())};
  }

  for (const [name, probe] of [
    ['pending support probe', ()=>new Promise(()=>{})],
    ['false support probe', ()=>Promise.resolve(false)],
    ['rejected support probe', ()=>Promise.reject(new Error('temporarily unavailable'))],
  ]) {
    const f=fixture(probe);
    await Promise.resolve(); await Promise.resolve();
    check(!f.elements.start.disabled, name+' permanently disabled Start');
    await f.elements.start.onclick();
    check(!!f.api.getSession(), name+' prevented an actual session request');
    passed.push(name);
  }

  const f=fixture();
  await f.elements.start.onclick();
  check(f.elements.start.disabled && !f.elements.stop.disabled, 'Active session buttons incorrect');
  await Promise.resolve();
  check(f.elements.start.disabled, 'Late support probe enabled Start during active VR');
  await f.elements.stop.onclick();
  check(!f.elements.start.disabled && f.elements.stop.disabled, 'Stop did not re-enable Start');
  await f.elements.start.onclick();
  check(f.requested()===2 && !!f.api.getSession(), 'Second session could not start');
  passed.push('start / stop / start');
  const active=f.api.getSession();
  await active.end();
  check(!f.elements.start.disabled, 'Browser-initiated session end left Start disabled');
  passed.push('external session end');

  let finishProbe;
  const delayed=fixture(()=>new Promise(done=>{finishProbe=done;}));
  await delayed.elements.start.onclick();
  finishProbe(true);
  await Promise.resolve();
  check(delayed.elements.start.disabled, 'Delayed probe changed active-session button state');
  passed.push('late support probe preserves active state');

  const setupFailure=fixture();
  const request=setupFailure.xr.requestSession;
  setupFailure.xr.requestSession=async()=>{
    const opened=await request();
    opened.requestReferenceSpace=async()=>{throw new Error('Reference space unavailable');};
    return opened;
  };
  await setupFailure.elements.start.onclick();
  check(!setupFailure.elements.start.disabled && setupFailure.api.getSession()===null,
    'Reference-space failure left Start disabled');
  passed.push('reference-space setup failure retry');

  const failed=fixture();
  failed.xr.requestSession=async()=>{throw new Error('Permission denied');};
  await failed.elements.start.onclick();
  check(!failed.elements.start.disabled && failed.api.getSession()===null, 'Failed request locked Start');
  check(failed.elements.status.textContent.includes('Permission denied'), 'Startup error hidden');
  passed.push('request failure retry');

  const pending=fixture();
  let resolve;
  pending.xr.requestSession=()=>new Promise(done=>{resolve=done;});
  const first=pending.elements.start.onclick();
  check(pending.elements.start.disabled, 'Pending request did not disable duplicate start');
  await pending.elements.start.onclick();
  resolve({addEventListener(){},updateRenderState(){},
    requestReferenceSpace:async()=>({addEventListener(){}}),requestAnimationFrame(){}});
  await first;
  passed.push('duplicate start blocked');

  const navigation=fixture();
  await navigation.elements.start.onclick();
  navigation.event('pagehide');
  check(navigation.api.getSession()===null, 'Navigation did not end immersive session');
  passed.push('navigation teardown');

  const insecure=fixture(undefined,false);
  check(insecure.elements.start.disabled, 'Insecure context enabled VR');
  await insecure.elements.start.onclick();
  check(insecure.requested()===0, 'Insecure context requested VR');
  passed.push('insecure context remains disabled');
  return passed;
}

// Optional local runner: node tests/quest_client_lifecycle.js
if (typeof require === 'function' && typeof module !== 'undefined' && require.main === module) {
  const fs = require('node:fs');
  const path = require('node:path');
  const source = fs.readFileSync(path.join(__dirname, '../dual_franka/quest/static/app.js'), 'utf8');
  testQuestClientLifecycle(source).then(passed => console.log(passed.join('\n'))).catch(error => {
    console.error(error);
    process.exitCode = 1;
  });
}
