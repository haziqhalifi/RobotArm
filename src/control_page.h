#pragma once

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Robot Arm Control</title>
  <style>
    :root{color-scheme:dark;--bg:#07111f;--card:#111f33;--line:#29405d;--accent:#38bdf8;--good:#4ade80;--danger:#fb7185}
    *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
    body{margin:0;background:linear-gradient(160deg,#07111f,#0d1b2f);color:#edf6ff;font-family:Arial,sans-serif;overscroll-behavior:none}
    main{width:min(900px,94%);margin:auto;padding:18px 0 34px}
    h1{margin:0;font-size:clamp(1.7rem,6vw,2.4rem)} h2{margin:0 0 14px;font-size:1.1rem}
    .sub{color:#9fb2c9;margin:5px 0 14px;line-height:1.4}
    .card{background:rgba(17,31,51,.96);border:1px solid var(--line);border-radius:18px;padding:16px;margin:12px 0}
    .status{display:flex;justify-content:space-between;align-items:center;gap:12px;position:sticky;top:8px;z-index:5}
    .dot{width:11px;height:11px;border-radius:50%;display:inline-block;background:var(--good);margin-right:7px}
    .grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
    .pad-card{text-align:center}.pad-title{font-weight:700;margin-bottom:10px}
    .joystick{width:min(210px,75vw);aspect-ratio:1;margin:auto;border-radius:50%;position:relative;touch-action:none;user-select:none;background:radial-gradient(circle,#1f3854 0 28%,#142840 29% 62%,#0d1b2c 63%);border:2px solid #365679;box-shadow:inset 0 0 25px #050b13}
    .joystick:before,.joystick:after{content:"";position:absolute;background:#38516d;opacity:.55}
    .joystick:before{width:2px;height:82%;left:50%;top:9%}.joystick:after{height:2px;width:82%;top:50%;left:9%}
    .stick{width:34%;aspect-ratio:1;border-radius:50%;position:absolute;left:33%;top:33%;z-index:2;background:linear-gradient(145deg,#55cfff,#0876ad);box-shadow:0 8px 18px #02070c;pointer-events:none}
    .axis{display:flex;justify-content:space-between;color:#9fb2c9;font-size:.8rem;margin:8px auto 0;max-width:230px}
    .joint-head{display:flex;justify-content:space-between;gap:10px;font-weight:700;margin-bottom:8px}
    output{color:var(--accent);font-variant-numeric:tabular-nums}
    input[type=range]{width:100%;height:40px;accent-color:var(--accent)}
    input[type=number],input[type=text]{width:100%;padding:12px;border-radius:10px;border:1px solid #3b4b65;background:#0b1728;color:white;font-size:1rem;margin:8px 0}
    .buttons{display:grid;grid-template-columns:repeat(3,1fr);gap:9px}
    button{min-height:52px;border:0;border-radius:13px;background:#263b56;color:white;font-weight:700;font-size:1rem;touch-action:manipulation}
    button:active{transform:scale(.97);filter:brightness(1.2)}.primary{background:#057eaf}.danger{background:#b91c3c}.good{background:#15803d}
    .gripper{grid-template-columns:1fr 1fr}.gripper button{min-height:66px;font-size:1.1rem}
    details summary{cursor:pointer;font-weight:700;padding:4px 0}.joint{padding:11px 0;border-bottom:1px solid #243954}.joint:last-child{border:0}
    .recording{background:#dc2626}.routine{display:grid;grid-template-columns:1fr auto;gap:8px;margin-top:8px}
    .record-actions{display:none;position:fixed;left:50%;bottom:max(14px,env(safe-area-inset-bottom));transform:translateX(-50%);z-index:20;width:min(560px,calc(100% - 24px));padding:10px;background:rgba(7,17,31,.96);border:1px solid var(--line);border-radius:17px;box-shadow:0 12px 35px rgba(0,0,0,.55);grid-template-columns:1fr 1fr;gap:9px}
    .record-actions.visible{display:grid}.record-actions button{min-height:60px;font-size:1.05rem}
    body.recording-active main{padding-bottom:110px}
    small{display:block;text-align:center;color:#8293aa;margin-top:18px}
    @media(max-width:650px){.grid{grid-template-columns:1fr}.buttons{grid-template-columns:1fr 1fr}.status{font-size:.9rem}}
  </style>
</head>
<body><main>
  <h1>Robot Arm</h1>
  <p class="sub">Drag and hold a joystick to move. Release it to stop.</p>
  <div class="card status"><span><i class="dot" id="dot"></i><b id="state">Arm enabled</b></span><span id="message">Ready</span></div>
  <div class="card">
    <div class="joint-head"><label for="speed">Movement speed</label><output id="speedValue">30%</output></div>
    <input id="speed" type="range" min="1" max="60" step="1" value="30">
  </div>
  <section class="grid">
    <div class="card pad-card">
      <div class="pad-title">Arm position</div>
      <div class="joystick" id="armPad"><div class="stick"></div></div>
      <div class="axis"><span>Base left/right</span><span>Shoulder up/down</span></div>
    </div>
    <div class="card pad-card">
      <div class="pad-title">Wrist position</div>
      <div class="joystick" id="wristPad"><div class="stick"></div></div>
      <div class="axis"><span>Roll left/right</span><span>Pitch up/down</span></div>
    </div>
  </section>
  <div class="card">
    <h2>Elbow</h2>
    <div class="joint-head"><span>Bend / straighten</span><output id="elbowValue">0°</output></div>
    <input id="elbow" type="range" min="0" max="155" step="1" value="0">
  </div>
  <div class="card">
    <h2>Gripper</h2>
    <div class="buttons gripper">
      <button class="good" onclick="setDegree(5,0)">Open</button>
      <button class="primary" onclick="setDegree(5,60)">Close</button>
    </div>
    <div class="joint-head" style="margin-top:12px"><span>Opening</span><output id="gripperValue">0°</output></div>
  </div>
  <details class="card">
    <summary>Precise angle controls</summary>
    <p class="sub">Use these sliders when you need an exact joint angle.</p>
    <section id="joints"></section>
  </details>
  <div class="card">
    <div class="buttons">
      <button class="primary" onclick="action('home')">Home</button>
      <button class="danger" onclick="action('stop')">Stop motion</button>
      <button onclick="action('disable')">Disable motors</button>
      <button onclick="action('enable')">Enable motors</button>
    </div>
  </div>
  <div class="card">
    <div class="joint-head"><span>Recorded tasks</span><span id="recordState">Not recording</span></div>
    <p class="sub">Start a task, set exact joint angles above (for example Elbow 35Â° and Wrist Pitch 40Â°), then press Add step. Repeat for every position before saving.</p>
    <input id="routineName" type="text" maxlength="20" placeholder="Task name (example: Pick and place)">
    <div class="buttons">
      <button id="recordButton" class="primary" onclick="startRecording()">New task</button>
      <button id="addStepButton" class="good" onclick="addStep()" disabled>Add step</button>
      <button id="saveButton" onclick="saveRecording()" disabled>Save task</button>
      <button class="danger" onclick="cancelRecording()">Cancel</button>
    </div>
    <p class="sub">Each task runs once and then stops.</p>
    <div id="routines"></div>
  </div>
  <small>RobotArm-Control · 192.168.4.1</small>
</main>
<div id="floatingRecordActions" class="record-actions" aria-hidden="true">
  <button class="good" onclick="addStep()">Add step</button>
  <button class="primary" onclick="saveRecording()">Save task</button>
</div>
<script>
const names=['Base','Shoulder','Elbow','Wrist Pitch','Wrist Roll','Gripper'];
const limits=[270,110,155,140,180,60];
let angles=[0,0,0,0,0,0],requestBusy=false;
const box=document.getElementById('joints');
const speed=document.getElementById('speed'),speedValue=document.getElementById('speedValue');
speed.oninput=()=>speedValue.textContent=`${speed.value}%`;
speed.onchange=()=>send(`/speed?value=${speed.value}`);
names.forEach((name,i)=>{
  box.insertAdjacentHTML('beforeend',`<div class="joint"><div class="joint-head"><label for="j${i}">${name}</label><output id="v${i}">0°</output></div><input id="j${i}" type="range" min="0" max="${limits[i]}" step="1" value="0"></div>`);
  const slider=document.getElementById(`j${i}`),value=document.getElementById(`v${i}`);
  slider.oninput=()=>value.textContent=`${slider.value}°`;
  slider.onchange=()=>setDegree(i,Number(slider.value));
});
async function send(url){
  const m=document.getElementById('message');m.textContent='Sending...';
  try{const r=await fetch(url);const text=await r.text();m.textContent=text;return r.ok}
  catch(e){m.textContent='Connection lost';return false}
}
function updateAngles(next){
  angles=next.map(Number);
  angles.forEach((angle,i)=>{
    const slider=document.getElementById(`j${i}`),value=document.getElementById(`v${i}`);
    slider.value=angle;value.textContent=`${angle}°`;
  });
  document.getElementById('elbow').value=angles[2];
  document.getElementById('elbowValue').textContent=`${angles[2]}°`;
  document.getElementById('gripperValue').textContent=`${angles[5]}°`;
}
async function loadState(){
  try{const data=await(await fetch('/state')).json();updateAngles(data.degrees)}
  catch(e){document.getElementById('message').textContent='Connection lost'}
}
async function setDegree(joint,degree){
  degree=Math.max(0,Math.min(limits[joint],Math.round(degree)));
  if(await send(`/set?joint=${joint}&degree=${degree}`)){angles[joint]=degree;updateAngles(angles)}
}
async function jog(j1,d1,j2,d2){
  if(requestBusy||(!d1&&!d2))return;
  requestBusy=true;
  let url=`/jog?j1=${j1}&d1=${d1}`;
  if(j2!==undefined)url+=`&j2=${j2}&d2=${d2}`;
  try{
    const r=await fetch(url),data=await r.json();
    updateAngles(data.degrees);document.getElementById('message').textContent='Moving';
  }catch(e){document.getElementById('message').textContent='Connection lost'}
  requestBusy=false;
}
function makeJoystick(id,xJoint,yJoint){
  const pad=document.getElementById(id),stick=pad.querySelector('.stick');
  let nx=0,ny=0,timer=null;
  const tick=()=>{
    const size=Math.max(Math.abs(nx),Math.abs(ny)),step=size>.72?4:2,dead=.22;
    const dx=Math.abs(nx)>dead?Math.sign(nx)*step:0;
    const dy=Math.abs(ny)>dead?Math.sign(ny)*step:0;
    jog(xJoint,dx,yJoint,dy);
  };
  const move=e=>{
    const r=pad.getBoundingClientRect(),radius=r.width*.34;
    nx=Math.max(-1,Math.min(1,(e.clientX-r.left-r.width/2)/radius));
    ny=Math.max(-1,Math.min(1,(e.clientY-r.top-r.height/2)/radius));
    stick.style.transform=`translate(${nx*95}%,${ny*95}%)`;
  };
  const stop=()=>{
    clearInterval(timer);timer=null;nx=ny=0;stick.style.transform='';
    document.getElementById('message').textContent='Ready';
  };
  pad.onpointerdown=e=>{pad.setPointerCapture(e.pointerId);move(e);tick();timer=setInterval(tick,180)};
  pad.onpointermove=e=>{if(timer)move(e)};
  pad.onpointerup=stop;pad.onpointercancel=stop;
}
makeJoystick('armPad',0,1);
makeJoystick('wristPad',4,3);
const elbow=document.getElementById('elbow');
elbow.oninput=()=>document.getElementById('elbowValue').textContent=`${elbow.value}°`;
elbow.onchange=()=>setDegree(2,Number(elbow.value));
async function action(name){
  const ok=await send(`/action?name=${name}`),enabled=name!=='disable';
  if(name==='disable'||name==='enable'){
    document.getElementById('state').textContent=enabled?'Arm enabled':'Output disabled';
    document.getElementById('dot').style.background=enabled?'#4ade80':'#fb7185';
  }
  if(ok&&name==='home')loadState();
}
function setRecordingControls(active){
  document.getElementById('recordButton').classList.toggle('recording',active);
  document.getElementById('recordButton').disabled=active;
  document.getElementById('addStepButton').disabled=!active;
  document.getElementById('saveButton').disabled=!active;
  const floating=document.getElementById('floatingRecordActions');
  floating.classList.toggle('visible',active);
  floating.setAttribute('aria-hidden',active?'false':'true');
  document.body.classList.toggle('recording-active',active);
}
async function startRecording(){
  if(!await send('/record?action=start'))return;
  document.getElementById('recordState').textContent='0 steps';
  setRecordingControls(true);
}
async function addStep(){
  const response=await fetch('/capture');
  const text=await response.text();
  document.getElementById('message').textContent=text;
  if(!response.ok)return;
  const match=text.match(/\d+/);
  if(match)document.getElementById('recordState').textContent=`${match[0]} step${match[0]==='1'?'':'s'}`;
}
async function saveRecording(){
  const field=document.getElementById('routineName');
  const name=field.value.trim()||'Saved task';
  if(!await send(`/record?action=save&name=${encodeURIComponent(name)}`))return;
  document.getElementById('recordState').textContent='Saved';
  setRecordingControls(false);
  field.value='';
  loadRoutines();
}
async function cancelRecording(){
  if(!await send('/record?action=cancel'))return;
  document.getElementById('recordState').textContent='Not recording';
  setRecordingControls(false);
}
async function playRoutine(id){
  await send(`/play?id=${id}`);
}
async function deleteRoutine(id){
  if(!confirm('Delete this saved task?'))return;
  if(await send(`/delete?id=${id}`))loadRoutines();
}
async function loadRoutines(){
  const list=document.getElementById('routines');
  try{
    const response=await fetch('/routines',{cache:'no-store'});
    if(!response.ok)throw new Error();
    const data=await response.json();
    list.replaceChildren();
    if(!data.length){
      list.innerHTML='<p class="sub">No saved tasks yet.</p>';
      return;
    }
    data.forEach(r=>{
      const row=document.createElement('div'),play=document.createElement('button'),del=document.createElement('button');
      row.className='routine';
      play.textContent=`${r.name} (${r.steps} steps)`;
      play.onclick=()=>playRoutine(r.id);
      del.className='danger';del.textContent='Delete';
      del.onclick=()=>deleteRoutine(r.id);
      row.append(play,del);list.append(row);
    });
  }catch(e){
    list.innerHTML='<p class="sub">Could not load saved tasks.</p>';
  }
}
loadState();loadRoutines();
</script></body></html>
)HTML";
