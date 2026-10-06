const $ = id => document.getElementById(id);
let token = sessionStorage.getItem('robotToken') || '';
let state = null, online = false, sending = false, imageURL = null;
let polling = false, beating = false, loadingImage = false;
let sessionRequest = null;
const icon = name => `<i data-lucide="${name}"></i>`;
const icons = () => lucide.createIcons();
const modeName = {manual:'手动', automatic:'自动', idle:'空闲', drag:'拖动', off:'下电', on:'上电', jog:'点动', moving:'运动中'};
const actionName = {joint_target:'应用关节目标',gripper_target:'应用夹爪目标',jog:'关节点动',gripper_step:'夹爪步进',drag:'拖动示教',save_point:'记录示教点',connect_robot:'连接机械臂',disconnect_robot:'断开机械臂',connect_gripper:'连接夹爪',disconnect_gripper:'断开夹爪'};
const drafts=Array.from({length:6},()=>({dirty:false,value:0,min:0,max:0}));
const gripDraft={dirty:false,value:0,min:0,max:100};
for (let joint=1;joint<=6;joint++) {
  const row = document.createElement('div'); row.className='joint';
  row.innerHTML=`<div class="joint-title"><b>J${joint}</b><span>当前 <output id="joint-${joint}">—</output></span></div><div class="target-controls"><input id="slider-${joint}" type="range" min="-180" max="180" step="0.1" value="0" aria-label="J${joint} 目标角度滑条" disabled><input id="target-${joint}" type="number" step="0.1" value="0" aria-label="J${joint} 目标角度" disabled><button id="apply-${joint}" disabled>应用</button></div><div class="range-label"><span id="min-${joint}">—</span><span id="max-${joint}">—</span></div>`;
  $('joints').append(row);
}
for (const axis of ['X','Y','Z','Rx','Ry','Rz']) {
  const div=document.createElement('div');div.innerHTML=`<dt>${axis}</dt><dd id="tcp-${axis}">—</dd>`;$('tcp').append(div);
}
icons();

function targetControl(draft, current, bounds, ready, busy, ids, suffix){
  if(current===null){draft.dirty=false;}
  else if(!busy&&draft.dirty&&Math.abs(current-draft.value)<0.01){draft.dirty=false;}
  if(current!==null&&!draft.dirty){
    draft.min=Math.ceil(bounds[0]*10)/10;draft.max=Math.floor(bounds[1]*10)/10;
    draft.value=Math.max(draft.min,Math.min(draft.max,Math.round(current*10)/10));
  }
  const slider=$(ids.slider),number=$(ids.number),apply=$(ids.apply);
  for(const input of [slider,number]){input.min=draft.min;input.max=draft.max;input.disabled=!ready;}
  slider.value=Number.isFinite(draft.value)?draft.value:draft.min;
  if(document.activeElement!==number)number.value=Number.isFinite(draft.value)?draft.value.toFixed(1):'';
  $(ids.min).textContent=current===null?'—':draft.min.toFixed(1)+suffix;
  $(ids.max).textContent=current===null?'—':draft.max.toFixed(1)+suffix;
  apply.disabled=!ready||!draft.dirty||!Number.isFinite(draft.value)||draft.value<draft.min||draft.value>draft.max||!number.validity.valid;
}

function editTarget(draft, sliderId, numberId){
  $(sliderId).oninput=e=>{draft.dirty=true;draft.value=Number(e.target.value);$(numberId).value=draft.value.toFixed(1);render();};
  $(numberId).oninput=e=>{draft.dirty=true;draft.value=e.target.value===''?NaN:Number(e.target.value);render();};
}
for(let j=1;j<=6;j++)editTarget(drafts[j-1],'slider-'+j,'target-'+j);
editTarget(gripDraft,'grip-slider','grip-target');

function alertMessage(message) { $('alert').textContent=message||''; $('alert').hidden=!message; }
async function api(path, body) {
  const response=await fetch('/api/'+path,{method:body===undefined?'GET':'POST',headers:{'Authorization':'Bearer '+token,...(body===undefined?{}:{'Content-Type':'application/json'})},body:body===undefined?undefined:JSON.stringify(body),signal:AbortSignal.timeout(4000)});
  const result=await response.json();
  if(!response.ok) {
    if(response.status===401) { token='';sessionStorage.removeItem('robotToken'); }
    throw new Error(result.error||`HTTP ${response.status}`);
  }
  return result;
}
async function ensureSession(){
  if(token)return;
  if(!sessionRequest)sessionRequest=api('session',{}).then(r=>{
    token=r.token;sessionStorage.setItem('robotToken',token);
  }).finally(()=>{sessionRequest=null;});
  await sessionRequest;
}

function render() {
  $('network').textContent=online?'在线':'连接中断';$('network').className='badge '+(online?'live':'warn');
  $('stop').disabled=!token;
  const s=state, own=online&&s?.owner, free=own&&!s.busy&&!s.stopping&&!sending;
  $('claim').disabled=!online||sending||!!s?.busy||!!s?.stopping||(s?.occupied&&!s?.owner);
  $('claim').querySelector('span').textContent=own?'释放控制权':'取得控制权';
  $('ownership').textContent=own?'本机持有控制权':s?.occupied?'其他设备正在控制':'未取得控制权';
  $('activity').textContent=s?.stopping?'正在停止':s?.busy?'执行中':'待命';
  $('environment').textContent=s?.simulation?'模拟设备':'实机';$('environment').className='badge '+(s?.simulation?'warn':'');
  for(const kind of ['robot','gripper']) {
    $(kind+'-connect').disabled=!free;
    $(kind+'-connect').querySelector('span').textContent=s?.[kind]?.connected?'断开':'连接';
  }
  const r=s?.robot||{},g=s?.gripper||{},drag=r.operation==='drag';
  $('robot-mode').textContent='模式 '+(modeName[r.mode]||r.mode||'—');
  $('robot-power').textContent='电源 '+(modeName[r.power]||r.power||'—');
  $('robot-operation').textContent='状态 '+(r.connected?(modeName[r.operation]||r.operation):'未连接');
  for(let j=1;j<=6;j++) {
    $('joint-'+j).textContent=r.connected?Number(r.joints_deg[j-1]).toFixed(3)+'°':'—';
    const current=r.connected?r.joints_deg[j-1]:null;
    const hard=r.limits_deg?.[j-1];
    const bounds=hard&&current!==null?[Math.max(hard[0],current-50),Math.min(hard[1],current+50)]:[0,0];
    targetControl(drafts[j-1],current,bounds,free&&r.connected&&!!hard&&r.mode==='manual'&&r.operation==='idle',s?.busy,
      {slider:'slider-'+j,number:'target-'+j,apply:'apply-'+j,min:'min-'+j,max:'max-'+j},'°');
  }
  $('rate').disabled=!free||drag;
  $('drag-toggle').disabled=!free||!r.connected||r.mode!=='manual'||!['idle','drag'].includes(r.operation);
  $('drag-toggle').checked=drag;
  $('gripper-connect').disabled=!free||drag;
  $('gripper-position').innerHTML=(g.connected?(g.position/10).toFixed(1):'—')+'<small>%</small>';
  $('gripper-meter').value=g.connected?g.position:0;
  $('gripper-status').textContent=!g.connected?'未连接':g.initialized!==1?'未完成初始化':['运动中','到位 · 未夹持','已夹持','掉落'][g.status];
  const gripReady=free&&!drag&&g.connected&&g.initialized===1&&[1,2].includes(g.status);
  targetControl(gripDraft,g.connected?g.position/10:null,g.connected?[Math.max(0,g.position/10-50),Math.min(100,g.position/10+50)]:[0,100],gripReady,s?.busy,
    {slider:'grip-slider',number:'grip-target',apply:'grip-apply',min:'grip-min',max:'grip-max'},'%');
  $('save-point').disabled=!free||!r.connected||!['idle','drag'].includes(r.operation);
  $('download').disabled=!online||!s?.points?.length;
  $('camera-toggle').disabled=!own||sending;$('camera-toggle').checked=!!s?.camera?.enabled;
  const fresh=online&&s?.camera?.enabled&&Date.now()/1000-s.camera.updated<2;
  const cal=s?.calibration||{};
  $('capture-calibration').disabled=!free||!r.connected||r.mode!=='manual'||r.operation!=='idle'||!fresh||!!cal.running||!!s?.simulation;
  $('capture-calibration').querySelector('span').textContent=cal.running?'采集中…':'采集标定样本';
  $('calibration-count').textContent=`求解 ${cal.train||0} / 20 · 验证 ${cal.validation||0} / 5`;
  $('calibration-status').textContent=cal.running?'正在采集图像与法兰位姿':cal.error?'采样失败：'+cal.error:cal.message?`${cal.message} · ${cal.session}`:r.operation==='drag'?'退出拖动后可采样':cal.session||'session-01';
  $('calibration-status').classList.toggle('failed',!!cal.error);
  $('camera').hidden=!fresh||!imageURL;$('camera-empty').hidden=fresh&&!!imageURL;
  $('camera-empty').querySelector('span').textContent=s?.camera?.error?'相机异常：'+s.camera.error:s?.camera?.enabled?'相机预热 / 等待新帧':'相机未开启';
  ['X','Y','Z','Rx','Ry','Rz'].forEach((axis,i)=>$('tcp-'+axis).textContent=r.connected?Number(r.tcp_m_rad[i]).toFixed(4):'—');
  $('last-update').textContent=s?.updated?'状态 '+new Date(s.updated*1000).toLocaleTimeString():'—';
  if(!s)return;
  $('point-count').textContent=s.points.length;
  $('points').replaceChildren();
  if(!s.points.length){const p=document.createElement('p');p.className='empty';p.textContent='暂无示教点';$('points').append(p);}
  [...s.points].reverse().forEach(p=>{const div=document.createElement('div');div.className='point';const name=document.createElement('span');name.textContent=p.name;const t=document.createElement('time');t.textContent=new Date(p.time).toLocaleTimeString();div.append(name,t);$('points').append(div);});
  $('events').replaceChildren();
  s.events.slice(0,15).forEach(event=>{
    const li=document.createElement('li'),t=document.createElement('time'),text=document.createElement('span');t.textContent=new Date(event.time).toLocaleTimeString();
    const labels={accepted:'已接收',completed:'完成',control_acquired:'取得控制权',stop_requested:'请求停止',stopped:'机械臂停止处理完成',error:'错误',stop_failed:'停止失败',disconnect_failed:'断开失败'};
    text.textContent=(labels[event.event]||event.event)+(event.action?' · '+(actionName[event.action]||event.action):'')+(event.message?' · '+event.message:'')+(event.reason?' · '+event.reason:'');li.append(t,text);$('events').append(li);
  });
  if(s.error)alertMessage(s.error);
}
async function poll(){
  if(polling)return;polling=true;
  try{await ensureSession();state=await api('state');if(!online)alertMessage('');online=true;render();}
  catch(error){online=false;render();alertMessage('状态连接中断：'+error.message);}
  finally{polling=false;}
}
async function command(path,body){
  if(sending)return;
  sending=true;render();alertMessage('');
  try{await api(path,body);await poll();}
  catch(error){alertMessage(error.message);}
  finally{sending=false;render();}
}
const action=(name,args={})=>command('action',{action:name,args});
$('claim').onclick=()=>command(state?.owner?'release':'claim',{});
$('stop').onclick=async()=>{try{await api('stop',{});await poll();}catch(error){alertMessage('停止请求未确认：'+error.message);}};
for(const kind of ['robot','gripper'])$(kind+'-connect').onclick=()=>action((state?.[kind]?.connected?'disconnect_':'connect_')+kind);
for(let j=1;j<=6;j++)$('apply-'+j).onclick=()=>action('joint_target',{joint:j,target_deg:drafts[j-1].value,rate:Number($('rate').value)});
$('grip-apply').onclick=()=>action('gripper_target',{target:Math.round(gripDraft.value*10)});
$('drag-toggle').onchange=e=>action('drag',{enabled:e.target.checked});
$('camera-toggle').onchange=e=>command('camera',{enabled:e.target.checked});
$('capture-calibration').onclick=()=>command('calibration/save',{});
$('point-form').onsubmit=e=>{e.preventDefault();action('save_point',{name:$('point-name').value});};
$('download').onclick=async()=>{try{const points=await api('points');const url=URL.createObjectURL(new Blob([JSON.stringify(points,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='teaching-points.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(error){alertMessage(error.message);}};
setInterval(poll,300);
setInterval(async()=>{
  if(!token||!state?.owner||beating||document.hidden)return;beating=true;
  try{await api('heartbeat',{});}catch(error){online=false;render();alertMessage('控制心跳中断：'+error.message);}finally{beating=false;}
},1000);
setInterval(async()=>{
  if(!token||!state?.camera?.enabled||loadingImage||document.hidden)return;loadingImage=true;
  try{const response=await fetch('/api/camera.jpg',{headers:{Authorization:'Bearer '+token},signal:AbortSignal.timeout(2000)});if(!response.ok)return;const url=URL.createObjectURL(await response.blob());const old=imageURL;imageURL=url;$('camera').src=url;if(old)URL.revokeObjectURL(old);}
  catch{}finally{loadingImage=false;}
},100);
function release(){if(token&&state?.owner)fetch('/api/release',{method:'POST',headers:{Authorization:'Bearer '+token,'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{});}
document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
window.addEventListener('pagehide',release);
poll();
