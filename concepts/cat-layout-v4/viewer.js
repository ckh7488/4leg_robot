/* Offline viewer. Joint transforms are shared with generated URDF/MuJoCo. */
(()=>{'use strict';
const $=id=>document.getElementById(id),M=window.DORO_MODEL,clips=window.DORO_MOTIONS;
try{
if(!window.THREE||!M||!clips)throw Error('모델 파일을 불러오지 못했습니다. 폴더 전체가 같은 위치에 있는지 확인해 주세요.');
const scene=new THREE.Scene();scene.background=new THREE.Color('#ebe7e1');
const renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.13;
$('scene').append(renderer.domElement);
const camera=new THREE.PerspectiveCamera(33,1,.01,20);camera.up.set(0,0,1);
scene.add(new THREE.HemisphereLight('#fff8f4','#9ba5a2',2.1));
const sun=new THREE.DirectionalLight('#fff9ee',3.3);sun.position.set(1.3,-1.5,2.3);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);sun.shadow.camera.left=-1.1;sun.shadow.camera.right=1.1;sun.shadow.camera.top=1.1;sun.shadow.camera.bottom=-1.1;sun.shadow.camera.near=.1;sun.shadow.camera.far=6;sun.shadow.bias=-.0002;scene.add(sun);
const fill=new THREE.DirectionalLight('#ffdbea',1.0);fill.position.set(-1,1,1.2);scene.add(fill);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(20,20),new THREE.MeshStandardMaterial({color:'#ebe7e1',roughness:1}));floor.position.z=-.003;floor.receiveShadow=true;scene.add(floor);
const grid=new THREE.GridHelper(2.4,24,'#d8d0c9','#dfd8d1');grid.rotation.x=Math.PI/2;grid.position.z=-.002;scene.add(grid);
const robot=new THREE.Group();scene.add(robot);const nodes={},meshes=[],markers={},jointLookup={};
M.joint_order.forEach((n,i)=>jointLookup[n]=i);
function shape(g){
 let geometry;
 if(g.type==='box')geometry=new THREE.BoxGeometry(...g.size.map(v=>v*2));
 else if(g.type==='cylinder'){geometry=new THREE.CylinderGeometry(g.size[0],g.size[0],g.size[1]*2,32);geometry.rotateX(Math.PI/2);}
 else {geometry=new THREE.SphereGeometry(1,32,20);geometry.scale(...(g.type==='sphere'?[g.size[0],g.size[0],g.size[0]]:g.size));}
 const material=new THREE.MeshStandardMaterial({color:new THREE.Color(...g.color.slice(0,3)),roughness:g.layer==='motor'?.38:.69,metalness:g.layer==='motor'?.45:0});
 const mesh=new THREE.Mesh(geometry,material);mesh.position.fromArray(g.pos);mesh.rotation.set(...g.rpy,'ZYX');mesh.castShadow=true;mesh.receiveShadow=true;mesh.userData.layer=g.layer;mesh.userData.normalColor=material.color.clone();return mesh;
}
for(const n of M.links){
 const node=new THREE.Group();node.name=n.name;node.position.fromArray(n.origin);nodes[n.name]=node;(n.parent?nodes[n.parent]:robot).add(node);
 for(const g of n.geoms){const mesh=shape(g);mesh.name=g.name;mesh.userData.joint=n.axis?n.name:null;node.add(mesh);meshes.push(mesh);}
 if(n.axis){const dot=new THREE.Mesh(new THREE.SphereGeometry(.012,16,12),new THREE.MeshBasicMaterial({color:'#cf598b',transparent:true,opacity:.85,depthTest:false}));dot.renderOrder=10;dot.visible=false;node.add(dot);markers[n.name]=dot;}
}
const details=window.addKittenDetails(THREE,nodes);
const com=new THREE.Mesh(new THREE.SphereGeometry(.010,14,10),new THREE.MeshBasicMaterial({color:'#d88a36',depthTest:false}));com.renderOrder=11;com.visible=false;scene.add(com);
const targetDots=Array.from({length:4},()=>{const d=new THREE.Mesh(new THREE.SphereGeometry(.009,12,8),new THREE.MeshBasicMaterial({color:'#438d88',transparent:true,opacity:.7}));scene.add(d);return d;});
let mode='quad',layer='skin',time=0,playing=true,manual=false,selected='front_L_roll',currentFrame=null;
let yaw=-.70,elevation=.30,distance=1.95;const look=new THREE.Vector3(.02,0,.35);
function cameraUpdate(){const r=distance*Math.max(1,.95/camera.aspect);camera.position.set(look.x+r*Math.cos(elevation)*Math.cos(yaw),look.y+r*Math.cos(elevation)*Math.sin(yaw),look.z+r*Math.sin(elevation));camera.lookAt(look);}
cameraUpdate();
function resize(){const r=$('scene').getBoundingClientRect();renderer.setSize(r.width,r.height,false);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();cameraUpdate();}new ResizeObserver(resize).observe($('scene'));resize();
const descriptions={quad:'새끼고양이처럼 몸통을 눕히고 네 발로 섭니다. 앞뒤 다리를 몸 양옆에 나란히 배치했습니다.',stand:'뒷다리로 몸을 세우고 짧은 앞발을 접은 자세입니다. 실제 균형 제어는 아직 없습니다.',arms:'일어선 상태에서 짧은 앞다리를 양옆으로 벌립니다. 어깨 벌림과 팔꿈치가 함께 움직입니다.',transition:'네 발·두 발 사이의 관절 경로 제안입니다. 스스로 일어나는 힘과 균형은 검증 전입니다.'};
for(const [key,clip] of Object.entries(clips)){const b=document.createElement('button');b.type='button';b.textContent=clip.label;b.dataset.mode=key;b.onclick=()=>setMode(key);$('modes').append(b);}
for(const name of M.joint_order){const n=M.links.find(n=>n.name===name);const o=document.createElement('option');o.value=name;o.textContent=n.label;$('jointSelect').append(o);}
$('massTag').textContent='크기·모터·질량 미정';
function updateValidation(){if(manual){$('validation').textContent='직접 조작 중 · 이 자세의 간섭과 균형은 검사하지 않았습니다.';return;}const r=clips[mode].report;const pairs=Object.keys(r.overlapping_collision_primitive_pairs_below_1mm).length;const ik=r.within_joint_limits&&r.max_site_position_error_m<.0002&&r.max_rear_foot_orientation_error_rad<.003;$('validation').textContent=(ik?'지정한 손·발 위치와 관절 범위 확인. ':'일부 손·발 위치에 도달하지 못함. ')+(pairs?`부품 외형 겹침 ${pairs}쌍 — 구조 수정 필요. `:'검사한 충돌 도형의 큰 겹침 없음. ')+(r.floor_penetration_below_1mm_m<-.001?'바닥 침범 — 경로 수정 필요. ':'')+'외피·힘·균형은 미검증.';}
function setMode(key){mode=key;look.z=key==='quad'?.25:.40;distance=key==='quad'?1.65:2.10;cameraUpdate();time=0;manual=false;playing=clips[key].period>0;document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode===key));$('currentMode').textContent=clips[key].label;$('modeNote').textContent=descriptions[key];updateValidation();applyAt(0);syncPlay();}
function syncPlay(){$('play').textContent=playing?'멈춤':'재생';$('play').setAttribute('aria-label',playing?'동작 일시정지':'동작 재생');}
function setLayer(key){layer=key;for(const mesh of details){mesh.visible=key!=='skeleton';mesh.material.transparent=key==='xray';mesh.material.opacity=key==='xray'?.15:1;mesh.material.depthWrite=key!=='xray';}document.querySelectorAll('[data-layer]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.layer===key)));for(const mesh of meshes){const cosmetic=['shell','detail'].includes(mesh.userData.layer);const oldLimbCover=mesh.name.endsWith('_fur')||(mesh.name.endsWith('_cover')&&mesh.name!=='base_cover');mesh.visible=!oldLimbCover&&!(key==='skeleton'&&cosmetic)&&!(key==='skin'&&['motor','structure'].includes(mesh.userData.layer));mesh.material.transparent=key==='xray'&&cosmetic;mesh.material.opacity=key==='xray'&&cosmetic?.15:1;mesh.material.depthWrite=!(key==='xray'&&cosmetic);mesh.castShadow=key==='skin'||!cosmetic;}for(const [name,dot] of Object.entries(markers)){dot.visible=key!=='skin';dot.scale.setScalar(name===selected?1.6:1);}com.visible=false;targetDots.forEach(d=>d.visible=key!=='skin'&&!manual);}
function updateJoint(){const n=M.links.find(n=>n.name===selected);$('jointSelect').value=selected;$('jointRange').min=n.limits[0]*180/Math.PI;$('jointRange').max=n.limits[1]*180/Math.PI;$('jointLimits').textContent=`${Math.round(n.limits[0]*180/Math.PI)}° ~ ${Math.round(n.limits[1]*180/Math.PI)}°`;$('selectedLabel').textContent=n.label;for(const [name,dot] of Object.entries(markers))dot.scale.setScalar(name===selected?1.6:1);syncAngle();}
function syncAngle(){if(!currentFrame)return;const a=currentFrame.q[jointLookup[selected]]*180/Math.PI;$('jointRange').value=a;$('jointAngle').textContent=a.toFixed(1)+'°';}
function apply(f){currentFrame=f;robot.position.fromArray(f.base);robot.quaternion.set(f.quat[1],f.quat[2],f.quat[3],f.quat[0]);for(let i=0;i<M.joint_order.length;i++){const name=M.joint_order[i],n=M.links.find(n=>n.name===name);nodes[name].quaternion.setFromAxisAngle(new THREE.Vector3(...n.axis),f.q[i]);}robot.updateMatrixWorld(true);details.update();const center=new THREE.Vector3();for(const n of M.links)center.addScaledVector(nodes[n.name].localToWorld(new THREE.Vector3(...n.com)),n.mass/M.mass_kg);com.position.copy(center);targetDots.forEach((d,i)=>{d.position.fromArray(f.targets[i]);d.visible=layer!=='skin'&&!manual;});syncAngle();}
function applyAt(t){const clip=clips[mode],f=t*clip.fps,a=Math.min(Math.floor(f),clip.frames.length-1),b=Math.min(a+1,clip.frames.length-1),u=f-a;const A=clip.frames[a],B=clip.frames[b];const lerp=(v,w)=>v.map((n,i)=>n+(w[i]-n)*u);const q1=new THREE.Quaternion(A.quat[1],A.quat[2],A.quat[3],A.quat[0]),q2=new THREE.Quaternion(B.quat[1],B.quat[2],B.quat[3],B.quat[0]);q1.slerp(q2,u);apply({base:lerp(A.base,B.base),quat:[q1.w,q1.x,q1.y,q1.z],q:lerp(A.q,B.q),com:lerp(A.com,B.com),targets:A.targets.map((v,i)=>lerp(v,B.targets[i]))});$('timeline').value=clip.period?t/clip.period*1000:0;$('time').textContent=t.toFixed(1)+' / '+clip.period+'초';}
$('jointSelect').onchange=e=>{selected=e.target.value;updateJoint();};
$('jointRange').oninput=e=>{playing=false;manual=true;syncPlay();const f=structuredClone(currentFrame);f.q[jointLookup[selected]]=Number(e.target.value)*Math.PI/180;apply(f);updateValidation();};
$('play').onclick=()=>{if(manual){manual=false;applyAt(time);updateValidation();}playing=!playing;syncPlay();};
$('timeline').oninput=e=>{playing=false;manual=false;time=Number(e.target.value)/1000*clips[mode].period;applyAt(time);syncPlay();updateValidation();};
document.querySelectorAll('[data-layer]').forEach(b=>b.onclick=()=>setLayer(b.dataset.layer));
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{yaw=b.dataset.view==='front'?0:b.dataset.view==='side'?-Math.PI/2:-.60;elevation=b.dataset.view==='three'?.23:.06;distance=mode==='quad'?1.65:2.10;cameraUpdate();document.querySelectorAll('[data-view]').forEach(x=>x.classList.toggle('active',x===b));});
let drag=null;renderer.domElement.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};renderer.domElement.setPointerCapture(e.pointerId);});renderer.domElement.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-drag.x)*.007;elevation=Math.max(-.05,Math.min(1.2,elevation+(e.clientY-drag.y)*.005));drag={x:e.clientX,y:e.clientY};cameraUpdate();});renderer.domElement.addEventListener('pointerup',()=>drag=null);renderer.domElement.addEventListener('pointercancel',()=>drag=null);renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();distance=Math.max(.9,Math.min(3.5,distance*Math.exp(e.deltaY*.001)));cameraUpdate();},{passive:false});
setMode(mode);setLayer(layer);updateJoint();let last=performance.now();
function frame(now){const dt=Math.max(0,Math.min((now-last)/1000,.08));last=now;if(playing&&clips[mode].period){time=(time+dt*Number($('speed').value))%clips[mode].period;applyAt(time);}renderer.render(scene,camera);requestAnimationFrame(frame);}renderer.render(scene,camera);requestAnimationFrame(frame);
window.DoroDebug={surfacesFinite:()=>details.every(m=>m.geometry.getAttribute('position').array.every(Number.isFinite)),setMode,setLayer,pause:()=>{playing=false;syncPlay();},seek:t=>{playing=false;time=t;applyAt(t);syncPlay();},state:()=>({mode,layer,time,manual,q:currentFrame.q,base:currentFrame.base,joints:M.joint_order}),point:(name,local=[0,0,0])=>{robot.updateMatrixWorld(true);return nodes[name].localToWorld(new THREE.Vector3(...local)).toArray();},ready:true};
}catch(error){$('error').hidden=false;$('error').textContent=error.message;console.error(error);}
})();
