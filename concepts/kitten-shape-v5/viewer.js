/* Appearance study: deformable cosmetic surfaces follow a fixed-length posing rig.
   No depicted part is a selected motor, manufactured housing, or collision proof. */
(()=>{'use strict';
const $=id=>document.getElementById(id),C=window.KITTEN_DESIGN,G=C.geometry_m,P=window.KittenPose,V=p=>new THREE.Vector3(...p);
try{
let dirty=true;
const scene=new THREE.Scene();scene.background=new THREE.Color('#eae9e2');
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.04;renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;$('scene').append(renderer.domElement);
const camera=new THREE.PerspectiveCamera(32,1,.005,20);camera.up.set(0,0,1);
scene.add(new THREE.HemisphereLight('#fff9ef','#9a9e8c',2.1));
const sun=new THREE.DirectionalLight('#fff7e6',3.1);sun.position.set(.5,-.8,1.2);sun.castShadow=true;sun.shadow.mapSize.set(1024,1024);Object.assign(sun.shadow.camera,{left:-.5,right:.5,top:.5,bottom:-.5,near:.1,far:3});sun.shadow.bias=-.00015;sun.shadow.normalBias=.0005;scene.add(sun);
const fill=new THREE.DirectionalLight('#dae4eb',1.0);fill.position.set(-.6,.4,.6);scene.add(fill);
const ground=new THREE.Mesh(new THREE.PlaneGeometry(20,20),new THREE.MeshStandardMaterial({color:'#eae9e2',roughness:1}));ground.rotation.set(0,0,0);ground.position.z=-.0015;ground.receiveShadow=true;scene.add(ground);
const grid=new THREE.GridHelper(2,40,'#d0d3c6','#dcdfd3');grid.rotation.x=Math.PI/2;grid.position.z=-.001;scene.add(grid);
const skin=new THREE.Group(),rig=new THREE.Group();scene.add(skin,rig);
const fur='#b5b4a7',white='#f0f0e7',pink='#b98783',dark='#45463f';
const materials={};function mat(color){return materials[color]??=new THREE.MeshStandardMaterial({color,roughness:.92});}
const sphere=new THREE.SphereGeometry(1,36,26);
function ell(parent,p,r,color){const m=new THREE.Mesh(sphere,mat(color));m.position.fromArray(p);m.scale.fromArray(r);m.castShadow=true;m.receiveShadow=true;parent.add(m);return m;}
function tube(parent,points,r,color){const curve=new THREE.CatmullRomCurve3(points.map(V)),mesh=new THREE.Mesh(new THREE.TubeGeometry(curve,32,r,8,false),mat(color));parent.add(mesh);return mesh;}
function surface(parent,rings=40,sides=20){
 const g=new THREE.BufferGeometry(),a=new Float32Array((rings+1)*(sides+1)*3),idx=[];
 for(let i=0;i<rings;i++)for(let j=0;j<sides;j++){const n=i*(sides+1)+j;idx.push(n,n+1,n+sides+1,n+1,n+sides+2,n+sides+1);}
 g.setAttribute('position',new THREE.BufferAttribute(a,3));g.setIndex(idx);
 const m=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:fur,roughness:.97,side:THREE.DoubleSide}));parent.add(m);m.castShadow=true;m.receiveShadow=true;
 return {mesh:m,rings,sides,update(fn){const a=g.attributes.position;for(let i=0;i<=rings;i++)for(let j=0;j<=sides;j++){const p=fn(i/rings,j/sides*Math.PI*2);a.setXYZ(i*(sides+1)+j,...p);}a.needsUpdate=true;g.computeVertexNormals();g.computeBoundingSphere();}};
}
const coat=window.KittenCoat(THREE,skin,fur);
const head=new THREE.Group();skin.add(head);
const skull=ell(head,[0,0,0],G.skull_radii,fur);skull.geometry=sphere.clone();skull.material=mat(fur).clone();
const sc=new Float32Array(skull.geometry.attributes.position.count*3);
for(let i=0;i<skull.geometry.attributes.position.count;i++){const z=skull.geometry.attributes.position.getZ(i),blend=THREE.MathUtils.smoothstep(z,-.60,.05);new THREE.Color(white).lerp(new THREE.Color(fur),blend).toArray(sc,i*3);}
skull.geometry.setAttribute('color',new THREE.BufferAttribute(sc,3));skull.material.color.set('white');skull.material.vertexColors=true;
ell(head,[.037,-.009,-.015],[.017,.013,.010],white);ell(head,[.037,.009,-.015],[.017,.013,.010],white);
for(const side of [-1,1]){
 const eye=new THREE.Group();eye.position.set(.036,side*.025,.009);eye.rotation.z=side*.58;head.add(eye);
 ell(eye,[0,0,0],[.0024,.0105,.0090],dark);
 ell(eye,[.002,0,.0002],[.0020,.0094,.0079],'#9d9e73');
 ell(eye,[.0034,-side*.001,0],[.0013,.0038,.0068],'#202b28');
 ell(eye,[.0044,-.002,.0032],[.0008,.0014,.0014],'#fffefa');
 const shape=new THREE.Shape();shape.moveTo(-.020,0);shape.quadraticCurveTo(-.025,.012,-.013,.035);shape.quadraticCurveTo(-.007,.049,.001,.040);shape.quadraticCurveTo(.011,.023,.023,.002);shape.quadraticCurveTo(.002,-.006,-.020,0);
 const rotate=new THREE.Matrix4().set(0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1);
 const geo=new THREE.ExtrudeGeometry(shape,{depth:.0025,bevelEnabled:true,bevelSize:.002,bevelThickness:.0015,bevelSegments:3,steps:1,curveSegments:14});geo.applyMatrix4(rotate);
 const ear=new THREE.Group();ear.position.set(-.006,side*.030,.025);ear.rotation.x=-side*.20;head.add(ear);
 const outer=new THREE.Mesh(geo,mat(fur));outer.castShadow=true;ear.add(outer);
 const innerGeo=new THREE.ShapeGeometry(shape,20);innerGeo.applyMatrix4(rotate);innerGeo.scale(.85,.66,.74);
 const inner=new THREE.Mesh(innerGeo,new THREE.MeshStandardMaterial({color:pink,roughness:1,side:THREE.DoubleSide}));inner.position.set(.0042,0,.006);ear.add(inner);
 // Three fine whiskers on each side, swept out from the muzzle.
 for(let i=0;i<3;i++)tube(head,[[.046,side*.016,-.014-i*.003],[.047,side*.032,-.011-i*.006],[.038,side*.052,-.007-i*.009]],.00028,'#deded2');
}
const noseShape=new THREE.Shape();noseShape.moveTo(-.005,0);noseShape.quadraticCurveTo(0,.0015,.005,0);noseShape.quadraticCurveTo(.003,-.004,0,-.0045);noseShape.quadraticCurveTo(-.003,-.004,-.005,0);
const ng=new THREE.ShapeGeometry(noseShape,16);ng.applyMatrix4(new THREE.Matrix4().set(0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1));const nose=new THREE.Mesh(ng,new THREE.MeshStandardMaterial({color:pink,roughness:.7,side:THREE.DoubleSide}));nose.position.set(.054,0,-.012);head.add(nose);
tube(head,[[.053,0,-.016],[.052,0,-.021],[.050,-.004,-.023]],.00045,'#77766c');tube(head,[[.052,0,-.021],[.050,.004,-.023]],.00045,'#77766c');
const paws={},rigLines={},rigDots={};
const rigMaterial=new THREE.MeshStandardMaterial({color:'#c68c68',roughness:.6}),jointMaterial=new THREE.MeshStandardMaterial({color:'#627d70',roughness:.6});
function bone(){const m=new THREE.Mesh(new THREE.CylinderGeometry(.0018,.0018,1,10),rigMaterial);rig.add(m);return m;}
for(const name of ['FL','FR','HL','HR']){
 const p=new THREE.Group();skin.add(p);paws[name]=p;
 ell(p,[.003,0,0],G.paw_radii,white);
 for(const side of [-1,0,1])ell(p,[.017,side*.008,-.001],[.008,.006,.0085],white);
 rigLines[name]=Array.from({length:3},bone);rigDots[name]=Array.from({length:4},()=>{const m=new THREE.Mesh(new THREE.SphereGeometry(.0035,16,12),jointMaterial);rig.add(m);return m;});
}
const spine=bone(),neckBone=bone(),tail=surface(skin,45,14);
function connect(mesh,a,b){const va=V(a),vb=V(b),delta=vb.clone().sub(va);mesh.position.copy(va.add(vb).multiplyScalar(.5));mesh.scale.y=delta.length();mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),delta.normalize());}
function radiusAt(t,values){const u=t*(values.length-1),i=Math.min(values.length-2,Math.floor(u));return THREE.MathUtils.lerp(values[i],values[i+1],u-i);}
function curveSurface(surface,points,radii,flatten=1){
 const curve=new THREE.CatmullRomCurve3(points.map(V),false,'centripetal'),frames=curve.computeFrenetFrames(surface.rings,false);
 surface.update((t,a)=>{const i=Math.round(t*surface.rings),p=curve.getPointAt(t),r=radiusAt(t,radii),n=frames.normals[i],b=frames.binormals[i];return p.addScaledVector(n,Math.cos(a)*r).addScaledVector(b,Math.sin(a)*r*flatten).toArray();});
}
let mode='idle',time=0,playing=true,layer='skin',state=null,yaw=-.78,elev=.28,dist=.79;
const focus=new THREE.Vector3(.008,0,.117);
function cameraUpdate(){dirty=true;const r=dist*Math.max(1,.91/camera.aspect);camera.position.set(focus.x+r*Math.cos(elev)*Math.cos(yaw),focus.y+r*Math.cos(elev)*Math.sin(yaw),focus.z+r*Math.sin(elev));camera.lookAt(focus);}
function resize(){const r=$('scene').getBoundingClientRect();renderer.setSize(r.width,r.height,false);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();cameraUpdate();}new ResizeObserver(resize).observe($('scene'));resize();
function apply(){
 dirty=true;state=P.pose(C,mode,time);const s=state;
 coat.update(s);
 head.position.fromArray(s.head);head.rotation.set(0,s.headPitch,s.headYaw,'ZYX');
 for(const [name,l] of Object.entries(s.legs)){
  const front=name[0]==='F',points=l.points;
  paws[name].position.fromArray(points[3]);paws[name].rotation.x=0;
  points.forEach((p,i)=>rigDots[name][i].position.fromArray(p));for(let i=0;i<3;i++)connect(rigLines[name][i],points[i],points[i+1]);
 }
 const th=s.hip[2],sit=mode==='sit'||mode==='paw'||mode==='lie';
 const tp=sit?[[s.hip[0]-.045,0,th],[s.hip[0]-.080,-.025,.045],[s.hip[0]-.057,-.073,.018],[s.hip[0]+.010,-.085,.014],[s.hip[0]+.053,-.068,.018]]:[[s.hip[0]-.045,0,th],[s.hip[0]-.090,s.tailSway*.04,th+.028],[s.hip[0]-.125,s.tailSway*.08,th+.085],[s.hip[0]-.142,s.tailSway*.08,th+.126],[s.hip[0]-.127,s.tailSway*.05,th+.142]];
 curveSurface(tail,tp,[.013,.013,.011,.009,.0035]);
 connect(spine,s.hip,s.chest);connect(neckBone,s.chest,s.head);
 grid.position.x=mode==='walk'?-(time*C.walk_preview.stride_m/C.walk_preview.cycle_s)%.05:0;
 $('timeline').value=time/P.modes[mode].period*1000;$('time').textContent=time.toFixed(1)+'초';
}
function syncPlay(){$('play').textContent=playing?'멈춤':'재생';$('play').setAttribute('aria-label',playing?'일시정지':'재생');}
function setMode(key){if(!P.modes[key])throw Error('Unknown mode');mode=key;time=0;document.querySelectorAll('[data-mode]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.mode===key)));$('modeLabel').textContent=P.modes[key].label;$('modeNote').textContent=P.modes[key].note;apply();}
function setLayer(key){dirty=true;layer=key;skin.visible=key==='skin';rig.visible=key==='rig';document.querySelectorAll('[data-layer]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.layer===key)));}
for(const [key,meta] of Object.entries(P.modes)){const b=document.createElement('button');b.dataset.mode=key;b.textContent=meta.label;b.onclick=()=>setMode(key);$('modes').append(b);}
document.querySelectorAll('[data-layer]').forEach(b=>b.onclick=()=>setLayer(b.dataset.layer));
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{const v=b.dataset.view;yaw=v==='side'?-Math.PI/2:v==='front'?0:-.78;elev=v==='top'?Math.PI/2-.001:v==='three'?.28:.045;cameraUpdate();document.querySelectorAll('[data-view]').forEach(x=>x.setAttribute('aria-pressed',String(x===b)));});
$('play').onclick=()=>{playing=!playing;syncPlay();};$('timeline').oninput=e=>{playing=false;time=Number(e.target.value)/1000*P.modes[mode].period;apply();syncPlay();};
let drag=null;renderer.domElement.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};renderer.domElement.setPointerCapture(e.pointerId);});renderer.domElement.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-drag.x)*.007;elev=THREE.MathUtils.clamp(elev+(e.clientY-drag.y)*.006,.02,1.56);drag={x:e.clientX,y:e.clientY};cameraUpdate();});for(const event of ['pointerup','pointercancel'])renderer.domElement.addEventListener(event,()=>drag=null);renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();dist=THREE.MathUtils.clamp(dist*Math.exp(e.deltaY*.001),.45,1.5);cameraUpdate();},{passive:false});
setMode('idle');setLayer('skin');syncPlay();
const neutral=P.pose(C,'idle',0),noseX=neutral.head[0]+.054,rumpX=neutral.hip[0]-.057;
$('lengthValue').textContent=((noseX-rumpX)*100).toFixed(0)+' cm';$('sizeNote').textContent='기본 자세의 등 높이 약 '+((G.hip_height+.049)*100).toFixed(0)+'cm. 외형을 비교하기 위한 임시 치수입니다.';
let last=performance.now(),lastPose=0;
function frame(now){const dt=Math.max((now-last)/1000,0);last=now;if(playing){time=(time+dt*Number($('speed').value))%P.modes[mode].period;if(now-lastPose>45){apply();lastPose=now;}}if(dirty){renderer.render(scene,camera);dirty=false;}requestAnimationFrame(frame);}requestAnimationFrame(frame);
window.KittenDebug={ready:true,setMode,setLayer,pause:()=>{playing=false;syncPlay();},seek:t=>{playing=false;time=THREE.MathUtils.clamp(t,0,P.modes[mode].period);apply();syncPlay();},state:()=>({mode,layer,time,pose:state}),finite:()=>coat.finite()&&tail.mesh.geometry.attributes.position.array.every(Number.isFinite),bounds:()=>{skin.updateMatrixWorld(true);const b=new THREE.Box3();skin.traverse(m=>{if(!m.isMesh||!m.visible)return;const a=m.geometry.attributes.position,count=Math.min(a.count,m.geometry.drawRange.count);for(let i=0;i<count;i++)b.expandByPoint(new THREE.Vector3().fromBufferAttribute(a,i).applyMatrix4(m.matrixWorld));});return {min:b.min.toArray(),max:b.max.toArray()};}};
}catch(e){$('error').hidden=false;$('error').textContent=e.message;console.error(e);}
})();
