/* Fixed-length artistic posing rig. No motors, masses, or dynamics. Works in Node and browser. */
(function(root){'use strict';
const D=Math.PI/180, add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]), mul=(a,k)=>a.map(v=>v*k), len=a=>Math.hypot(...a), clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const modes={idle:{label:'가만히 보기',period:6,note:'몸통과 다리 비율을 봅니다. 귀와 꼬리의 작은 움직임은 외형 연출입니다.'},walk:{label:'네 발 걷기',period:1.6,note:'바닥이 움직이는 보행 미리보기입니다. 발 순서와 실루엣을 보며, 실제 균형을 계산하지 않습니다.'},sit:{label:'앉기',period:6,note:'엉덩이를 낮추고 앞발로 받치는 자세입니다. 사람처럼 몸통을 세우지 않습니다.'},crouch:{label:'몸 낮추기',period:5,note:'장난감을 바라보듯 몸을 낮춥니다. 다리 마디 길이를 유지한 외형 미리보기입니다.'},lie:{label:'엎드려 쉬기',period:6,note:'배를 낮추고 앞발을 내밉니다. 바닥 접촉력과 외피 눌림은 아직 계산하지 않습니다.'},stretch:{label:'기지개',period:5,note:'앞발을 뻗고 가슴을 낮춥니다. 허리와 어깨의 실제 구조는 후속 검토입니다.'},paw:{label:'앞발 들어 보기',period:5,note:'앉아서 한 앞발을 얼굴 쪽으로 듭니다. 발 돌리기와 혀 움직임을 포함한 세수 전체는 미구현입니다.'}};
function solveLeg(root,target,L,distal,branch){
 const v=sub(target,root);let roll=Math.atan2(v[1],-v[2]);if(roll>Math.PI/2)roll-=Math.PI;if(roll<-Math.PI/2)roll+=Math.PI;
 const radial=Math.hypot(v[1],v[2])*(v[2]>0?1:-1);
 const plane=[v[0],0,radial], last=[L[2]*Math.sin(distal),0,-L[2]*Math.cos(distal)];
 const w=sub(plane,last), r=Math.hypot(w[0],w[2]), rSafe=clamp(r,Math.abs(L[0]-L[1])+1e-8,L[0]+L[1]-1e-8);
 const a=Math.atan2(w[0],-w[2])+branch*Math.acos(clamp((L[0]**2+rSafe**2-L[1]**2)/(2*L[0]*rSafe),-1,1));
 const b=-branch*Math.acos(clamp((rSafe**2-L[0]**2-L[1]**2)/(2*L[0]*L[1]),-1,1));
 const rot=p=>[p[0],p[1]*Math.cos(roll)-p[2]*Math.sin(roll),p[1]*Math.sin(roll)+p[2]*Math.cos(roll)];
 const segments=[a,a+b,distal].map((q,i)=>rot([L[i]*Math.sin(q),0,-L[i]*Math.cos(q)]));
 const points=[root];for(const s of segments)points.push(add(points.at(-1),s));
 return {points,angles:[roll,a,b,distal-a-b],target,error:len(sub(points.at(-1),target)),lengths:L};
}
function pose(C,mode,t=0){
 const G=C.geometry_m, meta=modes[mode];if(!meta)throw Error('Unknown pose '+mode);
 const phase=t/meta.period*2*Math.PI, breath=.0008*Math.sin(phase), foot=G.paw_radii[2];
 let h=G.hip_height, tilt=0, yaw=0, frontX=.077, rearX=-.079, headPitch=0, headYaw=.10*Math.sin(phase), rearDistal=25*D, frontDistal=10*D;
 if(mode==='sit'||mode==='paw'){h=.063;tilt=30*D;frontX=.065;rearX=-.058;rearDistal=64*D;headPitch=-.10;}
 if(mode==='crouch'){const s=.5-.5*Math.cos(phase);h=.112-.024*s;tilt=-.03;frontX=.085;rearX=-.090;headPitch=.10;}
 if(mode==='lie'){h=.083;tilt=-.055;frontX=.145;rearX=-.060;rearDistal=35*D;frontDistal=65*D;headPitch=.13;}
 if(mode==='stretch'){h=.125;tilt=-20*D;frontX=.151;rearX=-.103;frontDistal=35*D;headPitch=.28;headYaw=0;}
 const hip=[G.hip_x,0,h+breath],chest=add(hip,[G.torso_span*Math.cos(tilt)*Math.cos(yaw),G.torso_span*Math.cos(tilt)*Math.sin(yaw),G.torso_span*Math.sin(tilt)]);
 const targets={},legs={};
 const offsets={HL:0,FL:.25,HR:.5,FR:.75};
 for(const name of ['FL','FR','HL','HR']){
  const front=name[0]==='F',side=name[1]==='L'?1:-1,base=front?chest:hip,width=front?G.shoulder_half_width:G.hip_half_width;
  const joint=add(base,[0,side*width,front?-.009:0]);
  let x=front?frontX:rearX,z=foot,y=side*(front?.037:.041),distal=front?frontDistal:rearDistal,stance=true;
  if(mode==='walk'){
   const W=C.walk_preview,p=((t/W.cycle_s-offsets[name])%1+1)%1,d=W.stance_fraction,step=W.stride_m*d;
   if(p<d)x+=step*(.5-p/d);else{const s=(p-d)/(1-d);x+=step*(-.5+s-Math.sin(2*Math.PI*s)/(2*Math.PI));z+=W.paw_lift_m*Math.sin(Math.PI*s)**2;stance=false;}
  }
  if(mode==='paw'&&name==='FL'){
   const s=.5-.5*Math.cos(phase);x+=(chest[0]+.069-x)*s;y=G.shoulder_half_width;z+=(chest[2]+.013-z)*s;distal=frontDistal+(95*D-frontDistal)*s;stance=s<.001;
  }
  const target=[x,y,z];targets[name]=target;
  legs[name]={...solveLeg(joint,target,front?G.front_segments:G.rear_segments,distal,front?-1:1),stance};
 }
 const head=add(chest,[G.head_offset[0],0,G.head_offset[2]]);
 return {mode,time:t,hip,chest,tilt,yaw,head,headPitch,headYaw,legs,targets,tailSway:.13*Math.sin(phase),breath,maxError:Math.max(...Object.values(legs).map(l=>l.error))};
}
const api={modes,pose,solveLeg};if(typeof module!=='undefined')module.exports=api;else root.KittenPose=api;
})(typeof window==='undefined'?globalThis:window);
