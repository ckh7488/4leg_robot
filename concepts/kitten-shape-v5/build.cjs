const fs=require('fs'),path=require('path'),assert=require('assert'),P=require('./pose.js');
const C=JSON.parse(fs.readFileSync(path.join(__dirname,'design.json'),'utf8'));
assert.equal(C.walk_preview.cycle_s,P.modes.walk.period,'Keep the animation loop and foot cycle in sync');
const report={scope:'Only numerical limb lengths and reach in the artistic posing rig. No collision, anatomical range, actuator, balance, or skin-fit test.',fps:60,modes:{}};
const distance=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
for(const [key,m] of Object.entries(P.modes)){
 let reach=0,lengthError=0,minPointZ=Infinity,minContacts=4;const ranges=Array.from({length:4},()=>[Infinity,-Infinity]);
 const frames=Math.ceil(m.period*report.fps)+1;
 for(let i=0;i<frames;i++){
  const pose=P.pose(C,key,i/(frames-1)*m.period);minContacts=Math.min(minContacts,Object.values(pose.legs).filter(l=>l.stance).length);
  for(const l of Object.values(pose.legs)){
   reach=Math.max(reach,l.error);
   for(let j=0;j<3;j++)lengthError=Math.max(lengthError,Math.abs(distance(l.points[j],l.points[j+1])-l.lengths[j]));
   for(const p of l.points){assert(p.every(Number.isFinite));minPointZ=Math.min(minPointZ,p[2]);}
   l.angles.forEach((a,j)=>{ranges[j][0]=Math.min(ranges[j][0],a*180/Math.PI);ranges[j][1]=Math.max(ranges[j][1],a*180/Math.PI);});
  }
 }
 assert(lengthError<1e-12);assert(reach<.0001,`${key} unreachable by ${reach}m`);assert(minPointZ>=0,`${key} joint center below floor`);
 report.modes[key]={frames,max_paw_target_error_m:reach,max_segment_length_error_m:lengthError,min_joint_center_z_m:minPointZ,min_scheduled_stance_paws:minContacts,display_angles_deg:ranges,physical_feasibility:'unverified'};
}
// Meaningful boundary checks: unreachable requests must stay visible as residuals;
// neither close nor distant targets are silently accepted by the clamping solver.
for(const target of [[0,0,-1],[0,0,-.027]])assert(P.solveLeg([0,0,0],target,[.055,.065,.027],0,-1).error>.005);
let maxStanceDrift=0;
for(let i=1;i<=960;i++){
 const t=i*C.walk_preview.cycle_s/960,dt=C.walk_preview.cycle_s/960,a=P.pose(C,'walk',t-dt),b=P.pose(C,'walk',t);
 for(const n of Object.keys(a.legs))if(a.legs[n].stance&&b.legs[n].stance){const shift=b.legs[n].points[3][0]-a.legs[n].points[3][0]+C.walk_preview.stride_m/C.walk_preview.cycle_s*dt;maxStanceDrift=Math.max(maxStanceDrift,Math.abs(shift));}
}
assert(maxStanceDrift<1e-9);report.max_scheduled_stance_foot_world_drift_per_sample_m=maxStanceDrift;
fs.writeFileSync(path.join(__dirname,'design-data.js'),'window.KITTEN_DESIGN='+JSON.stringify(C)+';\n');
fs.writeFileSync(path.join(__dirname,'validation.json'),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
