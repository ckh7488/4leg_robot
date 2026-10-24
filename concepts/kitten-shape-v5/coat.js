/* Smooth cosmetic union. The field is a display surface, not skin or collision physics. */
window.KittenCoat=function(THREE,parent,color){
 const N=80,domain=.40,zCenter=.12,step=domain/N;
 const mesh=new window.MarchingCubes(N,new THREE.MeshStandardMaterial({color,roughness:.96}),false,false,24000);
 mesh.scale.setScalar(domain/2);mesh.position.z=zCenter;mesh.castShadow=true;mesh.receiveShadow=true;parent.add(mesh);
 const smooth=9, margin=.014;
 function fieldBox(center,extent,fn){
  const lo=center.map((v,i)=>Math.max(1,Math.floor((v-extent[i]-(i===2?zCenter:0)+domain/2)/step)));
  const hi=center.map((v,i)=>Math.min(N-2,Math.ceil((v+extent[i]-(i===2?zCenter:0)+domain/2)/step)));
  const f=mesh.field;
  for(let z=lo[2];z<=hi[2];z++)for(let y=lo[1];y<=hi[1];y++)for(let x=lo[0];x<=hi[0];x++){
   const p=[x*step-domain/2,y*step-domain/2,z*step-domain/2+zCenter],d=fn(p),idx=x+y*N+z*N*N,value=80-d*1000,old=f[idx],h=Math.max(smooth-Math.abs(value-old),0)/smooth;
   f[idx]=Math.max(old,value)+h*h*smooth/4;
  }
 }
 function ell(c,r,tilt=0){const bound=Math.max(...r)+margin,co=Math.cos(tilt),si=Math.sin(tilt);fieldBox(c,[bound,bound,bound],p=>{const x=p[0]-c[0],z=p[2]-c[2],lx=x*co+z*si,lz=-x*si+z*co;return (Math.hypot(lx/r[0],(p[1]-c[1])/r[1],lz/r[2])-1)*Math.min(...r);});}
 function capsule(a,b,ra,rb){const v=b.map((x,i)=>x-a[i]),v2=v.reduce((s,x)=>s+x*x,0),r=Math.max(ra,rb)+margin,c=a.map((x,i)=>(x+b[i])/2),ext=a.map((x,i)=>Math.abs(x-b[i])/2+r);fieldBox(c,ext,p=>{const t=THREE.MathUtils.clamp(p.reduce((s,x,i)=>s+(x-a[i])*v[i],0)/v2,0,1);return Math.hypot(...p.map((x,i)=>x-a[i]-v[i]*t))-(ra+(rb-ra)*t);});}
 return {mesh,update(s){
  mesh.reset();const center=s.hip.map((x,i)=>(x+s.chest[i])/2);
  ell(center,[.091,.043,.045],s.tilt);
  ell([s.hip[0]+.006,0,s.hip[2]],[.061,.043,.042]);
  ell([s.chest[0]-.008,0,s.chest[2]],[.053,.039,.044]);
  capsule(s.chest,s.head,.028,.032);
  for(const [name,l]of Object.entries(s.legs)){
   const p=l.points,r=name[0]==='F'?[.020,.015,.009,.009]:[.030,.024,.011,.009];
   for(let i=0;i<3;i++)capsule(p[i],p[i+1],r[i],r[i+1]);
  }
  mesh.update();mesh.geometry.computeBoundingSphere();
 },finite(){return mesh.geometry.attributes.position.array.slice(0,mesh.geometry.drawRange.count*3).every(Number.isFinite);}};
};
