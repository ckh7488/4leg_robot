/* Cosmetic kitten ears, whiskers and passive tail. No joints or physics added. */
window.addKittenDetails=function(THREE,nodes){
 const meshes=[];
 const material=color=>new THREE.MeshStandardMaterial({color,roughness:.9,side:THREE.DoubleSide});
 const add=(node,geometry,color)=>{const m=new THREE.Mesh(geometry,material(color));nodes[node].add(m);m.castShadow=true;meshes.push(m);return m;};
 const hc=[.02,0,.04];
 for(const side of [-1,1]){
  const points=[[.045,side*.065,.065],[.045,side*.177,.065],[.008,side*.141,.215],[-.035,side*.065,.065],[-.035,side*.177,.065],[-.035,side*.141,.215]].map(p=>p.map((x,i)=>x+hc[i]));
  const ids=[0,1,2,5,4,3,0,3,4,0,4,1,1,4,5,1,5,2,2,5,3,2,3,0];
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(ids.flatMap(i=>points[i]),3));g.computeVertexNormals();add('neck_pitch',g,'#9aabb5');
  const a=new THREE.BufferGeometry();const p=[[.047,side*.085,.082],[.047,side*.158,.082],[.014,side*.139,.181]].map(p=>p.map((v,i)=>v+hc[i]));a.setAttribute('position',new THREE.Float32BufferAttribute(p.flat(),3));a.computeVertexNormals();add('neck_pitch',a,'#e9a5af');
  for(let j=-1;j<=1;j++){
   const curve=new THREE.QuadraticBezierCurve3(new THREE.Vector3(.187,side*.050,.010+j*.008),new THREE.Vector3(.187,side*.105,.009+j*.015),new THREE.Vector3(.158,side*.176,.010+j*.027));
   add('neck_pitch',new THREE.TubeGeometry(curve,12,.0008,5,false),'#64757d');
  }
 }
 const tail=new THREE.CatmullRomCurve3([[0,0,-.19],[-.025,0,-.27],[-.085,0,-.35],[-.145,0,-.37],[-.18,0,-.33]].map(p=>new THREE.Vector3(...p)));
 add('base',new THREE.TubeGeometry(tail,32,.022,10,false),'#9aabb5');
 return meshes;
};
