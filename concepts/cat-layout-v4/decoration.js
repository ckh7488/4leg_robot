/* Cosmetic rounded ears, passive tail and deforming limb surfaces.
 * These surfaces are not collision geometry or a skin-strain simulation. */
window.addKittenDetails=function(THREE,nodes){
 const meshes=[],fur='#f7f9fa';
 const material=color=>new THREE.MeshStandardMaterial({color,roughness:.95,side:THREE.DoubleSide});
 const add=(node,geometry,color)=>{const m=new THREE.Mesh(geometry,material(color));nodes[node].add(m);m.castShadow=true;meshes.push(m);return m;};
 function earShape(scale=1){const s=new THREE.Shape();s.moveTo(-.047*scale,0);s.quadraticCurveTo(-.058*scale,.009*scale,-.043*scale,.034*scale);s.lineTo(.010*scale,.101*scale);s.quadraticCurveTo(.021*scale,.115*scale,.026*scale,.094*scale);s.lineTo(.050*scale,.012*scale);s.quadraticCurveTo(.054*scale,-.004*scale,.030*scale,-.004*scale);s.closePath();return s;}
 for(const side of [-1,1]){
  const rotate=new THREE.Matrix4().set(0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1);
  const g=new THREE.ExtrudeGeometry(earShape(),{depth:.018,bevelEnabled:true,bevelSegments:3,steps:1,bevelSize:.006,bevelThickness:.008,curveSegments:12});g.applyMatrix4(rotate);
  const ear=add('neck_pitch',g,fur);ear.position.set(.015,side*.105,.128);ear.rotation.x=side*.14;
  const inner=new THREE.ShapeGeometry(earShape(.70),20);inner.applyMatrix4(rotate);
  const e=add('neck_pitch',inner,'#e9bec5');e.position.set(.043,side*.105,.140);e.rotation.x=side*.14;
 }
 const tailPoints=[[0,0,-.19],[-.025,0,-.27],[-.085,0,-.35],[-.145,0,-.37],[-.18,0,-.33]].map(p=>new THREE.Vector3(...p));
 const tail=new THREE.CatmullRomCurve3(tailPoints);add('base',new THREE.TubeGeometry(tail,40,.021,12,false),fur);
 const cap=add('base',new THREE.SphereGeometry(.021,20,14),fur);cap.position.copy(tailPoints[4]);
 const limbs=[];
 for(const tag of ['R','L']){
  const front=[['front_'+tag+'_roll',[0,0,0]],['front_'+tag+'_pitch',[.020,0,-.035]],['front_'+tag+'_pitch',[.020,0,-.080]],['front_'+tag+'_elbow',[0,0,0]],['front_'+tag+'_elbow',[0,0,-.075]],['front_'+tag+'_elbow',[0,0,-.130]]];
  const rear=[['rear_'+tag+'_pitch',[0,0,.010]],['rear_'+tag+'_pitch',[0,0,-.070]],['rear_'+tag+'_knee',[0,0,0]],['rear_'+tag+'_knee',[0,0,-.075]],['rear_'+tag+'_ankle_roll',[0,0,.005]]];
  for(const [anchors,radii] of [[front,[.046,.053,.054,.046,.040,.033]],[rear,[.057,.053,.052,.061,.036]]]){const caps=[0,radii.length-1].map(i=>add('base',new THREE.SphereGeometry(radii[i],24,18),fur));limbs.push({anchors,radii,caps,mesh:add('base',new THREE.BufferGeometry(),fur)});}
 }
 meshes.update=()=>{
  for(const limb of limbs){
   const points=limb.anchors.map(([n,p])=>nodes.base.worldToLocal(nodes[n].localToWorld(new THREE.Vector3(...p))));
   limb.caps[0].position.copy(points[0]);limb.caps[1].position.copy(points[points.length-1]);
   const curve=new THREE.CatmullRomCurve3(points,false,'centripetal');
   const segments=60,radial=24,g=new THREE.TubeGeometry(curve,segments,1,radial,false),a=g.attributes.position;
   for(let i=0;i<=segments;i++){
    const t=i/segments,center=curve.getPointAt(t),u=t*(limb.radii.length-1),j=Math.min(Math.floor(u),limb.radii.length-2),r=THREE.MathUtils.lerp(limb.radii[j],limb.radii[j+1],u-j);
    for(let k=0;k<=radial;k++){const q=i*(radial+1)+k,v=new THREE.Vector3().fromBufferAttribute(a,q).sub(center).multiplyScalar(r).add(center);a.setXYZ(q,v.x,v.y,v.z);}
   }
   g.computeVertexNormals();limb.mesh.geometry.dispose();limb.mesh.geometry=g;
  }
 };
 return meshes;
};
