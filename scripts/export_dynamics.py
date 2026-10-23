"""Aggregate CAD masses/inertias. Run with FreeCAD Python after build_cad.py."""
from pathlib import Path
import json,math
import FreeCAD as A,Part,MeshPart
ROOT=Path(__file__).resolve().parents[1]
C=json.loads((ROOT/'design.json').read_text());G=C['geometry_mm'];K=json.loads((ROOT/'cad/home-kinematics.json').read_text())
P=json.loads((ROOT/'cad/assembly-instances.json').read_text());M={p['part']:p for p in json.loads((ROOT/'print/manifest.json').read_text())}
D=A.openDocument(str(ROOT/'cad/microquad-xl330-v1.FCStd'));S={o.Label:o.Shape for o in D.Objects}
I=A.Rotation();RX=lambda t:A.Rotation(A.Vector(1,0,0),math.degrees(t));RY=lambda t:A.Rotation(A.Vector(0,1,0),math.degrees(t))
def frame(p,r=I):return A.Placement(A.Vector(*p),r)
H=K['hip_height_mm'];frames={'trunk':frame([0,0,H])}
for k in K['legs']:
 n=k['name'];a,b,c=C['home_rad']
 frames[n+'_haa']=frame(k['haa'],RX(a))
 frames[n+'_thigh']=frame(k['hfe'],RX(a).multiply(RY(-k['front']*b)))
 frames[n+'_calf']=frame(k['kfe'],RX(a).multiply(RY(-k['front']*(b+c))))
def owner(n):
 if n[:2] not in ['FL','FR','RL','RR']:return 'trunk'
 leg=n[:2]
 if '_haa_case' in n or '_haa_mount' in n:return 'trunk'
 if '_kfe_idler' in n or '_calf_' in n or n.endswith('_foot'):return leg+'_calf'
 if '_hfe_idler' in n or '_thigh_' in n or '_kfe_case' in n:return leg+'_thigh'
 return leg+'_haa'
def shape_mass(p):
 if p['group']=='envelope':return {'battery_envelope':.055,'sbc_envelope':.017,'mcu_envelope':.015,'motor_regulator_envelope':.027}[p['name']]
 if p['key']=='reference_motor':return .018
 if p['key']=='reference_idler':return .002
 return M[p['key']]['solid_mass_g']/1000
bodies={k:[] for k in frames};visual={k:[] for k in frames};coll={k:[] for k in frames}
for p in P:
 n=p['name'];k=owner(n);F=frames[k].inverse()
 sh=S[n].copy();sh.Placement=F.multiply(sh.Placement)
 m=shape_mass(p);solid=sh.Solids[0];com=list(solid.CenterOfMass);mat=solid.MatrixOfInertia;rho=m/solid.Volume
 inertia=[[getattr(mat,'A'+str(i+1)+str(j+1))*rho*1e-6 for j in range(3)] for i in range(3)]
 bodies[k].append({'name':n,'mass_kg':m,'com_mm':com,'inertia_kgm2':inertia})
 visual[k].append(sh)
 if p['key']=='reference_motor':
  T=F.multiply(frame(p['p'],A.Rotation(*p['R'])))
  center=T.multVec(A.Vector(0,-7.5,-8))
  coll[k].append({'name':n,'pos_mm':list(center),'quat_xyzw':list(T.Rotation.Q),'size_mm':[20,34,23]})
# Unmodelled fittings and wire mass are explicit lumped allocations, not CAD-derived.
for k in bodies:
 extra=(.016+.015+.020) if k=='trunk' else (.005+(.0025 if k.endswith('_haa') else 0))
 bodies[k].append({'name':'allocated_fasteners_wiring_margin','mass_kg':extra,'com_mm':[0,0,0],
                  'inertia_kgm2':[[extra*1e-5 if i==j else 0 for j in range(3)] for i in range(3)]})
out={}
(ROOT/'models/meshes').mkdir(exist_ok=True)
for k,items in bodies.items():
 mass=sum(p['mass_kg'] for p in items)
 com=[sum(p['mass_kg']*p['com_mm'][i]/1000 for p in items)/mass for i in range(3)]
 tensor=[[0.0]*3 for _ in range(3)]
 for p in items:
  r=[p['com_mm'][i]/1000-com[i] for i in range(3)];r2=sum(x*x for x in r)
  for i in range(3):
   for j in range(3):tensor[i][j]+=p['inertia_kgm2'][i][j]+p['mass_kg']*((r2 if i==j else 0)-r[i]*r[j])
 out[k]={'mass_kg':mass,'com_m':com,'inertia_kgm2':tensor,'parts':items,'motor_collision_boxes':coll[k]}
 mesh=MeshPart.meshFromShape(Shape=Part.makeCompound(visual[k]),LinearDeflection=.3,AngularDeflection=.25,Relative=False)
 mesh.write(str(ROOT/'models/meshes'/f'{k}.stl'))
result={'status':'CAD volume and allocated mass, not measured hardware','total_mass_kg':sum(x['mass_kg'] for x in out.values()),'bodies':out,
'limitations':['Uniform-density actuator envelope inertia is an estimate.','Printed solids are treated as 100% filled; actual slicer mass must replace this.','Fastener, wire, battery and board masses are allocations pending selected SKUs.','Rotor-reflected inertia is supplied separately by BAM armature, not baked into rigid link inertia.']}
(ROOT/'models/dynamics.json').write_text(json.dumps(result,indent=2))
print(json.dumps({'total_mass_kg':result['total_mass_kg'],'body_count':len(out)}))
