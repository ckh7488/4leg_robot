"""Sample CAD motion. Run with FreeCAD Python, optional --leg FL/FR/RL/RR.
Joint ranges are not certified by passing a finite sample.
"""
from pathlib import Path
import json,math,itertools,argparse
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1];C=json.loads((ROOT/'design.json').read_text());G=C['geometry_mm']
K=json.loads((ROOT/'cad/home-kinematics.json').read_text());P=json.loads((ROOT/'cad/assembly-instances.json').read_text())
D=A.openDocument(str(ROOT/'cad/microquad-xl330-v1.FCStd'));S={o.Label:o.Shape for o in D.Objects}
RX=lambda q:A.Rotation(A.Vector(1,0,0),math.degrees(q));RY=lambda q:A.Rotation(A.Vector(0,1,0),math.degrees(q))
def pl(p,r):return A.Placement(A.Vector(*p),r)
def grp(n):
 if '_haa_case' in n or '_haa_mount' in n:return 'trunk'
 if '_kfe_idler' in n or '_calf_' in n or n.endswith('_foot'):return 'calf'
 if '_hfe_idler' in n or '_thigh_' in n or '_kfe_case' in n:return 'thigh'
 return 'haa'
def frames(k,q):
 a,b,c=q;rh=RX(a);rt=rh.multiply(RY(-k['front']*b));rc=rh.multiply(RY(-k['front']*(b+c)));p0=A.Vector(*k['haa'])
 p1=p0+rh.multVec(A.Vector(k['front']*G['hfe_offset_x'],k['side']*G['hfe_offset_y'],0))
 p2=p1+rt.multVec(A.Vector(0,k['side']*G['knee_lateral_offset'],-G['upper_length']))
 return {'haa':pl(list(p0),rh),'thigh':pl(list(p1),rt),'calf':pl(list(p2),rc)}
def potential(a,b):
 aa=a.BoundBox;bb=b.BoundBox
 return all(min(getattr(aa,k+'Max'),getattr(bb,k+'Max'))-max(getattr(aa,k+'Min'),getattr(bb,k+'Min'))>.05 for k in ['X','Y','Z'])
parser=argparse.ArgumentParser();parser.add_argument('--leg',default='FL');args=parser.parse_args()
k=next(k for k in K['legs'] if k['name']==args.leg);old=frames(k,C['home_rad'])
moving={p['name']:grp(p['name']) for p in P if p['name'].startswith(args.leg+'_') and grp(p['name'])!='trunk'}
fixed={n:s for n,s in S.items() if n not in moving}
lims=C['joint_limits_rad'];home=C['home_rad']
grid=[sorted(set([lims[j][0],home[i],lims[j][1]])) for i,j in enumerate(['HAA','HFE','KFE'])]
results=[]
for q in itertools.product(*grid):
 new=frames(k,q);shapes={}
 for n,g in moving.items():
  sh=S[n].copy();sh.Placement=new[g].multiply(old[g].inverse()).multiply(sh.Placement);shapes[n]=sh
 hits=[]
 for n,a in shapes.items():
  candidates=list(fixed.items())+[(nn,ss) for nn,ss in shapes.items() if nn>n and moving[nn]!=moving[n]]
  for nn,b in candidates:
   if not potential(a,b):continue
   v=a.common(b).Volume
   if v>.05:hits.append({'a':n,'b':nn,'mm3':round(v,3)})
 results.append({'q_rad':list(q),'hits':hits})
out={'leg':args.leg,'sample_count':len(results),'failed_pose_count':sum(bool(x['hits']) for x in results),
'meaning':'finite pose grid against CAD solids, other legs at home; not continuous swept-volume proof','poses':results}
(ROOT/'reports'/f'motion-{args.leg}.json').write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k!='poses'}),flush=True)

import sys
sys.exit(1 if out["failed_pose_count"] else 0)
