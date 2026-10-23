"""Generate prototype CAD from design.json. Units: millimetres.
Run with FreeCAD's Python. No GUI, slicing, or fabrication is performed.
"""
from pathlib import Path
import json, math, sys, itertools
import FreeCAD as A, Part, MeshPart
ROOT=Path(__file__).resolve().parents[1]
C=json.loads((ROOT/'design.json').read_text(encoding='utf-8')); G=C['geometry_mm']
for d in ['cad/parts','print/stl','reports','drawings']: (ROOT/d).mkdir(parents=True,exist_ok=True)
V=lambda a:A.Vector(*a)
I=A.Rotation()
def rx(t): return A.Rotation(A.Vector(1,0,0),math.degrees(t))
def ry(t): return A.Rotation(A.Vector(0,1,0),math.degrees(t))
def rz(t): return A.Rotation(A.Vector(0,0,1),math.degrees(t))
def mul(*rr):
 r=I
 for x in rr:r=r.multiply(x)
 return r
R_HAA=A.Rotation(A.Vector(0,1,0),A.Vector(0,0,1),A.Vector(1,0,0),'ZXY')
def r_pitch(side):return A.Rotation(A.Vector(-side,0,0),A.Vector(0,0,1),A.Vector(0,side,0),'ZXY')
def box(x,y,z,dx,dy,dz):return Part.makeBox(dx,dy,dz,A.Vector(x,y,z))
def cyl(r,h,p,axis=(0,0,1)):return Part.makeCylinder(r,h,V(p),V(axis))
def fuse(ss):
 s=ss[0]
 for b in ss[1:]:s=s.fuse(b)
 return s.removeSplitter()
def holes_z(s,xy,z,h,d=2.2):
 for x,y in xy:s=s.cut(cyl(d/2,h,(x,y,z)))
 return s.removeSplitter()
def placed(s,p,r=I):
 t=s.copy(); t.Placement=A.Placement(V(p),r).multiply(t.Placement); return t
def bbox(s):
 b=s.BoundBox;return [b.XMin,b.YMin,b.ZMin,b.XMax,b.YMax,b.ZMax]
def size(s):
 b=s.BoundBox;return [b.XLength,b.YLength,b.ZLength]
def rotp(r,p):return r.multVec(V(p))
def vecsum(a,b):return [a[i]+b[i] for i in range(3)]
q0,q1,q2=C['home_rad']
H=G['upper_length']*math.cos(q1)+G['lower_foot_center_length']*math.cos(q1+q2)+G['foot_radius']
doc=A.newDocument('Microquad_XL330_V1_Prototype')
parts={}; instances=[]; solids=[]; motor_instances=[]
def add_def(name,shape,material='PLA',color='#34b9bb',note='',print_rotation=None):
 if not shape.isValid() or len(shape.Solids)!=1:
  raise ValueError(f'{name}: invalid or disconnected solid ({len(shape.Solids)} solids)')
 shape=shape.removeSplitter()
 parts[name]={'shape':shape,'material':material,'color':color,'note':note,'print_rotation':print_rotation or I}
 return name
def add_inst(name,key,p,r=I,group='print'):
 s=placed(parts[key]['shape'],p,r)
 obj=doc.addObject('PartDesign::Feature','p_'+name);obj.Label=name;obj.Shape=s
 obj.addProperty('App::PropertyString','PrototypeStatus');obj.PrototypeStatus='FIT / LOAD TEST REQUIRED'
 obj.addProperty('App::PropertyString','PartKey');obj.PartKey=key
 instances.append({'name':name,'key':key,'p':list(p),'R':list(r.Q),'group':group,'bbox_mm':bbox(s)})
 solids.append((name,s,group))
 return s
# OEM dimensional proxy. The downloaded STEP is used separately for fit review.
motor=fuse([box(-10,-24.5,-19.5,20,34,23),cyl(8,3,(0,0,3.5))])
add_def('reference_motor',motor,'reference','#647383','OEM envelope, not a printable motor')
idler=cyl(8,3,(0,0,-22.5)).cut(cyl(3.5,5,(0,0,-23.5)))
add_def('reference_idler',idler,'reference','#85929e','Purchase genuine X330 idler / compatible qualified part')
# Flat printed spacers clear fixed-case screw heads before the rotating cheeks.
for name,th in [('front_spacer',G['front_spacer']),('back_spacer',G['back_spacer'])]:
 sh=cyl(8,th,(0,0,0)).cut(cyl(3.5,th+2,(0,0,-1)))
 sh=holes_z(sh,[(6,0),(-6,0),(0,6),(0,-6)],-1,th+2)
 add_def(name,sh,'PLA','#d8e4e9','4x M2, PCD12; fit before printing quantity')
# Each yoke is two flat cheeks and a separate bridge: no long unsupported roof.
def cheek(kind,back=False):
 z=G['back_cheek_z'] if back else G['front_cheek_z']
 if kind=='haa':ymin=-38
 elif kind=='thigh':ymin=-(G['upper_length']+14)
 else:ymin=-53
 sh=box(-12,ymin,z,24,10-ymin,3)
 # Remove windows without weakening the PCD12 output attachment.
 if kind=='thigh':
  sh=sh.cut(box(-5,-24,z-1,10,12,5))
  sh=sh.cut(box(-5,-G['upper_length']+12,z-1,10,15,5))
 elif kind=='calf':sh=sh.cut(box(-5,-40,z-1,10,25,5))
 else:sh=sh.cut(box(-4,-26,z-1,8,13,5))
 sh=holes_z(sh,[(6,0),(-6,0),(0,6),(0,-6)],z-1,5)
 sh=sh.cut(cyl(3.5,5,(0,0,z-1)))
 by=-47 if kind=='calf' else -32
 sh=holes_z(sh,[(-8,by),(8,by)],z-1,5)
 if kind=='thigh':
  # Knee motor is rotated 180deg in this cheek plane, and offset 5mm axially.
  yk=-G['upper_length']
  sh=sh.cut(cyl(8.3,5,(0,yk,z-1)))
  if not back:
   sh=holes_z(sh,[(x,yk+y) for x in [-8,8] for y in [-7.5,22.5]],z-1,5)
  else:
   # clearance window for the knee idler and cable access; rear cheek is a guard,
   # knee case is retained by its four front-case screws, not this guard.
   pass
 return sh.removeSplitter()
for kind in ['haa','thigh','calf']:
 for back in [False,True]:
  add_def(kind+('_back' if back else '_front'),cheek(kind,back),'PLA','#35b7bc',
          'Print flat. Standard 2.2mm clearance holes; verify coupon and screw engagement.')
 by=-50 if kind=='calf' else -35
 sh=box(-12,by,G['bridge_z_min'],24,6,G['bridge_z_max']-G['bridge_z_min'])
 sh=holes_z(sh,[(-8,by+3),(8,by+3)],G['bridge_z_min']-1,40)
 if kind=='calf':
  sh=sh.cut(cyl(1.1,8,(0,by-1,-8.25),(0,1,0)))
 add_def(kind+'_bridge',sh,'PLA','#2d94a1',
         'Two M2x45 through-bolts and nuts join flat cheeks; no hidden printed threads',
         rx(math.pi/2))
# Fixed HAA case plate, with a clear central aperture and a bolted base foot.
sh=fuse([box(-12,-29.5,-22.5,24,41.5,3),box(-12,-29.5,-22.5,24,3,17.5)])
sh=sh.cut(cyl(8.3,5,(0,0,-23.5)))
sh=holes_z(sh,[(x,y) for x in [-8,8] for y in [7.5,-22.5]],-23.5,5)
for x in [-8,8]:sh=sh.cut(cyl(1.1,5,(x,-30.5,-12),(0,1,0)))
add_def('haa_case_mount',sh,'PLA','#e4a74a','Use case holes, not case-assembly screws. Support needed under short foot if necessary.')
# C-shaped HFE carrier in the HAA parent frame. It clears the whole rear cheek.
for side,label in [(1,'left'),(-1,'right')]:
 def sybox(x,ya,z,dx,dy,dz):
  return box(x,ya if side==1 else -ya-dy,z,dx,dy,dz)
 f=G['hfe_offset_y']; plate=f+3.5
 sh=fuse([sybox(-20,32,9,24,3,12),sybox(-20,32,18,24,plate+3-32,3),
          sybox(-20,plate,-27,24,3,48)])
 # motor output axis and four case mounting holes
 sh=sh.cut(cyl(8.3,5,(-8,side*(plate-1),0),(0,side,0)))
 for x in [-8,8]:
  for y in [7.5,-22.5]:
   sh=sh.cut(cyl(1.1,5,(-8-side*x,side*(plate-1),y),(0,side,0)))
 # Connect carrier to HAA bridge with two through-bolts in the vertical riser.
 # These are separate from the two long Z-axis bridge bolts.
 for xx in [-16,0]:
  sh=sh.cut(cyl(1.1,5,(xx,side*31,15),(0,side,0)))
 add_def('hip_C_carrier_'+label,sh,'PLA','#e4a74a',
         'Support-required prototype. 2x M2x10/nuts to riser; front case M2x6 + washer.',
         rx(math.pi/2))
# Base, divider and lid with shared spacer holes.
spacers=[(x,y) for x in [-58,0,58] for y in [-20,20]]
for name,z,th,w in [('body_base',G['body_bottom_z'],3,G['body_width']),
                    ('electronics_deck',0,2,46),('body_lid',20,2,G['body_width'])]:
 sh=box(-86,-w/2,z,156,w,th)
 sh=holes_z(sh,spacers,z-1,th+2)
 if name=='body_base':
  for front in [1,-1]:
   for side in [1,-1]:
    # local carrier foot holes transformed through R_HAA
    for xx in [-8,8]:
     sh=sh.cut(cyl(1.1,5,(front*G['haa_x']-12,side*G['haa_y']+xx,z-1)))
  for x in [-30,14]:
   for y in [-19,17]:sh=sh.cut(box(x,y,z-1,7,2,th+2))
 if name=='body_lid':
  for x in range(-70,60,16):
   for y in [-35,27]:sh=sh.cut(box(x,y,z-1,10,8,th+2))
  for x in [-50,-30,-10,10,30,50]:sh=sh.cut(box(x,-10,z-1,10,20,th+2))
 if name=='electronics_deck':
  # Universal strap / tie slots; no claim of a specific board standoff fit.
  for x in [-60,-32,-4,24,52]:
   for y in [-16,14]:sh=sh.cut(box(x,y,z-1,6,2,th+2))
 add_def(name,sh,'PLA','#243647' if name!='electronics_deck' else '#4e6375',
         'PCB and battery held by straps and foam. Board headers remain an envelope check.')
for name,z,h in [('spacer_lower',-29.5,29.5),('spacer_upper',2,18)]:
 sh=cyl(3.2,h,(0,0,z)).cut(cyl(1.1,h+2,(0,0,z-1)))
 add_def(name,sh,'PLA','#738b9c','M2x60 through-bolt, washer and nut; do not overtighten')
# TPU foot, with a removable screw/washer retained central stem.
pad=fuse([Part.makeSphere(7,A.Vector(0,-55,-8.25)),cyl(3.8,6,(0,-51,-8.25),(0,1,0))])
pad=pad.cut(cyl(1.1,15,(0,-62,-8.25),(0,1,0)))
pad=pad.cut(cyl(2.7,7,(0,-63,-8.25),(0,1,0)))
add_def('foot_pad',pad,'TPU','#d8e4e9','TPU95A optional; start with bought rubber pad if no TPU is available',rx(math.pi/2))
# Fit coupon contains no robot-scale commitments.
coupon=box(0,0,0,48,24,3)
for i,d in enumerate([2.0,2.1,2.2,2.3,2.4]):coupon=coupon.cut(cyl(d/2,5,(5+i*8,5,-1)))
for i,d in enumerate([16.2,16.6]):coupon=coupon.cut(cyl(d/2,5,(12+i*23,15,-1)))
# Coupons with overlapping holes are deliberately avoided below by separating rows.
coupon=box(0,0,0,62,32,3)
for i,d in enumerate([2.0,2.1,2.2,2.3,2.4]):coupon=coupon.cut(cyl(d/2,5,(5+i*8,5,-1)))
for i,d in enumerate([16.2,16.6]):coupon=coupon.cut(cyl(d/2,5,(12+i*24,21,-1)))
add_def('fit_coupon',coupon,'PLA','#e4a74a','One before production; hole labels recorded in print guide')
# Kinematic transforms are exported in shaft-origin convention.
kin=[]
for name,front,side in [('FL',1,1),('FR',1,-1),('RL',-1,1),('RR',-1,-1)]:
 p0=[front*G['haa_x'],side*G['haa_y'],H]
 RH=rx(q0); RP=r_pitch(side)
 p1=vecsum(p0,list(rotp(RH,[G['hfe_offset_x'],side*G['hfe_offset_y'],0])))
 p2=vecsum(p1,list(rotp(mul(RH,ry(q1)),[0,side*G['knee_lateral_offset'],-G['upper_length']])))
 foot=vecsum(p2,list(rotp(mul(RH,ry(q1+q2)),[0,side*G['foot_lateral_offset'],-G['lower_foot_center_length']])))
 k={'name':name,'front':front,'side':side,'haa':p0,'hfe':p1,'kfe':p2,'foot_center':foot}
 kin.append(k)
 add_inst(name+'_haa_case','reference_motor',p0,R_HAA,'reference')
 add_inst(name+'_haa_idler','reference_idler',p0,R_HAA,'reference')
 add_inst(name+'_haa_mount','haa_case_mount',p0,R_HAA)
 rhaa=mul(rx(q0+side*math.pi/2),R_HAA)
 rth=mul(RH,ry(q1),RP);rcalf=mul(RH,ry(q1+q2),RP)
 add_inst(name+'_hfe_case','reference_motor',p1,mul(RH,RP),'reference')
 add_inst(name+'_hfe_idler','reference_idler',p1,mul(RH,RP),'reference')
 add_inst(name+'_kfe_case','reference_motor',p2,mul(RH,ry(q1),RP,rz(math.pi)),'reference')
 add_inst(name+'_kfe_idler','reference_idler',p2,mul(RH,ry(q1),RP,rz(math.pi)),'reference')
 for kind,p,r in [('haa',p0,rhaa),('thigh',p1,rth),('calf',p2,rcalf)]:
  for suffix in ['front','back','bridge']:add_inst(name+'_'+kind+'_'+suffix,kind+'_'+suffix,p,r)
  # Spacers are part of the child link, not the motor case.
  add_inst(name+'_'+kind+'_front_spacer','front_spacer',vecsum(p,list(rotp(r,[0,0,6.5]))),r)
  add_inst(name+'_'+kind+'_back_spacer','back_spacer',vecsum(p,list(rotp(r,[0,0,-25]))),r)
 add_inst(name+'_hip_C_carrier','hip_C_carrier_'+('left' if side==1 else 'right'),p0,RH)
 add_inst(name+'_foot','foot_pad',p2,rcalf)
for key in ['body_base','electronics_deck','body_lid']:add_inst(key,key,[0,0,H])
for i,(x,y) in enumerate(spacers):
 for key in ['spacer_lower','spacer_upper']:add_inst(f'{key}_{i}',key,[x,y,H])
# Board/battery envelopes are included in FCStd for review, but not print outputs.
for key,sz,pc in [('battery',G['battery_box'],G['battery_center']),('sbc',G['sbc_box'],G['sbc_center']),
                  ('mcu',G['mcu_box'],G['mcu_center']),('motor_regulator',[46,31,10],[-61,0,-16])]:
 sh=box(*[-s/2 for s in sz],*sz)
 add_def('envelope_'+key,sh,'envelope','#be6d5c','MODULE ENVELOPE: verify exact purchased SKU')
 add_inst(key+'_envelope','envelope_'+key,[pc[0],pc[1],H+pc[2]],I,'envelope')
doc.recompute()
doc.saveAs(str(ROOT/'cad/microquad-xl330-v1.FCStd'))
Part.export(list(doc.Objects),str(ROOT/'cad/microquad-xl330-v1.step'))
manifest=[]; mesh_preview=[]
for key,p in parts.items():
 if p['material'] in ['reference','envelope']:continue
 qty=sum(a['key']==key for a in instances)
 if key=='fit_coupon':qty=1
 # Flat cheeks need only translation. Bridges/carriers have documented orientations.
 sh=placed(p['shape'],[0,0,0],p['print_rotation'])
 b=sh.BoundBox;sh.translate(A.Vector(-b.XMin,-b.YMin,-b.ZMin))
 ob=doc.addObject('PartDesign::Feature','export_'+key);ob.Shape=p['shape'];ob.Label=key
 Part.export([ob],str(ROOT/'cad/parts'/f'{key}.step'));doc.removeObject(ob.Name)
 m=MeshPart.meshFromShape(Shape=sh,LinearDeflection=0.12,AngularDeflection=0.15,Relative=False)
 m.write(str(ROOT/'print/stl'/f'{key}.stl'))
 manifest.append({'part':key,'quantity':qty,'material':p['material'],'solid_volume_mm3':round(p['shape'].Volume,2),
                  'solid_mass_g':round(p['shape'].Volume*({'PLA':1.24,'TPU':1.21}[p['material']])/1000,3),
                  'print_bbox_mm':[round(v,3) for v in size(sh)],'note':p['note'],
                  'print_rotation_quat_xyzw':list(p['print_rotation'].Q)})
# Coarse display mesh is only visualization, never a collision oracle.
for name,s,group in solids:
 vv,tt=s.tessellate(1.0)
 key=next(a['key'] for a in instances if a['name']==name)
 mesh_preview.append({'name':name,'key':key,'group':group,'color':parts[key]['color'],
 'v':[[round(x.x,3),round(x.y,3),round(x.z,3)] for x in vv],'t':tt})
(ROOT/'cad/assembly-instances.json').write_text(json.dumps(instances,indent=2),encoding='utf-8')
(ROOT/'cad/preview-mesh.json').write_text(json.dumps(mesh_preview,separators=(',',':')),encoding='utf-8')
(ROOT/'print/manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
(ROOT/'cad/home-kinematics.json').write_text(json.dumps({'hip_height_mm':H,'legs':kin},indent=2),encoding='utf-8')
# Only exact solid validity / bed-fit is called passed here.
allshape=Part.makeCompound([s for _,s,g in solids if g!='envelope'])
report={'scope':'CAD validity and individual-part build-volume only; no fabrication release',
 'object_count':len(instances),'custom_part_types':len(manifest),'all_solids_valid':all(s.isValid() for _,s,_ in solids),
 'assembly_bbox_mm':[round(v,3) for v in size(allshape)],
 'individual_parts_fit_180mm':all(max(p['print_bbox_mm'])<=180 for p in manifest),
 'printed_solid_mass_g':round(sum(p['solid_mass_g']*p['quantity'] for p in manifest if p['part']!='fit_coupon'),2),
 'home_foot_bottom_z_mm':[round(k['foot_center'][2]-G['foot_radius'],6) for k in kin],
 'unknown':['Fastener heads/tool access and cable sweep must be checked separately.','No strength, fatigue, heat or actual printer validation.','Motor and module CAD objects are envelopes, not purchased as-built metrology.']}
(ROOT/'reports/cad-build.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2),flush=True)
