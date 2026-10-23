"""Generate SI-unit MuJoCo and URDF using CAD inertias and shared joint convention."""
from pathlib import Path
import json,math,xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
C=json.loads((ROOT/'design.json').read_text());G=C['geometry_mm'];D=json.loads((ROOT/'models/dynamics.json').read_text())
K=json.loads((ROOT/'cad/home-kinematics.json').read_text())
fmt=lambda a:' '.join(f'{x:.10g}' for x in a)
def inert(parent,name,urdf=False):
 b=D['bodies'][name];T=b['inertia_kgm2']
 if urdf:
  q=E.SubElement(parent,'inertial');E.SubElement(q,'origin',xyz=fmt(b['com_m']),rpy='0 0 0');E.SubElement(q,'mass',value=str(b['mass_kg']))
  E.SubElement(q,'inertia',**{k:str(v) for k,v in zip(['ixx','iyy','izz','ixy','ixz','iyz'],[T[0][0],T[1][1],T[2][2],T[0][1],T[0][2],T[1][2]])})
 else:E.SubElement(parent,'inertial',pos=fmt(b['com_m']),mass=str(b['mass_kg']),fullinertia=fmt([T[0][0],T[1][1],T[2][2],T[0][1],T[0][2],T[1][2]]))
def add_visual(parent,name,urdf=False):
 if urdf:
  v=E.SubElement(parent,'visual');q=E.SubElement(v,'geometry');E.SubElement(q,'mesh',filename=f'../meshes/{name}.stl',scale='.001 .001 .001')
 else:E.SubElement(parent,'geom',type='mesh',mesh=name+'_mesh',contype='0',conaffinity='0',group='2',mass='0',rgba='.22 .58 .62 1')
def add_box(parent,name,pos,sz,quat=None):
 a={'name':name,'type':'box','pos':fmt(pos),'size':fmt([v/2 for v in sz]),'group':'3','rgba':'.4 .4 .4 .15'}
 if quat is not None:a['quat']=fmt([quat[3],*quat[:3]])
 E.SubElement(parent,'geom',**a)
m=E.Element('mujoco',model='microquad_xl330_v1')
E.SubElement(m,'compiler',angle='radian',meshdir='../meshes')
E.SubElement(m,'option',timestep='.001',gravity='0 0 -9.81',iterations='80',integrator='implicitfast')
default=E.SubElement(m,'default');E.SubElement(default,'joint',limited='true',damping='.005',armature='.0016')
E.SubElement(default,'geom',friction='.8 .01 .001',condim='3',solref='.006 1',solimp='.9 .95 .001')
assets=E.SubElement(m,'asset')
for n in D['bodies']:E.SubElement(assets,'mesh',name=n+'_mesh',file=n+'.stl',scale='.001 .001 .001')
world=E.SubElement(m,'worldbody');E.SubElement(world,'light',pos='0 0 1',dir='0 0 -1')
E.SubElement(world,'geom',name='floor',type='plane',size='2 2 .1',rgba='.85 .87 .9 1')
tr=E.SubElement(world,'body',name='trunk',pos=f"0 0 {K['hip_height_mm']/1000:.10g}")
E.SubElement(tr,'freejoint',name='root');inert(tr,'trunk');add_visual(tr,'trunk')
body_nodes={'trunk':tr};joint_names=[]
for k in K['legs']:
 n=k['name'];front=k['front'];side=k['side'];ps=-front
 a=E.SubElement(tr,'body',name=n+'_haa',pos=fmt([front*G['haa_x']/1000,side*G['haa_y']/1000,0]))
 b=E.SubElement(a,'body',name=n+'_thigh',pos=fmt([front*G['hfe_offset_x']/1000,side*G['hfe_offset_y']/1000,0]))
 c=E.SubElement(b,'body',name=n+'_calf',pos=fmt([0,side*G['knee_lateral_offset']/1000,-G['upper_length']/1000]))
 for node,key,suffix,axis in [(a,n+'_haa','HAA',[1,0,0]),(b,n+'_thigh','HFE',[0,ps,0]),(c,n+'_calf','KFE',[0,ps,0])]:
  j=n+'_'+suffix;joint_names.append(j);body_nodes[key]=node
  E.SubElement(node,'joint',name=j,axis=fmt(axis),range=fmt(C['joint_limits_rad'][suffix]))
  inert(node,key);add_visual(node,key)
 E.SubElement(c,'geom',name=n+'_foot',type='sphere',pos=fmt([0,side*G['foot_lateral_offset']/1000,-G['lower_foot_center_length']/1000]),size=str(G['foot_radius']/1000),rgba='.2 .2 .2 1')
for name,b in D['bodies'].items():
 for ge in b['motor_collision_boxes']:add_box(body_nodes[name],ge['name']+'_collision',[v/1000 for v in ge['pos_mm']],[v/1000 for v in ge['size_mm']],ge['quat_xyzw'])
add_box(tr,'base_core',[0,0,(G['body_bottom_z']+1.5)/1000],[G['body_length']/1000,.036,.003])
for x in [-18,18]:add_box(tr,'base_cross_'+str(x),[x/1000,0,(G['body_bottom_z']+1.5)/1000],[.010,.096,.003])
add_box(tr,'lid',[0,0,.021],[G['body_length']/1000,.050,.002])
act=E.SubElement(m,'actuator')
for j in joint_names:E.SubElement(act,'motor',name=j+'_act',joint=j,gear='1',ctrllimited='true',ctrlrange='-.65 .65')
# Initial pose matches CAD. Joint positions are NOT servo encoder offsets.
key=E.SubElement(m,'keyframe')
E.SubElement(key,'key',name='home',qpos=fmt([0,0,K['hip_height_mm']/1000,1,0,0,0]+C['home_rad']*4))
E.indent(m);E.ElementTree(m).write(ROOT/'models/mujoco/robot.xml',encoding='utf-8',xml_declaration=True)
# URDF uses identical local inertias, joint origins, axes and limits.
u=E.Element('robot',name='microquad_xl330_v1')
for name in D['bodies']:
 l=E.SubElement(u,'link',name=name);inert(l,name,True);add_visual(l,name,True)
for k in K['legs']:
 n=k['name'];f=k['front'];s=k['side'];ps=-f
 links=[('trunk',n+'_haa','HAA',[f*G['haa_x']/1000,s*G['haa_y']/1000,0],[1,0,0]),
 (n+'_haa',n+'_thigh','HFE',[f*G['hfe_offset_x']/1000,s*G['hfe_offset_y']/1000,0],[0,ps,0]),
 (n+'_thigh',n+'_calf','KFE',[0,s*G['knee_lateral_offset']/1000,-G['upper_length']/1000],[0,ps,0])]
 for pa,ch,jj,pos,axis in links:
  j=E.SubElement(u,'joint',name=n+'_'+jj,type='revolute');E.SubElement(j,'parent',link=pa);E.SubElement(j,'child',link=ch)
  E.SubElement(j,'origin',xyz=fmt(pos),rpy='0 0 0');E.SubElement(j,'axis',xyz=fmt(axis))
  lo,hi=C['joint_limits_rad'][jj];E.SubElement(j,'limit',lower=str(lo),upper=str(hi),effort='.52',velocity=str(C['motor']['no_load_rpm']*2*math.pi/60))
E.indent(u);E.ElementTree(u).write(ROOT/'models/urdf/robot.urdf',encoding='utf-8',xml_declaration=True)
(ROOT/'control/joint-order.json').write_text(json.dumps({'joint_names':joint_names,'home_rad':C['home_rad']*4,
 'unit':'radians','axes':'HAA=+X, pitch=-front*Y','servo_encoder_mapping':'physical sign: HAA=front, HFE/KFE=-front*side; zero offsets must be measured',
 'urdf_status':'visual/inertia/joint import asset only; no Isaac contact model or actuator is silently assigned'},indent=2))
print(json.dumps({'joint_count':len(joint_names),'bodies':len(D['bodies']),'mass_kg':D['total_mass_kg']}))
