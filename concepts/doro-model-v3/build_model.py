"""Build the common articulated model, URDF and MuJoCo description.

All geometry is a concept envelope; generated parts are not printable release CAD.
Run with the repository's simulation environment.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
C = json.loads((HERE/'design.json').read_text(encoding='utf8'))
G, MASS = C['geometry_m'], C['mass_allocations_kg']
COL = {'white':[.93,.92,.87,1], 'pink':[.82,.43,.59,1], 'hair':[.91,.61,.73,1],
       'dark':[.15,.19,.21,1], 'metal':[.38,.46,.47,1], 'pad':[.65,.39,.48,1],
       'eye':[.20,.12,.20,1], 'cream':[1,.94,.88,1], 'light':[1,1,1,1]}


def dump(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2)+'\n', encoding='utf8', newline='\n')


def vec(v):
    return ' '.join(f'{float(x):.10g}' for x in v)


def build():
    links=[]
    def link(name,parent=None,pos=(0,0,0),axis=None,kind=None,label='',mass=0.01,com=(0,0,0),box=(.05,.05,.05)):
        n={'name':name,'parent':parent,'origin':list(pos),'axis':axis,'kind':kind,'label':label,
           'mass':mass,'com':list(com),'inertia_box':list(box),'geoms':[],'sites':[]}
        if axis:n['limits']=list(np.radians(C['limits_deg'][kind]))
        links.append(n);return n
    def geom(n,name,t,pos,size,color='white',layer='shell',rpy=(0,0,0),collision=False):
        n['geoms'].append({'name':n['name']+'_'+name,'type':t,'pos':list(pos),'size':list(size),
                           'rpy':list(rpy),'color':COL[color],'layer':layer,'collision':collision})
    def motor(n,kind='large',pos=(0,0,0),axis='y',name='motor'):
        e=C['motor_envelopes'][kind]
        rp={'x':[0,math.pi/2,0],'y':[math.pi/2,0,0],'z':[0,0,0]}[axis]
        geom(n,name,'cylinder',pos,[e['radius_m'],e['length_m']/2],'dark','motor',rp,True)
        # A face ring helps distinguish the motor housing from a joint marker.
        shift=np.array(pos,float);shift['xyz'.index(axis)]+=e['length_m']/2+.001
        geom(n,name+'_cap','cylinder',shift,[e['radius_m']*.72,.002],'metal','motor',rp)
    def beam(n,length,radius=.013):
        geom(n,'beam','box',[0,0,-length/2],[radius,radius,length/2],'metal','structure',collision=True)
        geom(n,'cover','ellipsoid',[0,0,-length/2],[.038,.043,length/2+.01],'white','shell')
    base=link('base',mass=MASS['base'],com=[0,0,.025],box=[.23,.28,.26],label='몸통')
    geom(base,'core','box',[0,0,.025],[.10,.085,.11],'dark','structure',collision=True)
    geom(base,'cover','ellipsoid',G['body_center'],G['body_radii'],'white','shell')
    geom(base,'tether_socket','cylinder',[-.12,0,.10],[.018,.015],'metal','structure',[0,math.pi/2,0])
    joint_order=[]
    for side,tag,ko in [(-1,'R','오른쪽'),(1,'L','왼쪽')]:
        mirror=lambda p:[p[0],side*p[1],p[2]]
        pre='rear_'+tag
        a=link(pre+'_yaw','base',mirror(G['hip_yaw_origin']),[0,0,1],'hip_yaw',ko+' 골반 돌리기',MASS['hip_yaw'])
        motor(a,'medium',axis='z')
        b=link(pre+'_roll',a['name'],mirror(G['hip_roll_offset']),[1,0,0],'hip_roll',ko+' 다리 벌리기',MASS['hip_roll'])
        motor(b,axis='x')
        up,lo=G['rear_links']
        t=link(pre+'_pitch',b['name'],mirror(G['hip_pitch_offset']),[0,1,0],'hip_pitch',ko+' 다리 앞뒤',MASS['thigh'],[0,0,-.06],[.10,.06,.16])
        motor(t);motor(t,pos=[0,0,-.095],name='knee_drive');beam(t,up)
        s=link(pre+'_knee',t['name'],[0,0,-up],[0,1,0],'knee',ko+' 무릎',MASS['shank'],[0,0,-.075],[.10,.13,.16])
        beam(s,lo)
        motor(s,pos=[0,-.04,-.075],name='ankle_drive_a');motor(s,pos=[0,.04,-.075],name='ankle_drive_b')
        ap=link(pre+'_ankle_pitch',s['name'],[0,0,-lo],[0,1,0],'ankle_pitch',ko+' 발목 앞뒤',MASS['ankle_pitch'])
        geom(ap,'gimbal','box',[0,0,0],[.025,.035,.018],'metal','structure')
        ar=link(pre+'_ankle_roll',ap['name'],[0,0,0],[1,0,0],'ankle_roll',ko+' 발목 좌우',MASS['foot'],[.02,0,-.035],[.17,.115,.05])
        fs=G['rear_foot_size'];az=G['ankle_to_sole']
        geom(ar,'sole','box',[.02,0,-az+fs[2]/2],np.array(fs)/2,'pad','sole',collision=True)
        geom(ar,'foot','ellipsoid',[.023,0,-.033],[.089,.063,.028],'white','shell')
        for j in [-1,0,1]:geom(ar,'toe_'+str(j),'ellipsoid',[.078,j*.029,-.033],[.028,.021,.018],'white','shell')
        ar['sites'].append({'name':pre+'_sole','pos':[.02,0,-az]})
        joint_order += [n['name'] for n in [a,b,t,s,ap,ar]]
        pre='front_'+tag
        a=link(pre+'_roll','base',mirror(G['shoulder_roll_origin']),[1,0,0],'shoulder_roll',ko+' 어깨 벌리기',MASS['shoulder_roll'])
        motor(a,'medium',axis='x')
        up,lo=G['front_links']
        t=link(pre+'_pitch',a['name'],mirror(G['shoulder_pitch_offset']),[0,1,0],'shoulder_pitch',ko+' 팔 앞뒤',MASS['upper_arm'],[0,0,-.055],[.10,.06,.18])
        motor(t);motor(t,pos=[.06,0,-.10],name='elbow_drive');beam(t,up)
        s=link(pre+'_elbow',t['name'],[0,0,-up],[0,-1,0],'elbow',ko+' 팔꿈치',MASS['forearm'],[0,0,-.10],[.07,.07,.20])
        beam(s,lo,.011)
        geom(s,'paw','sphere',[0,0,-lo],[G['paw_radius']],'white','sole',collision=True)
        geom(s,'paw_pad','ellipsoid',[.023,0,-lo],[.022,.028,.028],'pad','detail')
        s['sites'].append({'name':pre+'_paw','pos':[0,0,-lo]})
        joint_order += [n['name'] for n in [a,t,s]]
    head=link('neck_pitch','base',G['neck_origin'],[0,1,0],'neck_pitch','고개 끄덕이기',MASS['head'],G['head_center'],[.30,.46,.36])
    motor(head,'head')
    hc=np.array(G['head_center'])
    geom(head,'head','ellipsoid',hc,G['head_radii'],'cream','shell')
    geom(head,'hair_back','ellipsoid',hc+[-.04,0,.028],[.142,.238,.175],'hair','shell')
    geom(head,'face','ellipsoid',hc+[.111,0,-.013],[.06,.168,.123],'cream','shell')
    for side in [-1,1]:
        geom(head,'bun_'+str(side),'ellipsoid',hc+[-.025,side*.232,.035],[.081,.078,.086],'pink','shell')
        geom(head,'bun_top_'+str(side),'ellipsoid',hc+[-.015,side*.245,.08],[.060,.055,.057],'hair','shell')
        geom(head,'eye_white_'+str(side),'ellipsoid',hc+[.162,side*.066,.015],[.014,.036,.043],'light','detail')
        geom(head,'eye_'+str(side),'ellipsoid',hc+[.174,side*.067,.005],[.008,.020,.027],'eye','detail')
        geom(head,'eye_glint_'+str(side),'sphere',hc+[.182,side*.061,.018],[.007],'light','detail')
        geom(head,'blush_'+str(side),'ellipsoid',hc+[.158,side*.117,-.041],[.009,.024,.012],'pink','detail')
    for i,y in enumerate(np.linspace(-.14,.14,7)):
        geom(head,'bang_'+str(i),'ellipsoid',hc+[.11,y,.105-abs(y)*.22],[.055,.032,.063 if i%2 else .074],'hair','shell',[0,.13*(i%3-1),0])
    geom(head,'mouth','ellipsoid',hc+[.173,0,-.052],[.005,.012,.006],'eye','detail')
    joint_order.append('neck_pitch')
    model={'config':C,'links':links,'joint_order':joint_order,'mass_kg':sum(n['mass'] for n in links),
           'scope':'Common visual, collision-approximation and kinematic tree; no fabrication or dynamic capability claim.'}
    assert len(joint_order)==19 and len(set(joint_order))==19
    dump(HERE/'model.json',model)
    export(model)
    return model


def export(model):
    meshes=HERE/'meshes';meshes.mkdir(exist_ok=True)
    # One reusable unit sphere preserves ellipsoid geometry in URDF as well.
    (meshes/'unit_sphere.obj').write_text(trimesh.creation.icosphere(subdivisions=3,radius=1).export(file_type='obj').rstrip()+'\n',encoding='utf8',newline='\n')
    sim=C['simulation_assumptions']
    root=ET.Element('mujoco',model='doro_v3_concept')
    ET.SubElement(root,'compiler',angle='radian',autolimits='true',inertiafromgeom='false')
    ET.SubElement(root,'option',timestep=str(sim['timestep_s']),gravity=f"0 0 {-sim['gravity_m_s2']}",integrator='implicitfast')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'joint',damping=str(sim['joint_damping']),armature=str(sim['joint_armature']))
    ET.SubElement(default,'geom',friction=vec(sim['friction']),condim='3')
    vis=ET.SubElement(root,'visual');ET.SubElement(vis,'global',offwidth='1000',offheight='1000')
    world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='1 -2 3',dir='-1 2 -3')
    ET.SubElement(world,'geom',name='floor',type='plane',size='3 3 .02',rgba='.93 .92 .89 1')
    bodies={};ur=ET.Element('robot',name='doro_v3_concept')
    ur.append(ET.Comment('Concept model: allocated inertias, approximate collision envelopes, unvalidated actuators. Base is the root link.'))
    actuators=ET.SubElement(root,'actuator')
    for n in model['links']:
        parent=bodies.get(n['parent'],world)
        pos=n['origin'] if n['parent'] else [0,0,G['standing_base_height']]
        body=ET.SubElement(parent,'body',name=n['name'],pos=vec(pos));bodies[n['name']]=body
        if n['axis']:
            ET.SubElement(body,'joint',name=n['name'],type='hinge',axis=vec(n['axis']),range=vec(n['limits']))
            f=sim['abstract_output_torque_limit_nm']
            ET.SubElement(actuators,'motor',name=n['name']+'_input',joint=n['name'],gear='1',ctrlrange=f'{-f} {f}',forcerange=f'{-f} {f}')
        else:ET.SubElement(body,'freejoint',name='floating_base')
        a,b,c=n['inertia_box'];mass=n['mass'];I=mass/12*np.array([b*b+c*c,a*a+c*c,a*a+b*b])
        n['inertia_diag']=I.tolist()
        ET.SubElement(body,'inertial',pos=vec(n['com']),mass=str(mass),diaginertia=vec(I))
        ul=ET.SubElement(ur,'link',name=n['name']);inert=ET.SubElement(ul,'inertial')
        ET.SubElement(inert,'origin',xyz=vec(n['com']),rpy='0 0 0');ET.SubElement(inert,'mass',value=str(mass))
        ET.SubElement(inert,'inertia',ixx=str(I[0]),iyy=str(I[1]),izz=str(I[2]),ixy='0',ixz='0',iyz='0')
        if n['parent']:
            uj=ET.SubElement(ur,'joint',name=n['name'],type='revolute')
            ET.SubElement(uj,'parent',link=n['parent']);ET.SubElement(uj,'child',link=n['name'])
            ET.SubElement(uj,'origin',xyz=vec(n['origin']),rpy='0 0 0');ET.SubElement(uj,'axis',xyz=vec(n['axis']))
            ET.SubElement(uj,'limit',lower=str(n['limits'][0]),upper=str(n['limits'][1]),effort=str(sim['abstract_output_torque_limit_nm']),velocity=str(sim['abstract_joint_velocity_limit_rad_s']))
        for g in n['geoms']:
            kw={'name':g['name'],'type':g['type'],'pos':vec(g['pos']),'size':vec(g['size']), 'euler':vec(g['rpy']),
                'rgba':vec(g['color']),'contype':'1' if g['collision'] else '0','conaffinity':'1' if g['collision'] else '0',
                'group':str({'structure':1,'motor':2,'shell':3,'sole':1,'detail':3}[g['layer']])}
            ET.SubElement(body,'geom',**kw)
            for tag in ['visual']+(['collision'] if g['collision'] else []):
                uv=ET.SubElement(ul,tag,name=g['name']);ET.SubElement(uv,'origin',xyz=vec(g['pos']),rpy=vec(g['rpy']));ug=ET.SubElement(uv,'geometry')
                if g['type']=='box':ET.SubElement(ug,'box',size=vec(2*np.array(g['size'])))
                elif g['type']=='cylinder':ET.SubElement(ug,'cylinder',radius=str(g['size'][0]),length=str(g['size'][1]*2))
                elif g['type']=='sphere':ET.SubElement(ug,'sphere',radius=str(g['size'][0]))
                else:ET.SubElement(ug,'mesh',filename='meshes/unit_sphere.obj',scale=vec(g['size']))
                if tag=='visual':
                    mat=ET.SubElement(uv,'material',name=g['name']+'_color');ET.SubElement(mat,'color',rgba=vec(g['color']))
        for s in n['sites']:ET.SubElement(body,'site',name=s['name'],pos=vec(s['pos']),size='.006',rgba='.8 .3 .5 1')
    for filename,tree in [('robot.xml',root),('robot.urdf',ur)]:
        ET.indent(tree,space='  ');(HERE/filename).write_text(ET.tostring(tree,encoding='unicode')+'\n',encoding='utf8',newline='\n')
    dump(HERE/'model.json',model)


if __name__=='__main__':
    model=build();print(f"Built {len(model['joint_order'])} joints, {len(model['links'])} links, {model['mass_kg']:.3f} kg allocated mass")
