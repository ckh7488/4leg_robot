"""Prescribed-base kinematic previews, using MuJoCo Jacobians for inverse kinematics.

No time integration or feedback controller drives these animations.
"""
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from build_model import build, dump

HERE=Path(__file__).resolve().parent


def ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def generate():
    model=build();C=model['config'];G=C['geometry_m'];spec=C['motion_sampling']
    m=mujoco.MjModel.from_xml_path(str(HERE/'robot.xml'));d=mujoco.MjData(m)
    joints=model['joint_order'];ji=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in joints]
    qa=m.jnt_qposadr[ji];va=m.jnt_dofadr[ji]
    limits=np.array([m.jnt_range[j] for j in ji]);lookup={name:i for i,name in enumerate(joints)}
    chains={}
    for tag in ['R','L']:
        for limb in ['rear','front']:
            prefix=limb+'_'+tag
            names=[n for n in joints if n.startswith(prefix+'_')]
            site=prefix+('_sole' if limb=='rear' else '_paw')
            chains[prefix]=([lookup[n] for n in names],mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,site))

    def solve(chain,target,orientation=False):
        ids,sid=chains[chain];dq=qa[ids];dv=va[ids]
        for _ in range(100):
            mujoco.mj_forward(m,d)
            ep=np.array(target)-d.site_xpos[sid]
            r=d.site_xmat[sid].reshape(3,3)
            er=.5*sum(np.cross(r[:,i],np.eye(3)[:,i]) for i in range(3))
            if np.linalg.norm(ep)<2e-6 and (not orientation or np.linalg.norm(er)<2e-5):break
            jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jacSite(m,d,jp,jr,sid)
            J=np.vstack([jp[:,dv],.15*jr[:,dv]]) if orientation else jp[:,dv]
            e=np.r_[ep,.15*er] if orientation else ep
            change=J.T@np.linalg.solve(J@J.T+np.eye(len(e))*2e-6,e)
            d.qpos[dq]=np.clip(d.qpos[dq]+np.clip(change,-.14,.14),limits[ids,0],limits[ids,1])
        mujoco.mj_forward(m,d)
        r=d.site_xmat[sid].reshape(3,3)
        angle=math.acos(np.clip((np.trace(r)-1)/2,-1,1))
        return float(np.linalg.norm(np.array(target)-d.site_xpos[sid])),angle if orientation else 0

    def initial():
        mujoco.mj_resetData(m,d)
        for i,n in enumerate(joints):
            if n.endswith('_knee'):d.qpos[qa[i]]=.6
            elif n.endswith('_elbow'):d.qpos[qa[i]]=1.7
            elif n.startswith('rear') and n.endswith('_pitch') and 'ankle' not in n:d.qpos[qa[i]]=-.3
        d.qpos[qa[-1]]=0

    def solve_arm(tag, side, target, base, R):
        # Exact displaced-roll-axis IK. The outboard shoulder offset rotates
        # with roll; treating it as a fixed shoulder position is incorrect.
        h=np.array(G['shoulder_roll_origin'],float);h[1]*=side
        x,y,z=R.T@(target-base)-h
        offset=side*G['shoulder_pitch_offset'][1]
        plane_z=-math.sqrt(max(0,y*y+z*z-offset*offset))
        roll=math.atan2(z,y)-math.atan2(plane_z,offset)
        roll=(roll+math.pi)%(2*math.pi)-math.pi
        l1,l2=G['front_links']
        cosine=(x*x+plane_z*plane_z-l1*l1-l2*l2)/(2*l1*l2)
        elbow=math.acos(np.clip(cosine,-1,1))
        pitch=math.atan2(-x,-plane_z)+math.atan2(l2*math.sin(elbow),l1+l2*math.cos(elbow))
        ids,sid=chains['front_'+tag]
        angles=np.array([roll,pitch,elbow])
        d.qpos[qa[ids]]=np.clip(angles,limits[ids,0],limits[ids,1])
        mujoco.mj_forward(m,d)
        return float(np.linalg.norm(target-d.site_xpos[sid])),0

    modes=[('quad','네 발 기본자세',0),('stand','두 발로 서기',0),('arms','짧은 팔 벌리기',6),('transition','자세 바꾸기',8)]
    clips={};reports={}
    for mode,label,period in modes:
        initial();frames=[];maxpos=0;maxrot=0;collisions={};floor_pen=0;minheight=10
        count=round(period*spec['fps']) if period else 0
        for i in range(count+1):
            time=i/spec['fps'];phase=2*math.pi*time/(period or 1)
            blend=(1-math.cos(phase))/2 if mode=='transition' else (1 if mode=='quad' else 0)
            phi=math.radians(90)*blend
            base=np.array([0.,0.,G['standing_base_height']*(1-blend)+G['quad_base_height']*blend]);R=ry(phi)
            if mode=='dance':base[1]=.018*math.sin(phase);base[2]-=.016*(1-math.cos(phase*2))/2
            if mode=='squat':base[2]-=.065*(1-math.cos(phase))/2
            if mode=='step':base[1]=-.045*math.sin(phase);base[2]-=.012*(1-math.cos(phase*2))/2
            d.qpos[:3]=base;d.qpos[3:7]=[math.cos(phi/2),0,math.sin(phi/2),0]
            d.qpos[qa[lookup['neck_pitch']]]=-phi+(.04*math.sin(phase) if mode=='dance' else 0)
            desired=[]
            for side,tag,j in [(-1,'R',0),(1,'L',1)]:
                foot=np.array(G['rear_sole_targets'][j],float)*(1-blend)+np.array(G['quad_rear_sole_targets'][j])*blend
                if mode=='step':foot[2]=.024*max(0,side*math.sin(phase))**2
                e,o=solve('rear_'+tag,foot,True);maxpos=max(maxpos,e);maxrot=max(maxrot,o)
                rest=np.array([.12,side*.19,.02])
                if mode in ['dance','arms']:
                    b=(1-math.cos(phase))/2
                    rest=np.array([.12*(1-b)+.02*b,side*(.19*(1-b)+.365*b),.02*(1-b)+.16*b])
                if mode in ['transition','quad']:
                    # World-space paw path stays above the floor while the torso
                    # rotates. Rotating a rest offset first swept through the legs.
                    start=np.array([0,0,G['standing_base_height']])+rest
                    target=start*(1-blend)+np.array(G['quad_paw_targets'][j])*blend
                    target[1]+=side*.035*math.sin(math.pi*blend)
                else:target=base+R@rest
                e,o=solve_arm(tag,side,target,base,R);maxpos=max(maxpos,e)
                desired.extend([foot.tolist(),target.tolist()])
            mujoco.mj_forward(m,d)
            # Collision diagnostics cover only the explicitly exported primitives.
            for con in d.contact:
                if con.dist>=-.001:continue
                names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in con.geom]
                if 'floor' in names:floor_pen=min(floor_pen,float(con.dist));continue
                pair=' / '.join(sorted(names));collisions[pair]=min(collisions.get(pair,0),float(con.dist))
            frames.append({'base':base.tolist(),'quat':[math.cos(phi/2),0,math.sin(phi/2),0],
                           'q':d.qpos[qa].tolist(),'com':d.subtree_com[1].tolist(),'targets':desired})
        q=np.array([f['q'] for f in frames]);speed=np.max(np.abs(np.diff(q,axis=0)))*spec['fps'] if count else 0
        report={'max_site_position_error_m':maxpos,'max_rear_foot_orientation_error_rad':maxrot,
                'within_joint_limits':bool(np.all(q>=limits[:,0]-1e-9) and np.all(q<=limits[:,1]+1e-9)),
                'max_prescribed_joint_speed_rad_s':float(speed),'floor_penetration_below_1mm_m':floor_pen,
                'overlapping_collision_primitive_pairs_below_1mm':collisions,
                'frames':len(frames),'dynamic_balance':'not tested; base trajectory prescribed'}
        reports[mode]=report
        clips[mode]={'label':label,'period':period,'fps':spec['fps'],'frames':frames,'report':report}
        print(mode, 'pos',round(maxpos,6),'rot',round(maxrot,6),'speed',round(float(speed),2),'pairs',len(collisions),'floor',round(floor_pen,4),flush=True)
    dump(HERE/'motion_validation.json',reports)
    dump(HERE/'motions.json',clips)
    # Root-free model keyframes make the static endpoints inspectable in MuJoCo.
    import xml.etree.ElementTree as ET
    tree=ET.parse(HERE/'robot.xml');keys=ET.SubElement(tree.getroot(),'keyframe')
    for name in ['stand','quad']:
        f=clips[name]['frames'][0]
        ET.SubElement(keys,'key',name=name,qpos=' '.join(f'{v:.10g}' for v in f['base']+f['quat']+f['q']))
    ET.indent(tree,space='  ');(HERE/'robot.xml').write_text(ET.tostring(tree.getroot(),encoding='unicode')+'\n',encoding='utf8',newline='\n')
    (HERE/'model-data.js').write_text('window.DORO_MODEL='+json.dumps(model,ensure_ascii=False,separators=(',',':'))+';\nwindow.DORO_MOTIONS='+json.dumps(clips,ensure_ascii=False,separators=(',',':'))+';\n',encoding='utf8',newline='\n')
    return model,clips


if __name__=='__main__':generate()
