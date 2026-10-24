"""Two-rod ankle geometry. This is a closed-chain screen, not a fabricated design."""
import math
import copy
import numpy as np


def rotation(p,r):
    cp,sp,cr,sr=math.cos(p),math.sin(p),math.cos(r),math.sin(r)
    Y=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    X=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Y@X,Y


def solve(config,radius,pitch,roll):
    P=config['ankle_linkage'];A=np.array(P['crank_axis_centers_m']);B0=np.array(P['foot_attachments_m'])
    L=np.linalg.norm(A+np.array([radius,0,0])-B0,axis=1)
    R,Y=rotation(pitch,roll);B=B0@R.T;d=B-A
    rho=np.hypot(d[:,0],d[:,2]);c=(np.sum(d*d,axis=1)+radius*radius-L*L)/(2*radius*rho)
    if np.any(abs(c)>1):return {'reachable':False}
    theta=np.arctan2(d[:,2],d[:,0])+np.arccos(np.clip(c,-1,1))
    E=A+radius*np.column_stack([np.cos(theta),np.zeros(2),np.sin(theta)])
    residual=np.linalg.norm(E-B,axis=1)-L
    assert max(abs(residual))<1e-10
    tangent=radius*np.column_stack([-np.sin(theta),np.zeros(2),np.cos(theta)])
    v=E-B
    denominator=np.sum(v*tangent,axis=1)
    if min(abs(denominator))<1e-10:return {'reachable':True,'singular':True}
    # Differentiate fixed rod length; H maps foot angular rates to crank rates.
    bp=np.cross(np.array([0,1,0]),B)
    br=np.cross(Y@np.array([1,0,0]),B)
    H=np.column_stack([np.sum(v*bp,axis=1)/denominator,np.sum(v*br,axis=1)/denominator])
    joint_torque=np.array(P['example_joint_torque_nm'])
    motor_torque=np.linalg.solve(H.T,joint_torque)
    force=motor_torque/(denominator/L)
    produced_moment=np.sum(np.cross(B,force[:,None]*v/L[:,None]),axis=0)
    produced=np.array([produced_moment[1],produced_moment@(Y@np.array([1,0,0]))])
    assert np.max(abs(produced-joint_torque))<1e-8
    gaps=[];n=P['clearance_sampling_intervals'];motor=config['motor_references']['large']
    for start,end in zip(E,B):
        points=start+(end-start)*np.linspace(0,1,n+1)[:,None]
        spacing=np.linalg.norm(end-start)/n
        for center in P['motor_housing_centers_m']:
            v=points-np.array(center)
            radial=np.hypot(v[:,0],v[:,2])-motor['diameter_mm']/2000
            axial=abs(v[:,1])-motor['length_mm']/2000
            q=np.column_stack([radial,axial])
            sdf=np.minimum(np.max(q,axis=1),0)+np.linalg.norm(np.maximum(q,0),axis=1)
            # Distance fields are 1-Lipschitz. Subtract half the sample spacing
            # and rod radius for a conservative bound over the whole segment.
            gaps.append(float(min(sdf)-spacing/2-P['rod_radius_m']))
    return {'reachable':True,'singular':False,'motor_angles_rad':theta,'foot_points_m':B,'crank_points_m':E,
            'rod_lengths_m':L,'constraint_residual_max_m':float(max(abs(residual))),
            'rate_mapping':H,'condition_number':float(np.linalg.cond(H)),
            'example_motor_torques_nm':motor_torque,'example_rod_axial_forces_n':force,
            'rod_to_motor_clearance_lower_bound_m':min(gaps)}


def evaluate(config):
    P=config['ankle_linkage'];all_cases=[];summary=[];fd_errors=[]
    for radius in P['crank_radii_m']:
        cases=[]
        for pd in P['pitch_samples_deg']:
            for rd in P['roll_samples_deg']:
                p,r=map(math.radians,[pd,rd]);row=solve(config,radius,p,r)
                row.update(radius_m=radius,pitch_deg=pd,roll_deg=rd)
                if row['reachable'] and not row.get('singular',False):
                    eps=1e-6
                    neighbors=[solve(config,radius,p+eps,r),solve(config,radius,p-eps,r),solve(config,radius,p,r+eps),solve(config,radius,p,r-eps)]
                    if all(x['reachable'] and not x.get('singular',False) for x in neighbors):
                        a,b,c,d=[x['motor_angles_rad'] for x in neighbors]
                        numeric=np.column_stack([(a-b)/(2*eps),(c-d)/(2*eps)])
                        error=float(np.max(abs(numeric-row['rate_mapping'])))
                        assert error<1e-5
                        fd_errors.append(error)
                        row['mapping_finite_difference_error']=error
                cases.append(row)
        usable=[x for x in cases if x['reachable'] and not x.get('singular',False)]
        summary.append({'radius_m':radius,'sample_count':len(cases),'reachable_count':sum(x['reachable'] for x in cases),
                        'nonsingular_reachable_count':len(usable),
                        'rod_housing_clear_count':sum(x['rod_to_motor_clearance_lower_bound_m']>0 for x in usable),
                        'max_abs_example_motor_torque_nm':max(float(max(abs(x['example_motor_torques_nm']))) for x in usable),
                        'max_abs_example_rod_force_n':max(float(max(abs(x['example_rod_axial_forces_n']))) for x in usable),
                        'max_condition_number':max(x['condition_number'] for x in usable)})
        all_cases.extend(cases)
    neutral=solve(config,P['display_radius_m'],0,0)
    variants=[]
    for half in P['housing_half_spacing_comparisons_m']:
        c=copy.deepcopy(config)
        c['ankle_linkage']['motor_housing_centers_m']=[[v[0],(-1 if v[1]<0 else 1)*half,v[2]] for v in P['motor_housing_centers_m']]
        rows=[solve(c,P['display_radius_m'],math.radians(p),math.radians(r)) for p in P['pitch_samples_deg'] for r in P['roll_samples_deg']]
        valid=[r for r in rows if r['reachable'] and not r.get('singular',False)]
        variants.append({'motor_half_spacing_m':half,'sample_count':len(rows),
                         'rod_housing_clear_count':sum(r['rod_to_motor_clearance_lower_bound_m']>0 for r in valid),
                         'minimum_clearance_lower_bound_m':min(r['rod_to_motor_clearance_lower_bound_m'] for r in valid)})
    return {'scope':P['scope'],'summary':summary,'neutral':neutral,'cases':all_cases,
            'housing_variants':variants,
            'max_mapping_finite_difference_error':max(fd_errors),
            'example_joint_torque_nm':P['example_joint_torque_nm'],
            'clearance_bound_scope':'Only rod capsules versus two motor cylinders, using a signed-distance Lipschitz lower bound; not all-pairs mechanical clearance.'}
