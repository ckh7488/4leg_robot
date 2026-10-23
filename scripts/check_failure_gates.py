"""Inject a fixed overlapping CAD obstacle in an isolated copy; require both checkers to reject it."""
from pathlib import Path
import argparse, json, shutil, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freecad-python',required=True)
    args=parser.parse_args()
    work=(ROOT/'_runs/failure-gates').resolve()
    if not work.is_relative_to((ROOT/'_runs').resolve()):raise ValueError('Fixture must stay under _runs')
    for name in ['scripts','cad','reports']:(work/name).mkdir(parents=True,exist_ok=True)
    for name in ['scripts/check_cad.py','scripts/check_motion.py','design.json',
                 'cad/home-kinematics.json','cad/assembly-instances.json','cad/microquad-xl330-v1.FCStd']:
        shutil.copyfile(ROOT/name,work/name)
    for name in ['cad-intersections.json','motion-FL.json']:
        p=work/'reports'/name
        if p.exists():p.unlink()
    injector=work/'inject_collision.py'
    injector.write_text('from pathlib import Path\nimport FreeCAD as A\n'
        "root=Path(__file__).resolve().parent\nd=A.openDocument(str(root/'cad/microquad-xl330-v1.FCStd'))\n"
        "s=next(o.Shape for o in d.Objects if o.Label=='FL_calf_front')\n"
        "o=d.addObject('PartDesign::Feature','IntentionalObstacle');o.Label='intentional_fixed_obstacle';o.Shape=s.copy()\n"
        'd.recompute();d.save()\n',encoding='utf-8')
    outcomes=[]
    for name,extra in [('inject_collision.py',[]),('scripts/check_cad.py',[]),('scripts/check_motion.py',['--leg','FL'])]:
        result=subprocess.run([args.freecad_python,str(work/name),*extra],cwd=work,capture_output=True,text=True,encoding='utf-8',errors='replace')
        (work/(Path(name).stem+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        outcomes.append({'script':name,'exit_code':result.returncode})
    valid_reports=(work/'reports/cad-intersections.json').exists() and (work/'reports/motion-FL.json').exists()
    intersections=[];failed_poses=0
    if valid_reports:
        intersections=json.loads((work/'reports/cad-intersections.json').read_text(encoding='utf-8'))['positive_intersections']
        failed_poses=json.loads((work/'reports/motion-FL.json').read_text(encoding='utf-8'))['failed_pose_count']
    intended=any({row['a'],row['b']}=={'FL_calf_front','intentional_fixed_obstacle'} and row['volume_mm3']>0.05 for row in intersections)
    passed=[r['exit_code'] for r in outcomes]==[0,1,1] and intended and failed_poses>0
    report={'scope':'Isolated intentional collision; original CAD unchanged',
            'expected_exit_codes':[0,1,1],'observed':outcomes,'intended_collision_detected':intended,
            'motion_failed_pose_count':failed_poses,'pass':passed}
    (ROOT/'reports/failure-gate-regression.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report));return 0 if passed else 1
if __name__=='__main__':sys.exit(main())
