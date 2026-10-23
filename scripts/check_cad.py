from pathlib import Path
import json,math,itertools
import FreeCAD as A,Part
ROOT=Path(__file__).resolve().parents[1]
D=A.openDocument(str(ROOT/'cad/microquad-xl330-v1.FCStd'))
items=[(o.Label,o.Shape) for o in D.Objects if hasattr(o,'Shape')]
collisions=[]; tested=0
for (na,a),(nb,b) in itertools.combinations(items,2):
 ba,bb=a.BoundBox,b.BoundBox
 if any(min(getattr(ba,k+'Max'),getattr(bb,k+'Max'))-max(getattr(ba,k+'Min'),getattr(bb,k+'Min'))<0.01 for k in ['X','Y','Z']):continue
 tested+=1
 v=a.common(b).Volume
 if v>0.05:collisions.append({'a':na,'b':nb,'volume_mm3':round(v,4)})
r={'scope':'all home-pose solid pairs including module envelopes; positive volumes are unresolved unless explicitly explained',
   'object_count':len(items),'exact_common_tests':tested,'positive_intersections':collisions}
(ROOT/'reports/cad-intersections.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
print(json.dumps(r,indent=2),flush=True)

import sys
sys.exit(1 if collisions else 0)
