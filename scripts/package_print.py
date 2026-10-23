"""Write standard geometry-only 3MFs and verify STL topology. Not Bambu project profiles."""
from pathlib import Path
import json,sys,zipfile,xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
import trimesh
M=json.loads((ROOT/'print/manifest.json').read_text());out=[];(ROOT/'print/3mf').mkdir(exist_ok=True)
NS='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
for p in M:
 mesh=trimesh.load_mesh(ROOT/'print/stl'/f"{p['part']}.stl",process=True)
 xml=E.Element('model',unit='millimeter',xmlns=NS)
 E.SubElement(xml,'metadata',name='Title').text=p['part']
 E.SubElement(xml,'metadata',name='Description').text='Geometry only. Select your actual printer/nozzle/material profile in Bambu Studio; not pre-sliced.'
 rs=E.SubElement(xml,'resources');obj=E.SubElement(rs,'object',id='1',type='model',name=p['part']);me=E.SubElement(obj,'mesh')
 vs=E.SubElement(me,'vertices')
 for v in mesh.vertices:E.SubElement(vs,'vertex',x=f'{v[0]:.7g}',y=f'{v[1]:.7g}',z=f'{v[2]:.7g}')
 ts=E.SubElement(me,'triangles')
 for t in mesh.faces:E.SubElement(ts,'triangle',v1=str(t[0]),v2=str(t[1]),v3=str(t[2]))
 E.SubElement(E.SubElement(xml,'build'),'item',objectid='1')
 path=ROOT/'print/3mf'/f"{p['part']}.3mf"
 with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
  z.writestr('[Content_Types].xml','<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
  z.writestr('_rels/.rels','<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
  z.writestr('3D/3dmodel.model',E.tostring(xml,encoding='utf-8',xml_declaration=True))
 with zipfile.ZipFile(path) as z:
  parsed=E.fromstring(z.read('3D/3dmodel.model'));unit_ok=parsed.attrib.get('unit')=='millimeter'
 row={'part':p['part'],'watertight':bool(mesh.is_watertight),'winding_consistent':bool(mesh.is_winding_consistent),
      'volume_positive':bool(mesh.volume>0),'bounds_mm':mesh.extents.tolist(),'fits_180mm':bool(max(mesh.extents)<180),
      'standard_3mf_unit_mm':unit_ok,'triangles':len(mesh.faces)}
 out.append(row)
r={'status':'STL topology and standard 3MF package check; Bambu Studio slicing not executed','parts':out,
'all_pass':all(p['watertight'] and p['winding_consistent'] and p['volume_positive'] and p['fits_180mm'] and p['standard_3mf_unit_mm'] for p in out)}
(ROOT/'reports/print-meshes.json').write_text(json.dumps(r,indent=2))
print(json.dumps({'part_types':len(out),'all_pass':r['all_pass']}))
if not r['all_pass']:sys.exit(1)
