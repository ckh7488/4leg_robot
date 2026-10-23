"""Deterministic orthographic preview of the CAD tessellation (not a test)."""
from pathlib import Path
import json,math
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1]
P=json.loads((ROOT/'cad/preview-mesh.json').read_text())
font='DejaVuSans.ttf'
def F(n):
 try:return ImageFont.truetype(font,n)
 except OSError:return ImageFont.load_default(size=n)
im=Image.new('RGB',(1600,1120),'#edf3f7');d=ImageDraw.Draw(im)
d.text((50,24),'MICROQUAD  /  XL330-M288-T x12',font=F(34),fill='#213747')
d.text((50,70),'Prototype CAD - mm - shaft-based reference geometry',font=F(20),fill='#536878')
views=[('ASSEMBLY',.44,.8),('SIDE',0,math.pi/2),('FRONT',0,0),('TOP',math.pi/2,math.pi/2)]
for k,(title,elev,azim) in enumerate(views):
 x0=30+(k%2)*790;y0=110+(k//2)*480
 d.rounded_rectangle((x0,y0,x0+750,y0+455),radius=16,fill='#ffffff')
 d.text((x0+24,y0+15),title,font=F(22),fill='#213747')
 cam=[math.cos(azim)*math.cos(elev),math.sin(azim)*math.cos(elev),math.sin(elev)]
 right=[-math.sin(azim),math.cos(azim),0]
 up=[-math.cos(azim)*math.sin(elev),-math.sin(azim)*math.sin(elev),math.cos(elev)]
 def dot(a,b):return sum(x*y for x,y in zip(a,b))
 scale=1.65
 tris=[]
 for ob in P:
  if ob['group']=='envelope':continue
  co=ob['color'].lstrip('#');rgb=[int(co[i:i+2],16) for i in (0,2,4)]
  vv=[[v[0]+8,v[1],v[2]-65] for v in ob['v']]
  proj=[(x0+375+scale*dot(v,right),y0+245-scale*dot(v,up),dot(v,cam)) for v in vv]
  for t in ob['t']:
   tri=[proj[i] for i in t];a,b,c=[vv[i] for i in t]
   u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
   n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
   norm=math.sqrt(dot(n,n)) or 1
   light=.62+.38*abs(dot(n,[.3,-.4,.866])/norm)
   color=tuple(min(255,int(x*light)) for x in rgb)
   tris.append((sum(p[2] for p in tri)/3,[(p[0],p[1]) for p in tri],color))
 for depth,tri,color in sorted(tris,key=lambda x:x[0]):d.polygon(tri,fill=color)
 d.text((x0+24,y0+420),'Dark: body | teal: moving links | gold: hip carriers',font=F(16),fill='#536878')
d.text((50,1080),'Fit and load tests required. This preview does not demonstrate walking, measured response or fatigue strength.',font=F(19),fill='#536878')
im.save(ROOT/'drawings/assembly-review.png')
print(json.dumps({'render':'drawings/assembly-review.png','size':[1600,1120],'triangles':sum(len(o['t']) for o in P)}))
