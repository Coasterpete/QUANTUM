"""Generate QUANTUM's metre-scale, +X forward test components (not final assets)."""
import json, struct
from pathlib import Path

def box(length=1, width=.2, height=.12, centered=False):
    positions=[];normals=[];uv=[];indices=[]
    for axis in range(3):
        for sign in [-1,1]:
            a=(axis+1)%3;b=(axis+2)%3;first=len(positions)
            for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]:
                p=[0.,0.,0.];p[axis]=sign*[length,width,height][axis]/2
                p[a]=u*[length,width,height][a]/2;p[b]=v*sign*[length,width,height][b]/2
                if not centered:p[0]+=length/2
                n=[0.,0.,0.];n[axis]=sign
                # Blender Z-up -> glTF Y-up: (x,y,z) -> (x,z,-y).
                positions.append((p[0],p[2],-p[1]));normals.append((n[0],n[2],-n[1]));uv.append(((u+1)/2,(v+1)/2))
            indices.extend([first,first+1,first+2,first,first+2,first+3])
    return positions,normals,uv,indices

def write(path, centered=False, hierarchy=False):
    p,n,uv,ix=box(.18 if centered else 1,1.15 if centered else .2,.12,centered)
    buffers=[b''.join(struct.pack('<3f',*v) for v in p),b''.join(struct.pack('<3f',*v) for v in n),b''.join(struct.pack('<2f',*v) for v in uv),b''.join(struct.pack('<I',v) for v in ix)]
    views=[];binary=b''
    for data in buffers:
        views.append(dict(buffer=0,byteOffset=len(binary),byteLength=len(data)));binary+=data
    doc=dict(asset=dict(version='2.0',generator='QUANTUM procedural pass-4 fixture'),buffers=[dict(byteLength=len(binary))],bufferViews=views,
        accessors=[dict(bufferView=i,componentType=5126 if i<3 else 5125,count=len(p) if i<3 else len(ix),type='VEC3' if i<2 else 'VEC2' if i==2 else 'SCALAR') for i in range(4)],
        materials=[dict(pbrMetallicRoughness=dict(baseColorFactor=[.05,.5,.2,1],metallicFactor=.4,roughnessFactor=.35))],
        meshes=[dict(primitives=[dict(attributes=dict(POSITION=0,NORMAL=1,TEXCOORD_0=2),indices=3,material=0)])],
        nodes=[dict(mesh=0)],scenes=[dict(nodes=[0])],scene=0)
    if hierarchy:
        doc['meshes'].append(doc['meshes'][0]);doc['nodes']=[dict(translation=[2,3,4],children=[1,2]),dict(mesh=0,scale=[2,1,.5]),dict(mesh=1,translation=[2,0,0],scale=[-1,1,1])]
    js=json.dumps(doc,separators=(',',':')).encode();js+=b' '*((-len(js))%4)
    result=struct.pack('<III',0x46546c67,2,12+8+len(js)+8+len(binary))+struct.pack('<II',len(js),0x4e4f534a)+js+struct.pack('<II',len(binary),0x004e4942)+binary
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(result)
root=Path(__file__).resolve().parents[2]
write(root/'assets/track/components/example-spine.glb')
write(root/'assets/track/components/example-tie.glb',centered=True)
write(root/'tests/fixtures/track-families/hierarchy.glb',hierarchy=True)
