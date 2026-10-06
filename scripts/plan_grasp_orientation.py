"""Generate vertical, short-side grasp orientations; never sends motion."""

import json
import argparse
from pathlib import Path
import subprocess

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp
from block_top import TopFaceNotFound, detect_top, load_plane, save_result

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='New directory; existing plans are never overwritten')
    parser.add_argument('--vision-only',action='store_true',help='Save top-face detection without connecting to the robot')
    parser.add_argument('--roi',type=int,nargs=4,default=[0,500,1280,220])
    parser.add_argument('--opening-axis',choices=('X','Y'),default='X',
                        help='Installed gripper closing axis; tool X verified by empty-jaw motion')
    args=parser.parse_args()
    folder=args.output
    folder.mkdir(parents=True,exist_ok=False)
    session=ROOT/'captures/calibration/session-01'
    plane=load_plane(session)
    image_path=args.image.resolve()
    im=cv2.imread(str(image_path))
    if im is None:
        parser.error('Image could not be read')
    try:
        top=detect_top(im,plane,args.roi)
    except TopFaceNotFound as exc:
        save_result(im,{'status':'rejected','reason':str(exc),'motion_sent':False},folder)
        parser.error(str(exc))
    top.update(image=str(image_path),motion_sent=False)
    save_result(im,top,folder)
    if args.vision_only:
        print(json.dumps(top,indent=2))
        return
    logs=sorted((ROOT/'logs/runs').glob('configuration-check-*.jsonl'),key=lambda p:p.stat().st_mtime)
    records=[json.loads(x) for x in logs[-1].read_text().splitlines()]
    start=next(r['value'] for r in records if r['step']=='posture_endInRef_m_rad')
    normal=plane.normal
    long=np.array(top['long_axis_base'])
    short=np.cross(normal,long)
    z=-normal
    initial=Rotation.from_euler('xyz',start[3:])
    paths=[];candidates=[]
    for sign in (1,-1):
        if args.opening_axis=='X':
            x=short*sign;y=np.cross(z,x)
        else:
            y=short*sign;x=np.cross(y,z)
        rotation=np.column_stack([x,y,z])
        assert np.allclose(rotation.T@rotation,np.eye(3)) and np.linalg.det(rotation)>.999
        goal=Rotation.from_matrix(rotation)
        angle=float(np.degrees((initial.inv()*goal).magnitude()))
        matrices=[]
        count=max(2,int(np.ceil(angle/0.25))+1)
        interp=Slerp([0,1],Rotation.from_matrix([initial.as_matrix(),rotation]))
        for a in np.linspace(0,1,count):
            mat=np.eye(4);mat[:3,:3]=interp(a).as_matrix();mat[:3,3]=start[:3]
            matrices.append(mat.ravel().tolist())
        name='short_side_'+('positive' if sign==1 else 'negative')
        paths.append({'name':name,'tcp_matrices_row_major':matrices})
        for lift in (.05,.10,.15):
            route=[]
            high=np.array(start[:3])+normal*lift
            for a in np.linspace(0,1,int(lift/.002)+1):
                mat=np.eye(4);mat[:3,:3]=initial.as_matrix();mat[:3,3]=np.array(start[:3])+a*normal*lift
                route.append(mat.ravel().tolist())
            for a in np.linspace(0,1,count)[1:]:
                mat=np.eye(4);mat[:3,:3]=interp(a).as_matrix();mat[:3,3]=high
                route.append(mat.ravel().tolist())
            for a in np.linspace(0,1,int(lift/.002)+1)[1:]:
                mat=np.eye(4);mat[:3,:3]=rotation;mat[:3,3]=high-a*normal*lift
                route.append(mat.ravel().tolist())
            paths.append({'name':name+f'_lift_{int(lift*1000)}mm','tcp_matrices_row_major':route})
        candidates.append({'name':name,'rotation_matrix':rotation.tolist(),
                           'rpy_xyz_rad':goal.as_euler('xyz').tolist(),
                           'rotation_from_current_deg':angle,'samples':count})
    source=folder/'ik-input.json';source.write_text(json.dumps({'start_tcp_m_rad':start,'paths':paths},indent=2))
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/grasp_orientation_ik.py'),
                    str(source),str(folder/'ik-results.json')],check=True)
    checks=json.loads((folder/'ik-results.json').read_text())
    report={'status':'orientation_plan_only_not_collision_validated','motion_sent':False,
            'image':str(image_path),'robot_snapshot':str(logs[-1]),'start_tcp_m_rad':start,
            'tool_opening_axis':args.opening_axis,
            'tool_opening_axis_basis':'Tool X verified by opposing rubber-pad motion on 2026-10-06; recheck if remounted',
            'block_size_mm':[90.23,29.86,29.61], 'block_long_axis_base':long.tolist(),
            'top_face':top,'estimated_top_center_base_m':top['center_base_m'],
            'approach_axis_base':z.tolist(),'candidates':candidates,
            'ik_summary':[{k:v for k,v in p.items() if k!='samples'} for p in checks['paths']],
            'limitations':['No full-arm or finger swept-volume collision check',
                           'Gripper opening axis must be rechecked if mounting changes',
                           'Top face requires supported edges and known dimensions; see top-face.json',
                           'Orientation paths retain current TCP position; top center is not an executed translation',
                           'No descent, jaw opening, gripping depth or force plan executed']}
    (folder/'plan.yaml').write_text(yaml.safe_dump(report,sort_keys=False))
    print(yaml.safe_dump(report,sort_keys=False))


if __name__=='__main__':main()
