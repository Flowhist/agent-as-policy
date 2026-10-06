"""Generate vertical, short-side grasp orientations; never sends motion."""

import json
import argparse
from pathlib import Path
import subprocess

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--opening-axis',choices=('X','Y'),default='X',
                        help='Installed gripper closing axis; tool X verified by empty-jaw motion')
    args=parser.parse_args()
    folder=ROOT/'captures/grasp-orientation-plan'
    folder.mkdir(exist_ok=True)
    session=ROOT/'captures/calibration/session-01'
    camera=yaml.safe_load((session/'samples.yaml').read_text())['camera']
    extr=yaml.safe_load((session/'solve-15-validate-5/result.yaml').read_text())
    table=yaml.safe_load((session/'table-plane.yaml').read_text())
    logs=sorted((ROOT/'logs/runs').glob('configuration-check-*.jsonl'),key=lambda p:p.stat().st_mtime)
    records=[json.loads(x) for x in logs[-1].read_text().splitlines()]
    start=next(r['value'] for r in records if r['step']=='posture_endInRef_m_rad')
    image_path=args.image.resolve()
    im=cv2.imread(str(image_path))
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(0,100,70),(12,255,255)) | cv2.inRange(hsv,(170,100,70),(179,255,255))
    mask[:500]=0
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contour=max(contours,key=cv2.contourArea)
    rect=cv2.minAreaRect(contour)
    if cv2.contourArea(contour)<500:
        raise RuntimeError('Target not reliably detected')
    intr=rs.intrinsics()
    for key in ('width','height','fx','fy','ppx','ppy'):setattr(intr,key,camera[key])
    intr.model=rs.distortion.inverse_brown_conrady;intr.coeffs=camera['coefficients']
    bc=np.array(extr['T_base_camera']);normal=np.array(table['table_up_normal_base'])
    def deproject(pixel):
        ray=bc[:3,:3]@rs.rs2_deproject_pixel_to_point(intr,list(map(float,pixel)),1.)
        return bc[:3,3]+((.02961-table['plane_d_m']-normal@bc[:3,3])/(normal@ray))*ray
    corners=np.array([deproject(p) for p in cv2.boxPoints(rect)])
    edges=np.roll(corners,-1,axis=0)-corners
    long=edges[np.argmax(np.linalg.norm(edges,axis=1))]
    long-=normal*(normal@long);long/=np.linalg.norm(long)
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
            'approach_axis_base':z.tolist(),'candidates':candidates,
            'ik_summary':[{k:v for k,v in p.items() if k!='samples'} for p in checks['paths']],
            'limitations':['No full-arm or finger swept-volume collision check',
                           'Gripper opening axis must be rechecked if mounting changes',
                           'Target silhouette includes side faces; not a precise top-face fit',
                           'No descent, jaw opening, gripping depth or force plan executed']}
    (folder/'plan.yaml').write_text(yaml.safe_dump(report,sort_keys=False))
    print(yaml.safe_dump(report,sort_keys=False))


if __name__=='__main__':main()
