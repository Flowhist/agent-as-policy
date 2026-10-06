"""Plan tool-X short-side grasps after the observed empty-jaw motion test."""
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    folder = ROOT/'captures/grasp-orientation-plan'
    old = yaml.safe_load((folder/'plan.yaml').read_text())
    snapshot = max((ROOT/'logs/runs').glob('configuration-check-*.jsonl'),key=lambda p:p.stat().st_mtime)
    records = [json.loads(line) for line in snapshot.read_text().splitlines()]
    start = np.array(next(row['value'] for row in records if row['step']=='posture_endInRef_m_rad'))
    r0 = Rotation.from_euler('xyz',start[3:])
    z = np.array(old['approach_axis_base'])
    short = np.cross(-z,np.array(old['block_long_axis_base']))
    position = np.array(old['start_tcp_m_rad'][:3])
    paths = []
    for angle in (-90,90):
        x = short * (-1 if angle == 90 else 1)
        goal = np.column_stack([x,np.cross(z,x),z])
        interpolation = Slerp([0,1],Rotation.from_matrix([r0.as_matrix(),goal]))
        matrices = []
        for t in np.linspace(0,1,201):
            mat = np.eye(4)
            mat[:3,:3] = interpolation(t).as_matrix()
            mat[:3,3] = start[:3]+t*(position-start[:3])
            matrices.append(mat.ravel().tolist())
        paths.append({'name':f'tool_x_short_side_{angle}','tcp_matrices_row_major':matrices})
    source = folder/'corrected-ik-input.json'
    source.write_text(json.dumps({'start_tcp_m_rad':start.tolist(),'paths':paths},indent=2))
    result = folder/'corrected-ik-results.json'
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/grasp_orientation_ik.py'),str(source),str(result)],check=True)
    checked = json.loads(result.read_text())
    routes = []
    for path,check in zip(paths,checked['paths']):
        if not check['complete']:
            continue
        rows = []
        for matrix,row in zip(path['tcp_matrices_row_major'],check['samples']):
            mat = np.array(matrix).reshape(4,4)
            rows.append({'joints_deg':row['joints_deg'],'tcp_m_rad':np.r_[mat[:3,3],Rotation.from_matrix(mat[:3,:3]).as_euler('xyz')].tolist()})
        routes.append({'name':path['name'],'samples':rows,'min_abs_j5_deg':check['min_abs_j5_deg'],
                       'max_joint_travel_deg':max(abs(a-b) for a,b in zip(rows[0]['joints_deg'],rows[-1]['joints_deg']))})
    routes.sort(key=lambda row:row['max_joint_travel_deg'])
    if not routes:
        raise RuntimeError('No corrected route passed IK')
    selected = routes[0]
    cuts = [0]
    while cuts[-1] < len(selected['samples'])-1:
        end = cuts[-1]+1
        while end+1 < len(selected['samples']) and max(abs(a-b) for a,b in zip(selected['samples'][cuts[-1]]['joints_deg'],selected['samples'][end+1]['joints_deg'])) < 4.8:
            end += 1
        cuts.append(end)
    selected['segment_indices'] = cuts
    (folder/'corrected-joint-route.json').write_text(json.dumps({'motion_sent':False,'opening_axis':'tool X, observed opposing jaw motion','routes':[selected]},indent=2))
    print(json.dumps({k:v for k,v in selected.items() if k!='samples'},indent=2))


if __name__ == '__main__':
    main()
