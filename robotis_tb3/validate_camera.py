"""Offline simulation calibration and marker validation; never controller feedback.

Fit intrinsics from rendered checkerboards, check held-out views, then evaluate
the unchanged Part B detector against MuJoCo camera/base transforms. All fixtures,
misses, noise settings and exact source hashes are saved for reproducibility.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from track_a_sensors import (build_room, CalibratedMarkerCamera, evaluation_pose,
                             WIDTH, HEIGHT, FOV)


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def reprojection_rmse(projected, observed):
    """Match corners, avoiding OpenCV 4/5 N-by-1-by-2 versus N-by-2 broadcasting."""
    projected = np.asarray(projected).reshape(-1, 2)
    observed = np.asarray(observed).reshape(-1, 2)
    if projected.shape != observed.shape or len(projected) == 0:
        raise ValueError('Matching nonempty corner arrays are required')
    return float(np.sqrt(np.mean(np.sum((projected-observed)**2, axis=1))))


def calibrate(output, rng):
    square = 80
    board = np.zeros((7 * square, 9 * square), dtype=np.uint8)
    for y in range(7):
        for x in range(9):
            board[y*square:(y+1)*square, x*square:(x+1)*square] = 255 * ((x+y) % 2)
    texture = output / 'checkerboard.png'
    cv2.imwrite(str(texture), board)
    xml = f'''<mujoco><visual><global offwidth="{WIDTH}" offheight="{HEIGHT}"/></visual>
      <asset><texture name="board" type="2d" file="{texture.resolve()}"/>
      <material name="board" texture="board" texrepeat="1 1" texuniform="false" emission="1"/></asset>
      <worldbody><camera name="test" pos="0 0 0" xyaxes="1 0 0 0 -1 0" fovy="{FOV}"/>
      <body mocap="true"><geom type="plane" size=".18 .14 .001" material="board"/></body>
      </worldbody></mujoco>'''
    (output / 'calibration-scene.xml').write_text(xml.replace(str(texture.resolve()), 'checkerboard.png'))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    obj = np.zeros((8*6, 3), np.float32)
    obj[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * .04
    views, fixtures = [], []
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        for index in range(24):
            pos = rng.uniform([-.12, -.09, .55], [.12, .09, .95])
            angles = rng.uniform([-.4, -.4, -.3], [.4, .4, .3])
            quat = Rotation.from_euler('xyz', angles + [np.pi, 0, 0]).as_quat()
            data.mocap_pos[0] = pos
            data.mocap_quat[0] = quat[[3, 0, 1, 2]]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera='test')
            rgb = renderer.render()
            ok, corners = cv2.findChessboardCornersSB(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), (8, 6))
            name = f'checker-view-{index:02}.png'
            cv2.imwrite(str(output/name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            fixtures.append({'image': name, 'position': pos.tolist(),
                             'quaternion_wxyz': data.mocap_quat[0].tolist(), 'detected': bool(ok)})
            if ok:
                views.append({'index': index, 'corners': corners})
    if len(views) < 16:
        raise RuntimeError(f'Only {len(views)} checkerboards detected; evidence retained')
    train, held = views[:len(views)-6], views[-6:]
    flags = (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 |
             cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6)
    rms, fitted, distortion, _, _ = cv2.calibrateCamera(
        [obj for _ in train], [v['corners'] for v in train], (WIDTH, HEIGHT), None, None, flags=flags)
    focal = HEIGHT / (2 * np.tan(np.deg2rad(FOV)/2))
    configured = np.array([[focal, 0, (WIDTH-1)/2], [0, focal, (HEIGHT-1)/2], [0, 0, 1.]])
    held_errors = []
    for view in held:
        ok, rvec, tvec = cv2.solvePnP(obj, view['corners'], fitted, distortion)
        if not ok:
            raise RuntimeError('Held-out checkerboard pose fit failed')
        projected, _ = cv2.projectPoints(obj, rvec, tvec, fitted, distortion)
        held_errors.append(reprojection_rmse(projected, view['corners']))
    (output/'checkerboard-fixtures.json').write_text(json.dumps(fixtures, indent=2)+'\n')
    result = {'attempted_views': 24, 'detected_views': len(views),
              'training_indices': [v['index'] for v in train], 'held_out_indices': [v['index'] for v in held],
              'square_size_m': .04, 'configured_K': configured.tolist(), 'fitted_K': fitted.tolist(),
              'training_reprojection_rms_px': float(rms),
              'held_out_reprojection_rms_px': held_errors,
              'focal_relative_error_percent': (100*np.abs(np.diag(fitted)[:2]/focal-1)).tolist(),
              'principal_point_error_px': (fitted[:2,2]-configured[:2,2]).tolist(),
              'distortion_protocol': 'zero distortion fixed to match the simulation; not estimated'}
    if max(result['focal_relative_error_percent']) > 5:
        raise RuntimeError('Recovered focal length disagrees by over 5%; investigate calibration fixture')
    return result


def validate_marker(output, rng):
    model = build_room()
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    initial = data.qpos.copy()
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'navigation')
    marker = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'marker')
    detector = CalibratedMarkerCamera()
    rows = []
    options = mujoco.MjvOption()
    options.geomgroup[:] = 1
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        for x in (.2, .7, 1.4, 1.9, 2.35):
            for y in (-.6, 0., .6):
                for offset in (-20, 0, 20, 90):
                    yaw = np.arctan2(-y, 3-x) + np.deg2rad(offset)
                    data.qpos[:] = initial
                    data.qpos[:2] = [x, y]
                    data.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
                    data.qvel[:] = 0
                    mujoco.mj_forward(model, data)
                    truth = evaluation_pose(model, data)
                    delta = data.geom_xpos[marker, :2] - data.cam_xpos[camera, :2]
                    expected = np.array([np.linalg.norm(delta),
                                         wrap(np.arctan2(delta[1], delta[0])-truth[2]), truth[2]])
                    renderer.update_scene(data, camera='navigation', scene_option=options)
                    clean = renderer.render()
                    for noise in (0, 5, 15):
                        for repetition in range(2):
                            rgb = (np.clip(clean.astype(float)+rng.normal(0, noise, clean.shape), 0, 255).astype(np.uint8)
                                   if noise else clean)
                            observed, _, _ = detector.observe(rgb)
                            row = {'x_m': x, 'y_m': y, 'heading_offset_deg': offset,
                                   'pixel_noise_sd': noise, 'repetition': repetition,
                                   'range_truth_m': float(expected[0]), 'detected': observed is not None,
                                   'range_error_m': None, 'bearing_error_deg': None, 'yaw_error_deg': None}
                            if observed is not None:
                                error = observed-expected
                                row.update(range_error_m=float(error[0]),
                                           bearing_error_deg=float(np.rad2deg(wrap(error[1]))),
                                           yaw_error_deg=float(np.rad2deg(wrap(error[2]))))
                            rows.append(row)
    with (output/'marker-observations.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    summary = []
    for noise in (0, 5, 15):
        eligible = [r for r in rows if r['pixel_noise_sd']==noise and r['heading_offset_deg']!=90]
        hits = [r for r in eligible if r['detected']]
        errors = np.array([[r[k] for k in ('range_error_m','bearing_error_deg','yaw_error_deg')] for r in hits])
        negative = [r for r in rows if r['pixel_noise_sd']==noise and r['heading_offset_deg']==90]
        summary.append({'pixel_noise_sd': noise, 'attempts_in_view_group': len(eligible),
                        'detections': len(hits), 'rmse_range_m_bearing_deg_yaw_deg': np.sqrt(np.mean(errors**2,axis=0)).tolist(),
                        'bias_range_m_bearing_deg_yaw_deg': np.mean(errors,axis=0).tolist(),
                        'residual_covariance_m_deg_deg': np.cov(errors.T).tolist(),
                        'out_of_view_attempts': len(negative),
                        'out_of_view_detections': sum(r['detected'] for r in negative)})
    return rows, summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seed', type=int, default=11)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    calibration = calibrate(args.output, rng)
    rows, marker = validate_marker(args.output, rng)
    sources = [Path(__file__), Path(__file__).with_name('track_a_sensors.py'),
               Path(__file__).with_name('turtlebot3_waffle_pi.xml')]
    result = {'seed': args.seed, 'calibration': calibration, 'marker_validation': marker,
              'scope': 'Offline rendered fixtures, not navigation trials or physical-camera calibration. Fitted K is evaluated, not substituted into the controller.',
              'source_sha256': {s.name: hashlib.sha256(s.read_bytes()).hexdigest() for s in sources}}
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout='constrained')
    for ax, key, label in zip(axes, ('range_error_m','bearing_error_deg','yaw_error_deg'), ('Range error (m)','Bearing error (deg)','Heading error (deg)')):
        for noise in (0, 5, 15):
            hits = [r for r in rows if r['pixel_noise_sd']==noise and r['detected'] and r['heading_offset_deg']!=90]
            ax.scatter([r['range_truth_m'] for r in hits], [r[key] for r in hits], s=13, alpha=.55, label=f'pixel SD {noise}')
        ax.axhline(0, color='gray', lw=.7)
        ax.set(xlabel='True camera range (m)', ylabel=label)
        ax.legend(fontsize=8)
    fig.suptitle('Project 11: independent pose-fixture camera validation (detected frames)')
    fig.savefig(args.output/'camera-validation.png', dpi=150)
    plt.close(fig)
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
