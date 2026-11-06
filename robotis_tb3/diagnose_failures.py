"""Failure diagnosis for Part D: the numbers quoted in Sections 3.3, 4.4 and 4.5 of the write-up.

    python robotis_tb3/lqr/diagnose_failures.py <data_root>

<data_root> holds eval_sweep/, eval_cmp/ and eval_recovery/ (one folder per controller/condition/seed,
each with result.json and trace.json). Ground truth is read here for evaluation only. No simulator imports.

Prints three blocks:
  1. every failed plain-LQR sweep trial: share of frames at zero speed, achieved yaw rate by commanded
     yaw-rate band, and the heading error the controller was acting on while the robot was stationary
     (|turn command| / K_theta, so it does not depend on the logged evaluation fields).
  2. the baseline no-path stall in the shifted-obstacle condition: last accepted marker update, body turn
     against odometry turn between 2.7 s and 6.1 s, heading error and filter sigma at 8 s, marker
     detections while stalled, and what the surviving seeds did.
  3. baseline against baseline + recovery, per seed: outcome, no-path and search frames, marker updates
     during and after each no-path episode, and whether the run is identical to the baseline.
"""
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lqr.track_a_control_design import bryson_weights, lqr_discrete, V_NOM     # noqa: E402
from lqr.diagnose_no_path import episodes                                      # noqa: E402

DT = 0.1


def load(p):
    return json.loads(Path(p).read_text())


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def sweep_stalls(root):
    Q, R = bryson_weights()
    print('== 1. failed plain-LQR sweep trials (dead-band stall)')
    for tag, scale in (('lqr_r2', 2.0), ('lqr_r4', 4.0)):
        kth = lqr_discrete(V_NOM, Q, R * scale)[0][1]
        print(f'-- {tag}: K_theta = {kth:.3f}; command at the 0.25 rad exit threshold = {0.25 * kth:.3f} rad/s')
        implied_all = []
        for cond in ('nominal', 'camera_noise_dropout'):
            for seed in range(5):
                d = Path(root) / 'eval_sweep' / tag / cond / f'seed-{seed}'
                if not d.exists() or load(d / 'result.json')['success']:
                    continue
                t = load(d / 'trace.json')
                th = np.unwrap([x['truth'][2] for x in t])
                yaw = np.diff(th) / DT
                sp = np.array([x['speed_cmd'] for x in t])
                tc = np.array([x['turn_cmd'] for x in t])
                zero = sp[:-1] == 0
                bands = []
                for lo, hi in ((0.2, 0.45), (0.45, 0.6), (0.6, 0.7), (0.7, 0.81)):
                    m = zero & (np.abs(tc[:-1]) >= lo) & (np.abs(tc[:-1]) < hi)
                    if m.sum() > 20:
                        bands.append(f'{lo}-{hi}: n={m.sum()} median |yaw|={np.median(np.abs(yaw[m])):.3f}')
                for i, x in enumerate(t[:-1]):
                    if (x['status'].startswith('LIDAR PATH') and x['speed_cmd'] == 0 and 0.05 < abs(x['turn_cmd']) < 0.79
                            and abs(yaw[i]) < 0.01):
                        implied_all.append(abs(x['turn_cmd']) / kth)
                print(f'   {cond} seed {seed}: zero-speed {zero.mean():.2f}; final {t[-1]["status"][:30]}; ' + '; '.join(bands))
        if implied_all:
            v = np.array(implied_all)
            print(f'   stationary frames (achieved |yaw| < 0.01 rad/s): {len(v)}; implied |e_theta| median {np.median(v):.3f} '
                  f'(IQR {np.percentile(v, 25):.3f}-{np.percentile(v, 75):.3f}) against exit 0.25 and entry 0.40')


def baseline_stall(root):
    print('\n== 2. baseline no-path stall, shifted obstacle')
    for seed in range(5):
        d = Path(root) / 'eval_cmp' / 'heuristic' / 'shifted_obstacle' / f'seed-{seed}'
        if not d.exists():
            continue
        res, t = load(d / 'result.json'), load(d / 'trace.json')
        T = np.array([x['time_s'] for x in t])
        th = np.unwrap([x['truth'][2] for x in t])
        od = np.unwrap([x['odometry'][2] for x in t])
        es = np.unwrap([x['estimate'][2] for x in t])
        err = np.degrees(wrap(es - th))
        sig = np.degrees([np.sqrt(x['covariance'][2][2]) for x in t])
        i0, i1, i8 = (int(np.searchsorted(T, s)) for s in (2.7, 6.1, 8.0))
        acc = [x['time_s'] for x in t if x['camera_accepted']]
        stalled = [x for x in t if x['status'].startswith('NO SAFE')]
        later = sum(x['camera_accepted'] for x in t if x['time_s'] > 20)
        print(f'seed {seed}: success={res["success"]}; last accepted update before 3 s = '
              f'{max(a for a in acc if a < 3):.1f} s; body turn 2.7-6.1 s {np.degrees(th[i1] - th[i0]):.1f} deg '
              f'vs odometry {np.degrees(od[i1] - od[i0]):.1f} deg; heading error at 8 s {err[i8]:.1f} deg '
              f'(filter sigma {sig[i8]:.1f}); stalled frames {len(stalled)} '
              f'(marker detections {sum(x["camera_measurement"] is not None for x in stalled)}); '
              f'accepted updates after 20 s {later}; final heading error {err[-1]:.1f} deg')


def recovery(root):
    print('\n== 3. baseline vs baseline + recovery, shifted obstacle')
    for seed in range(5):
        a = Path(root) / 'eval_cmp' / 'heuristic' / 'shifted_obstacle' / f'seed-{seed}'
        b = Path(root) / 'eval_recovery' / 'heuristic_recovery' / 'shifted_obstacle' / f'seed-{seed}'
        if not (a.exists() and b.exists()):
            continue
        ra, rb = load(a / 'result.json'), load(b / 'result.json')
        ta, tb = load(a / 'trace.json'), load(b / 'trace.json')
        same = len(ta) == len(tb) and all(x['estimate'] == y['estimate'] and x['truth'] == y['truth'] for x, y in zip(ta, tb))
        print(f'seed {seed}: baseline success={ra["success"]} no-path frames={ra["tracking"]["no_path_frames"]}; '
              f'recovery success={rb["success"]} time={rb["simulation_seconds"]:.1f} s '
              f'no-path frames={rb["tracking"]["no_path_frames"]} search frames={rb["tracking"]["search_frames"]}; identical={same}')
        for e in episodes(tb):
            if e['search_frames']:
                print(f'    episode at {e["start_s"]:.1f} s: {e["frames"]} frames, {e["search_frames"]} search; marker updates during/after '
                      f'{e["accepted_during"]}/{e["accepted_after"]}; heading error {e["heading_err_start_deg"]:.1f} -> '
                      f'{e["heading_err_end_deg"]:.1f} deg; route returned={e["ended"]}')


if __name__ == '__main__':
    root = sys.argv[1] if len(sys.argv) > 1 else '.'
    sweep_stalls(root)
    baseline_stall(root)
    recovery(root)
