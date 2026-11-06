"""Part D evaluation: controller x condition x seed grid. Resumable.

A controller spec is  heuristic | lqr  with optional modifiers, in this order:

  lqr:<r_scale>      R multiplied by r_scale (larger = gentler turning)   e.g. lqr:0.5
  lqr@<rotate>       rotate-in-place threshold [rad] (default 0.40)        e.g. lqr@0.6
  <spec>+recovery    rotate to search for the marker when the planner has no route for 1 s
                     e.g. heuristic+recovery, lqr:2@0.6+recovery
  <spec>+comp        actuator compensation (friction offset, measured gains, yaw-priority speed limit)
                     on path following and search; docking keeps the original mapping.  e.g. lqr+comp
  <spec>+compall     the same, docking included.  Always the LAST modifier: lqr+recovery+comp

The same spec list covers every study:

  comparison  python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic lqr --trials 10 --output eval_cmp
  R sweep     python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr:0.25 lqr:0.5 lqr lqr:2 lqr:4 \\
                  --conditions nominal camera_noise_dropout --trials 10 --output eval_sweep
  rotate      python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr@0.6 lqr@0.8 \\
                  --conditions nominal slippery_floor --trials 10 --output eval_rotate
  recovery    python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic heuristic+recovery \\
                  --conditions shifted_obstacle --trials 10 --output eval_recovery
  compensate  python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr+comp lqr+compall \\
                  --conditions nominal shifted_obstacle --trials 10 --output eval_comp

Layout: <output>/<tag>/<condition>/seed-<n>/result.json (+ trace). summary.json is rewritten
after every trial; a trial whose result.json already exists is loaded, not re-run, so an
interrupted batch continues where it stopped. Failures stay in the summary.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/ (shared modules, `lqr` package)
import json
import hashlib
import importlib.metadata
import re
from track_a_project import parser, run

# Same conditions as evaluate_track_a.py so results are comparable with the baseline.
CONDITIONS = {
    'nominal': {},
    'camera_noise_dropout': {'pixel_noise': 5, 'dropout': .30},
    'shifted_obstacle': {'obstacle_shift': .15, 'encoder_bias': .004},
    'slippery_floor': {'friction': .6},
}


SPEC = re.compile(r'^(heuristic|lqr)(?::([0-9]*\.?[0-9]+))?(?:@([0-9]*\.?[0-9]+))?(\+recovery)?$')


def parse_spec(spec):
    """'lqr:0.5@0.6+recovery' -> ('lqr', 0.5, 'lqr_r0.5_rot0.6_recovery', 0.6, True).

    Returns (name, r_scale, tag, rotate_enter, recovery). Tags for the older specs are
    unchanged ('lqr', 'lqr_r0.25', 'heuristic'), so cached trials are still found."""
    m = SPEC.match(spec)
    if not m:
        raise ValueError(f'bad controller spec {spec!r}')
    name, scale, rotate, recovery = m.group(1), m.group(2), m.group(3), bool(m.group(4))
    if name == 'heuristic' and (scale or rotate):
        raise ValueError('heuristic has no R scale or rotate threshold')
    r = float(scale) if scale else 1.0
    rot = float(rotate) if rotate else 0.40
    if r <= 0:
        raise ValueError('R scale must be positive')
    if not 0.1 <= rot <= 1.2:
        raise ValueError('rotate threshold must be between 0.1 and 1.2 rad')
    tag = name + (f'_r{scale}' if scale else '') + (f'_rot{rotate}' if rotate else '') + ('_recovery' if recovery else '')
    return name, r, tag, rot, recovery


COMP = re.compile(r'\+comp(all)?$')


def split_comp(spec):
    """'lqr+recovery+comp' -> ('lqr+recovery', 'path');  '...+compall' -> (..., 'all');  else (spec, 'off').

    Kept apart from parse_spec so the tags and cached trials of every earlier spec are unchanged."""
    m = COMP.search(spec)
    if not m:
        return spec, 'off'
    return spec[:m.start()], ('all' if m.group(1) else 'path')


def parse_full_spec(spec):
    """(name, r_scale, tag, rotate_enter, recovery, compensate); the tag gains _comp / _compall."""
    base, comp = split_comp(spec)
    name, r, tag, rot, recovery = parse_spec(base)
    return name, r, tag + {'off': '', 'path': '_comp', 'all': '_compall'}[comp], rot, recovery, comp


def source_fingerprint():
    """Invalidate resumed trials when the runtime, model or sensor image changes."""
    here = Path(__file__).resolve().parents[1]
    names = ['track_a_project.py', 'track_a_sensors.py', 'track_a_estimation.py',
             'track_a_navigation.py', 'actuator_compensation.py', 'turtlebot_model.py',
             'camera_navigation.py', 'turtlebot3_waffle_pi.xml',
             'assets/navigation_marker.png', 'lqr/track_a_control.py',
             'lqr/track_a_control_design.py', 'lqr/evaluate_controllers.py']
    sources = {name: hashlib.sha256((here / name).read_bytes()).hexdigest()
               for name in names}
    versions = {name: importlib.metadata.version(name) for name in
                ('numpy', 'mujoco', 'opencv-python', 'machinevision-toolbox-python', 'scipy')}
    return {'sources': sources, 'versions': versions}


def cache_signature(trial, fingerprint):
    settings = {key: value for key, value in vars(trial).items() if key != 'output'}
    return hashlib.sha256(json.dumps({'settings': settings, 'runtime': fingerprint},
                                     sort_keys=True).encode()).hexdigest()


def main():
    p = parser()
    p.add_argument('--trials', type=int, default=10, help='Seeds per controller and condition')
    p.add_argument('--controllers', nargs='+', default=['heuristic', 'lqr'])
    p.add_argument('--conditions', nargs='+', default=list(CONDITIONS), choices=list(CONDITIONS))
    args = p.parse_args()
    specs = [parse_full_spec(s) for s in args.controllers]
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=True)
    summary_path = root / 'summary.json'
    results = []
    fingerprint = source_fingerprint()
    for name, r_scale, tag, rotate, recovery, comp in specs:
        for condition in args.conditions:
            for seed in range(args.trials):
                out = root / tag / condition / f'seed-{seed}'
                done = out / 'result.json'
                trial = parser().parse_args([])
                for key in vars(trial):
                    setattr(trial, key, getattr(args, key))
                trial.headless = True
                trial.seed = seed
                trial.output = str(out)
                trial.controller = name
                trial.lqr_r_scale = r_scale
                trial.lqr_rotate_enter = rotate
                trial.recovery = recovery
                trial.compensate = comp
                for key, value in CONDITIONS[condition].items():
                    setattr(trial, key, value)
                signature = cache_signature(trial, fingerprint)
                if done.exists():
                    result = json.loads(done.read_text())
                    if result.get('evaluation_signature') != signature:
                        raise ValueError(f'Cached trial settings/source differ or lack provenance: {done}. '
                                         'Use a new --output directory; existing evidence is preserved.')
                else:
                    result = run(trial)
                    result['evaluation_signature'] = signature
                    # Store provenance even when a test/simulation runner only returns a result.
                    done.parent.mkdir(parents=True, exist_ok=True)
                    done.write_text(json.dumps(result, indent=2) + '\n')
                result.update(condition=condition, controller_tag=tag, controller_name=name,
                              lqr_r_scale=r_scale, lqr_rotate_enter=rotate, recovery=recovery, compensate=comp, seed=seed)
                results.append(result)
                summary_path.write_text(json.dumps(results, indent=2) + '\n')
    n_ok = sum(r['success'] for r in results)
    print(f'Successes {n_ok}/{len(results)}  ->  {summary_path}')


if __name__ == '__main__':
    main()
