"""
Run the real user program against the mock world and the REAL judge module.

usage: python3 run_mock.py --level veryhard --seed 1 [--no-dist] [-v]
       python3 run_mock.py --batch 30 --level veryhard
"""
import argparse
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import warnings

sys.dont_write_bytecode = True
warnings.filterwarnings('ignore', category=SyntaxWarning)
HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.environ.get('KRPC_SIM', os.path.expanduser('~/krpc/krpc7th-simulator'))
USER = os.environ.get('KRPC_USER', os.path.join(HERE, '..', 'solution', 'user_template.py'))

sys.path.insert(0, HERE)
import mockros  # noqa: E402
mockros.install_fake_modules()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def gen_cheatsheet(level, seed, outdir):
    gen_dir = os.path.join(SIM, 'rootfs/krpc/random_env_generator')
    code = ('import random,sys; random.seed(%d); sys.argv=["g","-l","%s","-o","%s"]; '
            'import runpy; runpy.run_path("generate_objects.py", run_name="__main__")' % (seed, level, outdir))
    subprocess.check_call([sys.executable, '-c', code], cwd=gen_dir, stdout=subprocess.DEVNULL)
    path = os.path.join(outdir, 'cheat_sheet_01.json')
    with open(path) as f:
        return json.load(f), path


class DummyThread(object):
    def is_alive(self):
        return True

    def join(self):
        pass


def make_referee(judge_mod, cheat_path, outdir):
    R = judge_mod.Referee
    ref = R.__new__(R)
    ref.test_mode = False
    ref.debug = False
    ref.sim_mode = 'regular'
    ref.gameHasStarted = False
    ref.finishflag = 0
    ref.score_api_final_sent = True

    class Pub(object):
        def publish(self, *a):
            pass
    ref.checkpoint_result_publisher = Pub()
    ref.goal_result_publisher = Pub()
    ref.ib2_pose = None
    ref.cheatsheet_path = cheat_path
    ref.cheat_data = ref.load_json(cheat_path)
    ref._checkpoint_rules, ref._checkpoint_defaults = ref.load_checkpoint_rules()
    ref._goal_rule = ref.load_goal_rule()
    ref.judge_state = {"mission_started": False, "mission_finished": False,
                       "mission_start_ros_time": None, "mission_finish_ros_time": None,
                       "checkpoints": {}, "goal": None, "events": []}
    ref.output_dir = outdir
    ref.result_path = os.path.join(outdir, 'result.json')
    ref.finish_path = os.path.join(outdir, 'finish_report')
    ref.row_f = io.StringIO()
    ref.pose_path = os.path.join(outdir, 'pose.txt')
    ref.result_json = {"Mission Time": {}, "RoundingReport": {"timestamp": "", "arrival": {"try": False, "success": False}},
                       "FoundReport": {}, "Illegal": False, "TimeBonusN": 0, "JudgeReport": {}}
    ref.status = 0
    ref.thread = DummyThread()
    ref.stop_thread = threading.Event()
    ref.sync_judge_report()
    return ref


def run_one(level, seed, disturbance=True, verbose=False, speed=1.0, pos_bias=0.02, cheat=None):
    mockros.VERBOSE = verbose
    mockros.LOG.clear()
    mockros.SERVICES.clear()
    mockros.SUBSCRIBERS.clear()
    mockros.ACTION_SERVERS.clear()
    mockros.CLOCK.__init__()

    outdir = tempfile.mkdtemp(prefix='krpc_')
    if cheat:
        cheat_path = cheat
    else:
        cheat, cheat_path = gen_cheatsheet(level, seed, outdir)
    judge_mod = load_module('judge_%d' % seed, os.path.join(SIM, 'rootfs/krpc/judge_module/judge.py'))
    loader_mod = load_module('csl', os.path.join(SIM, 'users/catkin_ws/src/krpc_api_server/scripts/cheat_sheet_loader.py'))
    ref = make_referee(judge_mod, cheat_path, outdir)
    payload = loader_mod.CheatSheetLoader.from_path(cheat_path).checkpoint_payload()

    world = mockros.World(seed * 7 + 3, disturbance=disturbance, speed_scale=speed, pos_bias_max=pos_bias)
    mockros.ACTION_SERVERS['/ctl/command'] = world
    state = {'start': None, 'finish': None}

    def svc_start():
        state['start'] = mockros.CLOCK.now()
        ref.parse_judge_msg({'status': 'Mission Start'})
        return mockros.Msg(success=True, message='accepted')

    def svc_finish():
        state['finish'] = mockros.CLOCK.now()
        ref.parse_judge_msg({'status': 'Mission Finish'})
        return mockros.Msg(success=True, message='accepted')

    def svc_cps():
        return mockros.Msg(success=True, message=json.dumps(payload))

    mockros.SERVICES['/event/mission_start'] = svc_start
    mockros.SERVICES['/event/mission_finished'] = svc_finish
    mockros.SERVICES['/mission/get_checkpoints'] = svc_cps

    class CpServer(object):
        def on_goal(self, client, goal):
            ref.parse_judge_msg({'status': 'CheckPointPassed', 'checkpoint_id': goal.checkpoint_id})
            client.finish(mockros.GoalStatus.SUCCEEDED, mockros.Msg(success=True, message='accepted'))

        def on_cancel(self, client):
            pass
    mockros.ACTION_SERVERS['/event/checkpoint_passed'] = CpServer()

    user_mod = load_module('user_%d' % seed, USER)
    node = user_mod.UserTemplate()

    def update_pose():
        p = mockros.Pose()
        p.position.x, p.position.y, p.position.z = world.pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = world.q
        ref.ib2_pose = p

    stop = threading.Event()

    def sim_loop():
        dt = 0.02
        while not stop.is_set():
            world.step(dt)
            update_pose()
            msg = world.nav_msg()
            for cb in mockros.SUBSCRIBERS.get('/sensor_fusion/navigation', []):
                cb(msg)
            mockros.CLOCK.advance(dt)
            time.sleep(0.0004)

    th = threading.Thread(target=sim_loop, daemon=True)
    th.start()
    mockros.CLOCK.sleep(1.0)
    user_thread = threading.Thread(
        target=lambda: [cb(mockros.Msg(id=1)) for cb in mockros.SUBSCRIBERS['/ib2_user/start']], daemon=True)
    user_thread.start()
    t_begin = mockros.CLOCK.now()
    while user_thread.is_alive() and mockros.CLOCK.now() - t_begin < 340:
        time.sleep(0.05)
    stop.set()
    mockros.CLOCK.shutdown = True

    rep = ref.result_json.get('JudgeReport', {})
    details = rep.get('checkpoint_detail', [])
    n_cp = len(details)
    levels = [d['level'] for d in details]
    goal_ok = rep.get('goal', {}).get('success')
    dur = rep.get('mission', {}).get('duration_sec')
    in_time = dur is not None and dur <= 300.0
    return {
        'level': level, 'seed': seed, 'n_cp': n_cp, 'levels': levels,
        'passed': sum(1 for d in details if d['success']), 'lv3': levels.count(3),
        'goal': bool(goal_ok), 'duration': dur, 'in_time': in_time,
        'illegal': ref.result_json.get('Illegal'),
        'raw': ref.result_json.get('checkpoint_detail'),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--level', default='veryhard')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--batch', type=int, default=0)
    ap.add_argument('--no-dist', action='store_true')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--bias', type=float, default=0.02)
    ap.add_argument('-v', action='store_true')
    ap.add_argument('--cheat', default=None, help='replay a real cheat_sheet.json')
    a = ap.parse_args()
    if a.batch:
        rows = []
        for s in range(1, a.batch + 1):
            r = run_one(a.level, s, not a.no_dist, False, a.speed, a.bias)
            rows.append(r)
            print('seed %2d  cps %2d passed %2d lv3 %2d goal %-5s dur %6s %s levels=%s' % (
                s, r['n_cp'], r['passed'], r['lv3'], r['goal'], r['duration'],
                'OK' if r['in_time'] else 'LATE', r['levels']), flush=True)
        n = len(rows)
        print('---')
        print('runs=%d  all-lv3=%d  goal=%d  in_time=%d  mean_dur=%.1f  max_dur=%.1f  mean_lv3_ratio=%.3f' % (
            n, sum(r['lv3'] == r['n_cp'] for r in rows), sum(r['goal'] for r in rows),
            sum(r['in_time'] for r in rows),
            sum(r['duration'] or 999 for r in rows) / n, max(r['duration'] or 999 for r in rows),
            sum(r['lv3'] / float(r['n_cp']) for r in rows) / n))
    else:
        r = run_one(a.level, a.seed, not a.no_dist, a.v, a.speed, a.bias, a.cheat)
        print(json.dumps({k: v for k, v in r.items() if k != 'raw'}, indent=1))
        if a.v:
            for cid, d in sorted((r['raw'] or {}).items(), key=lambda x: int(x[0])):
                print(cid, 'lvl', d.get('selected_band'), 'dist %.3f' % d.get('distance', -1),
                      'center', d.get('centering_angle_deg'), 'roll', d.get('roll_angle_deg'))


if __name__ == '__main__':
    main()
