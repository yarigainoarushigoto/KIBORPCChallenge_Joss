"""
Minimal fake ROS (rospy/actionlib/msgs) + Int-Ball2 world model so the real
user program and the REAL judge module can run without ROS/Gazebo.

The world model is an approximation built from Int-Ball2 ctl.yaml:
  - trapezoid translation profile (v_max 0.2 m/s, a = 0.181/3.216 m/s^2)
  - attitude slew w_max 0.436 rad/s
  - action SUCCEEDED after staying within 0.10 m / 0.5 rad for 3 s
  - optional disturbance: constant steady-state offset (like the microgravity module,
    the PD controller has ki=0) + noise
"""
import sys
import types
import math
import random
import threading
import time as _time


# ---------------------------------------------------------------------------
# Clock
# ---------------------------------------------------------------------------
class Clock(object):
    def __init__(self):
        self.t = 1000.0
        self.cv = threading.Condition()
        self.shutdown = False

    def now(self):
        return self.t

    def advance(self, dt):
        with self.cv:
            self.t += dt
            self.cv.notify_all()

    def sleep(self, d):
        target = self.t + d
        with self.cv:
            while self.t < target and not self.shutdown:
                self.cv.wait(0.05)


CLOCK = Clock()


# ---------------------------------------------------------------------------
# Generic message objects
# ---------------------------------------------------------------------------
class Msg(object):
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Vec(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Quat(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Pose(object):
    def __init__(self):
        self.position = Vec()
        self.orientation = Quat()


class Header(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = ''


class PoseStamped(object):
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class CtlStatusType(object):
    STAND_BY = 0
    KEEP_POSE = 10
    STOP_MOVING = 20
    MOVE_TO_RELATIVE_TARGET = 30
    MOVE_TO_ABSOLUTE_TARGET = 40

    def __init__(self, type=0):
        self.type = type


class CtlCommandGoal(object):
    def __init__(self, target=None, type=None):
        self.target = target
        self.type = type


class CtlCommandResult(object):
    TERMINATE_SUCCESS = 0
    TERMINATE_ABORTED = 1
    TERMINATE_TIME_OUT = 2
    TERMINATE_INVALID_NAV = 3
    TERMINATE_INVALID_CMD = 4

    def __init__(self, type=0):
        self.type = type


class CheckpointPassedGoal(object):
    def __init__(self):
        self.checkpoint_id = 0


class StopProcessingUserNodeResponse(object):
    SUCCESS = 0
    ERROR = 255

    def __init__(self, result=0):
        self.result = result


class SrvType(object):
    pass


# ---------------------------------------------------------------------------
# rospy
# ---------------------------------------------------------------------------
LOG = []
SERVICES = {}
SUBSCRIBERS = {}
PARAMS = {}


class ROSException(Exception):
    pass


class Duration(object):
    def __init__(self, secs=0.0):
        self.secs = float(secs)

    def to_sec(self):
        return self.secs


class Time(object):
    def __init__(self, t):
        self.t = t

    @staticmethod
    def now():
        return Time(CLOCK.now())

    def to_sec(self):
        return self.t


def _log(level, fmt, *args):
    try:
        text = fmt % args if args else str(fmt)
    except Exception:
        text = str(fmt) + ' ' + str(args)
    if text.startswith("{'Illegal'") or text.endswith('result.json'):
        return  # judge debug dump
    LOG.append((CLOCK.now(), level, text))
    if VERBOSE:
        print('[%7.2f][%s] %s' % (CLOCK.now() - 1000.0, level, text))


VERBOSE = False


def make_rospy():
    m = types.ModuleType('rospy')
    m.init_node = lambda *a, **k: None
    m.loginfo = lambda f, *a: _log('INFO', f, *a)
    m.logwarn = lambda f, *a: _log('WARN', f, *a)
    m.logerr = lambda f, *a: _log('ERROR', f, *a)
    m.logdebug = lambda f, *a: None
    m.logwarn_throttle = lambda p, f, *a: _log('WARN', f, *a)
    m.get_time = CLOCK.now
    m.Time = Time
    m.Duration = Duration
    m.ROSException = ROSException
    m.sleep = lambda d: CLOCK.sleep(d.to_sec() if isinstance(d, Duration) else d)
    m.is_shutdown = lambda: CLOCK.shutdown
    m.set_param = lambda k, v: PARAMS.__setitem__(k, v)
    m.get_param = lambda k, d=None: PARAMS.get(k, d)
    m.spin = lambda: None
    m.signal_shutdown = lambda *a: None
    m.on_shutdown = lambda f: None

    def wait_for_service(name, timeout=None):
        if name not in SERVICES:
            raise ROSException('no service ' + name)
    m.wait_for_service = wait_for_service

    class ServiceProxy(object):
        def __init__(self, name, typ):
            self.name = name

        def __call__(self, *a):
            return SERVICES[self.name]()
    m.ServiceProxy = ServiceProxy

    class Service(object):
        def __init__(self, name, typ, cb):
            SERVICES['user:' + name] = cb
    m.Service = Service

    class Subscriber(object):
        def __init__(self, name, typ, cb, queue_size=None):
            SUBSCRIBERS.setdefault(name, []).append(cb)
    m.Subscriber = Subscriber

    class Publisher(object):
        def __init__(self, *a, **k):
            pass

        def publish(self, *a):
            pass
    m.Publisher = Publisher

    class Rate(object):
        def __init__(self, hz):
            self.p = 1.0 / hz

        def sleep(self):
            pass
    m.Rate = Rate
    return m


# ---------------------------------------------------------------------------
# actionlib
# ---------------------------------------------------------------------------
class GoalStatus(object):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


ACTION_SERVERS = {}


class SimpleActionClient(object):
    def __init__(self, name, typ):
        self.name = name
        self.state = GoalStatus.LOST
        self.result = None
        self.done = threading.Event()

    def wait_for_server(self, timeout=None):
        return self.name in ACTION_SERVERS

    def send_goal(self, goal):
        server = ACTION_SERVERS[self.name]
        # a real ActionClient goal replaces the previous one: preempt it first
        if self.state == GoalStatus.ACTIVE and hasattr(server, 'on_cancel'):
            server.on_cancel(self)
        self.done.clear()
        self.state = GoalStatus.ACTIVE
        self.result = None
        server.on_goal(self, goal)

    def finish(self, state, result):
        self.state = state
        self.result = result
        self.done.set()

    def cancel_goal(self):
        ACTION_SERVERS[self.name].on_cancel(self)

    def wait_for_result(self, timeout=None):
        t = timeout.to_sec() if timeout is not None else 1e9
        end = CLOCK.now() + t
        while CLOCK.now() < end and not self.done.is_set():
            CLOCK.sleep(0.02)
        return self.done.is_set()

    def get_result(self):
        return self.result

    def get_state(self):
        return self.state


def make_actionlib():
    m = types.ModuleType('actionlib')
    m.SimpleActionClient = SimpleActionClient
    m.GoalStatus = GoalStatus
    return m


# ---------------------------------------------------------------------------
# Quaternion helpers for the world
# ---------------------------------------------------------------------------
def qn(q):
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def slerp(a, b, t):
    a, b = qn(a), qn(b)
    d = sum(x * y for x, y in zip(a, b))
    if d < 0:
        b = [-c for c in b]
        d = -d
    if d > 0.9995:
        return qn([x + t * (y - x) for x, y in zip(a, b)])
    th = math.acos(d)
    s = math.sin(th)
    wa = math.sin((1 - t) * th) / s
    wb = math.sin(t * th) / s
    return qn([wa * x + wb * y for x, y in zip(a, b)])


def qangle(a, b):
    d = abs(sum(x * y for x, y in zip(qn(a), qn(b))))
    return 2 * math.acos(min(1.0, d))


def axis_angle(axis, ang):
    n = math.sqrt(sum(c * c for c in axis))
    s = math.sin(ang / 2)
    return [axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(ang / 2)]


# ---------------------------------------------------------------------------
# Int-Ball2 world
# ---------------------------------------------------------------------------
class World(object):
    V_MAX = 0.20
    A_MAX = 0.181 / 3.216
    W_MAX = 0.436

    def __init__(self, seed, disturbance=True, pos_bias_max=0.02, att_bias_max_deg=1.0,
                 noise_pos=0.003, noise_att_deg=0.3, speed_scale=1.0):
        self.rng = random.Random(seed)
        self.ref = [0.2, 0.0, 0.0]          # dock station (real run starts at x~0.20)
        self.refv = 0.0
        self.refw = 0.0
        self.refq = [0.0, 0.0, 0.0, 1.0]
        self.target = None
        self.targetq = None
        self.pending = None
        self.hold_until = 0.0
        self.client = None
        self.inside_since = None
        self.offset = [0.0, 0.0, 0.0]
        self.speed_scale = speed_scale
        self.noise_pos = noise_pos
        self.noise_att = math.radians(noise_att_deg)
        if disturbance:
            d = [self.rng.gauss(0, 1) for _ in range(3)]
            n = math.sqrt(sum(c * c for c in d))
            mag = self.rng.uniform(0.0, pos_bias_max)
            self.bias = [c / n * mag for c in d]
            ax = [self.rng.gauss(0, 1) for _ in range(3)]
            self.att_bias = axis_angle(ax, math.radians(self.rng.uniform(0, att_bias_max_deg)))
        else:
            self.bias = [0.0, 0.0, 0.0]
            self.att_bias = [0.0, 0.0, 0.0, 1.0]
        self.pos = list(self.ref)
        self.q = list(self.refq)
        self.lock = threading.Lock()

    # action server interface ------------------------------------------------
    def on_goal(self, client, goal):
        with self.lock:
            p = goal.target.pose.position
            o = goal.target.pose.orientation
            self.pending = ([p.x, p.y, p.z], qn([o.x, o.y, o.z, o.w]))
            self.client = client
            self.inside_since = None
            if CLOCK.now() >= self.hold_until:
                self._start_pending()

    def _start_pending(self):
        self.target, self.targetq = self.pending
        self.pending = None
        self.refv = 0.0
        self.refw = 0.0

    def on_cancel(self, client):
        with self.lock:
            if client.state == GoalStatus.ACTIVE:
                # real ctl: cancelTarget() -> stopping profile, held 3 s (duration_goal)
                self.target = list(self.ref)
                self.targetq = list(self.refq)
                self.refv = 0.0
                self.hold_until = CLOCK.now() + 3.0
                client.finish(GoalStatus.PREEMPTED, CtlCommandResult(CtlCommandResult.TERMINATE_ABORTED))

    # physics -----------------------------------------------------------------
    # Fitted to a real local Gazebo run: translate first (attitude frozen), then rotate
    # in place; translation a=0.0127 m/s^2 bang-bang (v<=0.2), rotation alpha=0.145 rad/s^2,
    # w<=0.436 rad/s.
    A_TRANS = 0.0127
    A_ROT = 0.145

    def step(self, dt):
        with self.lock:
            if self.pending is not None and CLOCK.now() >= self.hold_until:
                self._start_pending()
            if self.target is not None and CLOCK.now() >= self.hold_until:
                d = [t - r for t, r in zip(self.target, self.ref)]
                dist = math.sqrt(sum(c * c for c in d))
                vmax = self.V_MAX * self.speed_scale
                amax = self.A_TRANS * self.speed_scale
                if dist > 1e-4:
                    v_stop = math.sqrt(2 * amax * dist)
                    v_des = min(vmax, v_stop)
                    if self.refv < v_des:
                        self.refv = min(v_des, self.refv + amax * dt)
                    else:
                        self.refv = max(v_des, self.refv - amax * dt)
                    stepd = min(dist, max(self.refv, 1e-3) * dt)
                    self.ref = [r + c / dist * stepd for r, c in zip(self.ref, d)]
                    self.refw = 0.0
                else:
                    self.refv = 0.0
                    ang = qangle(self.refq, self.targetq)
                    if ang > 1e-4:
                        wmax = self.W_MAX * self.speed_scale
                        al = self.A_ROT * self.speed_scale
                        w_des = min(wmax, math.sqrt(2 * al * ang))
                        if self.refw < w_des:
                            self.refw = min(w_des, self.refw + al * dt)
                        else:
                            self.refw = max(w_des, self.refw - al * dt)
                        t = min(1.0, max(self.refw, 1e-3) * dt / ang)
                        self.refq = slerp(self.refq, self.targetq, t)
                    else:
                        self.refw = 0.0
            # actual = reference + (first-order) bias + noise
            k = min(1.0, dt / 2.0)
            self.offset = [o + k * (b - o) for o, b in zip(self.offset, self.bias)]
            self.pos = [r + o + self.rng.gauss(0, self.noise_pos) for r, o in zip(self.ref, self.offset)]
            nq = axis_angle([self.rng.gauss(0, 1) for _ in range(3)], abs(self.rng.gauss(0, self.noise_att)))
            self.q = qn(qmul(nq, qmul(self.att_bias, self.refq)))
            # action success check (ctl reachGoal: 0.10 m / 0.5 rad held 3 s)
            if (self.client is not None and self.client.state == GoalStatus.ACTIVE
                    and self.target is not None and self.pending is None):
                dpos = math.sqrt(sum((a - b) ** 2 for a, b in zip(self.pos, self.target)))
                dang = qangle(self.q, self.targetq)
                if dpos < 0.10 and dang < 0.5:
                    if self.inside_since is None:
                        self.inside_since = CLOCK.now()
                    elif CLOCK.now() - self.inside_since >= 3.0:
                        self.client.finish(GoalStatus.SUCCEEDED, CtlCommandResult(CtlCommandResult.TERMINATE_SUCCESS))
                else:
                    self.inside_since = None

    def nav_msg(self):
        m = Msg()
        m.pose = PoseStamped()
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = self.pos
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = self.q
        return m


def install_fake_modules():
    sys.modules['rospy'] = make_rospy()
    sys.modules['actionlib'] = make_actionlib()

    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    mod('platform_msgs')
    mod('platform_msgs.msg', UserLogic=Msg)
    mod('platform_msgs.srv', StopProcessingUserNode=SrvType,
        StopProcessingUserNodeResponse=StopProcessingUserNodeResponse)
    mod('geometry_msgs')
    mod('geometry_msgs.msg', PoseStamped=PoseStamped, Pose=Pose)
    mod('krpc_ib2_msgs')
    mod('krpc_ib2_msgs.srv', MissionStart=SrvType, MissionFinished=SrvType, CheckPoints=SrvType)
    mod('krpc_ib2_msgs.msg', CheckpointPassedAction=SrvType, CheckpointPassedGoal=CheckpointPassedGoal)
    mod('ib2_msgs')
    mod('ib2_msgs.msg', CtlCommandAction=SrvType, CtlCommandGoal=CtlCommandGoal,
        CtlCommandResult=CtlCommandResult, CtlStatusType=CtlStatusType, Navigation=Msg)
    try:
        import requests  # noqa: F401  (judge imports it; not needed offline)
    except ImportError:
        mod('requests', post=lambda *a, **k: None)
    mod('std_msgs')
    mod('std_msgs.msg', String=Msg, Bool=Msg)

    class _TB(object):
        def sendTransform(self, *a):
            pass

    class _TL(object):
        pass
    mod('tf', TransformBroadcaster=_TB, TransformListener=_TL,
        transformations=types.SimpleNamespace(quaternion_matrix=lambda q: None))
