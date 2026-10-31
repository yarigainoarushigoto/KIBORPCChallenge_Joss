#!/usr/bin/python3
# -*- coding:utf-8 -*-
"""
Kibo-RPC 7th - Int-Ball2 mission program (Python 3.6.9 / ROS Melodic)

Strategy
--------
1. Mission start -> read all checkpoints (position, marker pose, attitude, bands).
2. Plan the visiting order with an exact DP (Held-Karp) over estimated flight time,
   starting from the current pose and always ending at the goal checkpoint.
3. For every checkpoint:
     - fly there with /ctl/command (MOVE_TO_ABSOLUTE_TARGET),
     - evaluate ourselves with the SAME formula as the judge (position distance,
       camera centering on the AR marker, roll around the line of sight) using
       /sensor_fusion/navigation (the judge uses this topic too),
     - report /event/checkpoint_passed only after the best level is held for
       HOLD_SEC (each checkpoint can be reported ONCE - the judge ignores repeats),
     - if the level is not good enough, re-send a bias-compensated target
       (cancels steady-state offset caused by disturbances) a limited number of times.
4. A time manager keeps a reserve so the goal is always reached and
   /event/mission_finished is always sent inside the 5 minute limit.

Only APIs from Programming Manual chapter 6 are used.
"""
import json
import math
import threading

import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType
from ib2_msgs.msg import Navigation

PROGRAM_VERSION = 'krpc7-route-v1.2'

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
MISSION_TIME_LIMIT = 300.0      # [s] rulebook: 5 minutes from Mission Start
FINISH_MARGIN = 12.0            # [s] mission_finished must be sent before limit - margin
HOLD_SEC = 1.0                  # [s] level must be held this long before reporting (rulebook: 1 s)
SETTLE_SEC = 3.0                # [s] extra wait after a move ends, waiting for level 3
SETTLE_WINDOW = 1.5             # [s] robot is "settled" if it moved less than below in this window
SETTLE_POS = 0.012              # [m]
SETTLE_ATT_DEG = 1.0            # [deg]
SETTLE_NEAR = 0.25              # [m] settled only counts this close to the commanded target
SETTLE_NEAR_DEG = 20.0          # [deg] ... and this close to the commanded attitude
POST_REPORT_WAIT = 2.5
BIAS_ALPHA = 0.5                # EMA weight of the newest offset observation
BIAS_FF_GAIN = 0.8              # fraction of the learned offset pre-compensated on the first try
PREEMPT_HOLD = 3.0              # [s] ctl cancelTarget(): stop + 3 s hold before the next goal          # [s] max wait for the ctl action to finish after a report
MAX_CORRECTIONS = 2             # bias-compensated re-sends per checkpoint
MOVE_TIMEOUT_MIN = 30.0         # [s] lower bound of a single move timeout
SERVICE_TIMEOUT = 10.0          # [s]
POLL_DT = 0.05                  # [s] evaluation loop period
NAV_STALE_SEC = 2.0             # [s] navigation older than this is ignored

# Motion model for planning / time budgeting, fitted to a real local Gazebo run (sim time):
#  - the ctl TRANSLATES first (attitude frozen), THEN rotates in place
#  - translation ~ bang-bang with a = 0.0127 m/s^2 (1.5 m takes ~22 s, peak ~0.13 m/s)
#  - rotation: w_max 0.436 rad/s (25 deg/s), alpha ~0.145 rad/s^2
#  - a new goal starts moving ~3 s after it is sent (cancel/stop hold of the previous goal)
V_MAX = 0.20
A_MAX = 0.0127
W_MAX = 0.436
ALPHA_MAX = 0.145
START_DELAY = 3.0               # [s] from send_goal until the robot really starts
STOP_OVERHEAD = 1.0             # [s] settling after the motion profile ends
REPORT_OVERHEAD = 1.2           # [s] HOLD_SEC + report round trip
PLAN_SAFETY = 1.05              # multiply estimated times (updated online from measured legs)

# Safe flight box (area_config.json "SafeArea"); straight lines inside a box stay inside.
SAFE_MIN = (0.20, -0.80, 0.00)
SAFE_MAX = (7.00, 0.85, 1.68)

CAM_OFFSET = 0.14               # camera is 0.14 m in front of the body centre (+X)


# ---------------------------------------------------------------------------
# Small math helpers (quaternion order: x, y, z, w - same as ROS / judge)
# ---------------------------------------------------------------------------
def v_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def v_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def v_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_unit(a, fallback=(1.0, 0.0, 0.0)):
    n = v_norm(a)
    if n < 1e-9:
        return list(fallback)
    return [a[0] / n, a[1] / n, a[2] / n]


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def q_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def q_rotate(q, v):
    q = q_norm(q)
    r = q_mul(q_mul(q, [v[0], v[1], v[2], 0.0]), q_conj(q))
    return [r[0], r[1], r[2]]


def q_from_axis_angle(axis, angle):
    u = v_unit(axis)
    s = math.sin(angle * 0.5)
    return [u[0] * s, u[1] * s, u[2] * s, math.cos(angle * 0.5)]


def q_angle_between(a, b):
    d = abs(sum(x * y for x, y in zip(q_norm(a), q_norm(b))))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def q_from_two_vectors(a, b):
    """Shortest rotation that maps unit vector a onto unit vector b."""
    a = v_unit(a)
    b = v_unit(b)
    d = v_dot(a, b)
    if d > 1.0 - 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    if d < -1.0 + 1e-9:
        perp = v_cross(a, [1.0, 0.0, 0.0])
        if v_norm(perp) < 1e-6:
            perp = v_cross(a, [0.0, 1.0, 0.0])
        return q_from_axis_angle(perp, math.pi)
    c = v_cross(a, b)
    return q_norm([c[0], c[1], c[2], 1.0 + d])


def twist_angle_deg(q, axis):
    """Signed twist of q around axis (identical to judge._twist_angle_deg)."""
    qx, qy, qz, qw = q_norm(q)
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    d = qx * axis[0] + qy * axis[1] + qz * axis[2]
    return math.degrees(2.0 * math.atan2(d, qw))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def clamp_safe(p):
    return [min(max(p[i], SAFE_MIN[i]), SAFE_MAX[i]) for i in range(3)]


# ---------------------------------------------------------------------------
# Judge-equivalent evaluation
# ---------------------------------------------------------------------------
def band_level(value, bands):
    """bands: list of (threshold, level) sorted by threshold. 0 = outside."""
    for threshold, level in bands:
        if value <= threshold:
            return level
    return 0


def parse_bands(objs, key):
    bands = []
    if not isinstance(objs, list):
        return bands
    for b in objs:
        if not isinstance(b, dict):
            continue
        try:
            bands.append((float(b.get(key)), int(b.get('level'))))
        except (TypeError, ValueError):
            continue
    bands.sort(key=lambda x: x[0])
    return bands


def evaluate_pose(cp, pos, quat):
    """Return (level, detail) exactly like the judge would for this pose."""
    dist = v_norm(v_sub(pos, cp['position']))
    lv_pos = band_level(dist, cp['pos_bands'])
    levels = [lv_pos]
    detail = {'dist': dist, 'lv_pos': lv_pos, 'center': None, 'roll': None}
    if cp['target_q'] is not None and (cp['center_bands'] or cp['roll_bands']):
        view = v_unit(v_sub(cp['marker_pos'], pos))
        fwd = v_unit(q_rotate(quat, [1.0, 0.0, 0.0]))
        center = math.degrees(math.acos(max(-1.0, min(1.0, v_dot(fwd, view)))))
        roll = abs(wrap_deg(twist_angle_deg(quat, view) - twist_angle_deg(cp['target_q'], view)))
        detail['center'] = center
        detail['roll'] = roll
        if cp['center_bands']:
            levels.append(band_level(center, cp['center_bands']))
        if cp['roll_bands']:
            levels.append(band_level(roll, cp['roll_bands']))
    level = 0 if any(l == 0 for l in levels) else min(levels)
    detail['level'] = level
    return level, detail


def aim_quaternion(from_pos, marker_pos, target_q):
    """Attitude whose +X axis looks at the marker from from_pos and whose roll
    around the line of sight matches target_q (as measured by the judge)."""
    view = v_unit(v_sub(marker_pos, from_pos))
    if target_q is None:
        return q_from_two_vectors([1.0, 0.0, 0.0], view)
    fwd_t = q_rotate(target_q, [1.0, 0.0, 0.0])
    q = q_norm(q_mul(q_from_two_vectors(fwd_t, view), target_q))
    # fine-tune roll around view so that the judge's twist difference is ~0
    for _ in range(3):
        err = wrap_deg(twist_angle_deg(q, view) - twist_angle_deg(target_q, view))
        if abs(err) < 0.05:
            break
        q = q_norm(q_mul(q_from_axis_angle(view, -math.radians(err)), q))
    return q


# ---------------------------------------------------------------------------
# Planning helpers
# ---------------------------------------------------------------------------
def move_time(dist):
    """Trapezoidal (bang-coast-bang) translation time."""
    if dist <= 1e-6:
        return 0.0
    d_acc = V_MAX * V_MAX / A_MAX
    if dist < d_acc:
        return 2.0 * math.sqrt(dist / A_MAX)
    return dist / V_MAX + V_MAX / A_MAX


def rot_time(angle):
    if angle <= 1e-6:
        return 0.0
    a_acc = W_MAX * W_MAX / ALPHA_MAX
    if angle < a_acc:
        return 2.0 * math.sqrt(angle / ALPHA_MAX)
    return angle / W_MAX + W_MAX / ALPHA_MAX


def leg_time(p0, q0, p1, q1):
    """Estimated sim seconds from sending the goal until the checkpoint is reported.
    Translation and rotation are SEQUENTIAL in the real ctl, so the times add up."""
    return START_DELAY + move_time(v_norm(v_sub(p1, p0))) + rot_time(q_angle_between(q0, q1)) \
        + STOP_OVERHEAD + REPORT_OVERHEAD


def plan_route(start_p, start_q, cps, goal_id, budget=float('inf'), safety=1.0):
    """Orienteering with Held-Karp: among all subsets/orders that still reach the goal
    inside `budget` seconds (estimates * safety), pick the one visiting the MOST
    checkpoints, ties broken by the shortest time. Goal is always last.
    Returns (route, estimated_seconds)."""
    others = [c for c in cps if c['id'] != goal_id]
    goal = [c for c in cps if c['id'] == goal_id]
    n = len(others)
    g = goal[0] if goal else None

    def end_of(c):
        return leg_time(c['position'], c['cmd_q'], g['position'], g['cmd_q']) if g else 0.0

    direct = leg_time(start_p, start_q, g['position'], g['cmd_q']) if g else 0.0
    if n == 0:
        return goal, direct
    if n > 13:  # never happens (max 11 + goal); greedy fallback
        order, cur_p, cur_q, left = [], start_p, start_q, list(others)
        while left:
            nxt = min(left, key=lambda c: leg_time(cur_p, cur_q, c['position'], c['cmd_q']))
            order.append(nxt)
            left.remove(nxt)
            cur_p, cur_q = nxt['position'], nxt['cmd_q']
        return order + goal, float('inf')

    start_cost = [leg_time(start_p, start_q, c['position'], c['cmd_q']) for c in others]
    cost = [[leg_time(a['position'], a['cmd_q'], b['position'], b['cmd_q']) for b in others]
            for a in others]
    end_cost = [end_of(c) for c in others]

    INF = float('inf')
    size = 1 << n
    dp = [[INF] * n for _ in range(size)]
    parent = [[-1] * n for _ in range(size)]
    for i in range(n):
        dp[1 << i][i] = start_cost[i]
    for mask in range(size):
        row = dp[mask]
        for last in range(n):
            base = row[last]
            if base == INF:
                continue
            crow = cost[last]
            for nxt in range(n):
                bit = 1 << nxt
                if mask & bit:
                    continue
                val = base + crow[nxt]
                nm = mask | bit
                if val < dp[nm][nxt]:
                    dp[nm][nxt] = val
                    parent[nm][nxt] = last

    best_key, best_mask, best_last, best_total = None, 0, -1, direct
    for mask in range(1, size):
        cnt = bin(mask).count('1')
        row = dp[mask]
        for last in range(n):
            if row[last] == INF:
                continue
            total = row[last] + end_cost[last]
            if total * safety > budget:
                continue
            key = (cnt, -total)
            if best_key is None or key > best_key:
                best_key, best_mask, best_last, best_total = key, mask, last, total
    if best_key is None:
        return goal, direct
    order_idx, mask, cur = [], best_mask, best_last
    while cur != -1:
        order_idx.append(cur)
        prev = parent[mask][cur]
        mask &= ~(1 << cur)
        cur = prev
    order_idx.reverse()
    return [others[i] for i in order_idx] + goal, best_total


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action clients
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service clients
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Navigation (same source the judge uses)
        self.__nav_lock = threading.Lock()
        self.__nav_pose = None
        self.__nav_stamp = None
        self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav, queue_size=1)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        self.__checkpoint_active = False
        self.__stop_requested = False
        self.__t0 = None
        self.__speed_factor = PLAN_SAFETY
        self.__bias = [0.0, 0.0, 0.0]
        self.__reported = set()
        self.__results = []
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE
        rospy.loginfo('[%s] node ready', PROGRAM_VERSION)

    # ------------------------------------------------------------------ time
    def __elapsed(self):
        if self.__t0 is None:
            return 0.0
        return rospy.get_time() - self.__t0

    def __time_left(self):
        return MISSION_TIME_LIMIT - FINISH_MARGIN - self.__elapsed()

    # ------------------------------------------------------------------ navigation
    def __callback_nav(self, msg):
        try:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            with self.__nav_lock:
                self.__nav_pose = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
                self.__nav_stamp = rospy.get_time()
        except Exception as exc:  # never let a callback crash the node
            rospy.logwarn_throttle(5.0, 'nav callback error: %s' % exc)

    def __current_pose(self):
        with self.__nav_lock:
            pose, stamp = self.__nav_pose, self.__nav_stamp
        if pose is None or stamp is None:
            return None
        if rospy.get_time() - stamp > NAV_STALE_SEC:
            return None
        return pose

    # ------------------------------------------------------------------ mission events
    def __wait_service(self, name):
        try:
            rospy.wait_for_service(name, timeout=SERVICE_TIMEOUT)
            return True
        except rospy.ROSException:
            rospy.logerr('service %s not available', name)
            return False

    def __notify_mission_start(self):
        if not self.__wait_service('/event/mission_start'):
            return False
        res = self.__mission_start()
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        if not self.__wait_service('/event/mission_finished'):
            return False
        res = self.__mission_finished()
        rospy.loginfo('mission_finished result: success=%s message=%s (elapsed %.1f s)',
                      res.success, res.message, self.__elapsed())
        return res.success

    def __notify_checkpoint_passed(self, checkpoint_id):
        if checkpoint_id in self.__reported:
            rospy.logwarn('checkpoint %d already reported - skipped', checkpoint_id)
            return False
        self.__reported.add(checkpoint_id)
        rospy.loginfo('notify checkpoint_id=%d passed (elapsed %.1f s)', checkpoint_id, self.__elapsed())
        if not self.__checkpoint_client.wait_for_server(rospy.Duration(SERVICE_TIMEOUT)):
            rospy.logerr('checkpoint_passed server not available')
            return False
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = int(checkpoint_id)
        self.__checkpoint_active = True
        try:
            self.__checkpoint_client.send_goal(goal)
            deadline = rospy.get_time() + SERVICE_TIMEOUT
            while not rospy.is_shutdown() and not self.__stop_requested:
                if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                    result = self.__checkpoint_client.get_result()
                    rospy.loginfo('checkpoint_passed result: success=%s message=%s',
                                  getattr(result, 'success', None), getattr(result, 'message', None))
                    return bool(result and result.success)
                if rospy.get_time() > deadline:
                    rospy.logwarn('checkpoint_passed timed out')
                    break
            self.__checkpoint_client.cancel_goal()
            return False
        finally:
            self.__checkpoint_active = False

    def __get_checkpoints(self):
        for attempt in range(3):
            if not self.__wait_service('/mission/get_checkpoints'):
                continue
            try:
                res = self.__checkpoints()
                if not res.success:
                    rospy.logwarn('get_checkpoints failed: %s', res.message)
                    rospy.sleep(0.5)
                    continue
                payload = json.loads(res.message)
                return payload
            except Exception as exc:
                rospy.logwarn('get_checkpoints attempt %d error: %s', attempt + 1, exc)
                rospy.sleep(0.5)
        return None

    # ------------------------------------------------------------------ checkpoint parsing
    def __parse_checkpoints(self, payload):
        raw = payload.get('checkpoints', payload.get('Checkpoints', []))
        cps = []
        for item in raw:
            if not isinstance(item, dict):
                continue
            cid = item.get('checkpoint_id', item.get('checkpointid', item.get('id')))
            pos = item.get('position')
            if cid is None or not isinstance(pos, list) or len(pos) < 3:
                continue
            marker = item.get('marker_pose') or {}
            marker_pos = marker.get('position') if isinstance(marker, dict) else None
            if not isinstance(marker_pos, list) or len(marker_pos) < 3:
                marker_pos = None
            att = item.get('attitude') if isinstance(item.get('attitude'), dict) else {}
            target_q = att.get('quaternion')
            if isinstance(target_q, list) and len(target_q) >= 4:
                target_q = q_norm([float(x) for x in target_q[:4]])
            else:
                target_q = None
            pos_bands = parse_bands(item.get('score_bands'), 'radius') or [(0.1, 3), (0.2, 2), (0.3, 1)]
            cp = {
                'id': int(cid),
                'position': [float(x) for x in pos[:3]],
                'marker_pos': [float(x) for x in marker_pos[:3]] if marker_pos else None,
                'target_q': target_q,
                'pos_bands': pos_bands,
                'center_bands': parse_bands(att.get('centering_bands'), 'angle'),
                'roll_bands': parse_bands(att.get('roll_bands'), 'angle'),
            }
            if cp['marker_pos'] is None:          # cannot evaluate attitude -> position only
                cp['center_bands'], cp['roll_bands'] = [], []
            cp['cmd_pos'] = clamp_safe(cp['position'])
            if cp['marker_pos'] is not None:
                cp['cmd_q'] = aim_quaternion(cp['position'], cp['marker_pos'], target_q)
            else:
                cp['cmd_q'] = target_q if target_q is not None else [0.0, 0.0, 0.0, 1.0]
            cps.append(cp)
        goal_id = payload.get('goalpoint_id', payload.get('goal_checkpoint_id'))
        if goal_id is not None:
            goal_id = int(goal_id)
        return cps, goal_id

    # ------------------------------------------------------------------ movement
    def __send_move(self, pos, quat):
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'iss_body'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pos
        q = q_norm(quat)
        target.pose.orientation.x, target.pose.orientation.y = q[0], q[1]
        target.pose.orientation.z, target.pose.orientation.w = q[2], q[3]
        goal = CtlCommandGoal(target=target, type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))
        self.__action_active = True
        self.__command.send_goal(goal)

    def __cancel_move(self):
        if self.__action_active:
            self.__command.cancel_goal()
            self.__command.wait_for_result(rospy.Duration(10.0))
            self.__action_active = False

    def __watch(self, cp, deadline, stop_when_done=True, min_time=0.0, near=None, near_q=None):
        """Poll the pose; return as soon as level 3 is held for HOLD_SEC.
        Also returns early when the move action ended or the robot has settled
        (stopped moving) - waiting longer would not improve the level.
        Returns (reported_ok, best_level_held, reason)."""
        hold_start = {}
        best_held = 0
        history = []          # (t, pos, quat) of the last SETTLE_WINDOW seconds
        t_begin = rospy.get_time()
        while not rospy.is_shutdown() and not self.__stop_requested:
            now = rospy.get_time()
            pose = self.__current_pose()
            if pose is not None:
                level, _ = evaluate_pose(cp, pose[0], pose[1])
                for lv in (1, 2, 3):
                    if level >= lv:
                        hold_start.setdefault(lv, now)
                        if now - hold_start[lv] >= HOLD_SEC:
                            best_held = max(best_held, lv)
                    else:
                        hold_start.pop(lv, None)
                if best_held >= 3:
                    return True, 3, 'level3'
                history.append((now, pose[0], pose[1]))
                while history and now - history[0][0] > SETTLE_WINDOW:
                    history.pop(0)
                near_ok = ((near is None or v_norm(v_sub(pose[0], near)) < SETTLE_NEAR) and
                           (near_q is None or math.degrees(q_angle_between(pose[1], near_q)) < SETTLE_NEAR_DEG))
                if (now - t_begin > max(min_time, SETTLE_WINDOW + 0.5) and 3 not in hold_start
                        and near_ok and self.__is_settled(history)):
                    return False, best_held, 'settled'
            if now > deadline:
                return False, best_held, 'deadline'
            if stop_when_done and self.__move_done() and 3 not in hold_start:
                return False, best_held, 'move_done'
            rospy.sleep(POLL_DT)
        return False, best_held, 'stopped'

    @staticmethod
    def __is_settled(history):
        if len(history) < 5 or history[-1][0] - history[0][0] < SETTLE_WINDOW * 0.9:
            return False
        p_last, q_last = history[-1][1], history[-1][2]
        for _, p, q in history:
            if v_norm(v_sub(p, p_last)) > SETTLE_POS or math.degrees(q_angle_between(q, q_last)) > SETTLE_ATT_DEG:
                return False
        return True

    def __move_done(self):
        if not self.__action_active:
            return True
        state = self.__command.get_state()
        done = state in (actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED,
                         actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.REJECTED,
                         actionlib.GoalStatus.RECALLED, actionlib.GoalStatus.LOST)
        if done:
            self.__action_active = False
            result = self.__command.get_result()
            rtype = getattr(result, 'type', None)
            if rtype != CtlCommandResult.TERMINATE_SUCCESS:
                rospy.logwarn('move ended with state=%s result=%s', state, rtype)
        return done

    def __visit(self, cp, hard_deadline):
        """Fly to cp and report it. Returns achieved level (0 if not reported/failed)."""
        cid = cp['id']
        start_pose = self.__current_pose()
        t_leg_start = rospy.get_time()
        if start_pose is not None:
            est = leg_time(start_pose[0], start_pose[1], cp['cmd_pos'], cp['cmd_q'])
        else:
            est = 30.0
        est0 = est
        cmd_pos = clamp_safe(v_sub(cp['cmd_pos'], v_scale(self.__bias, BIAS_FF_GAIN)))
        cmd_q = list(cp['cmd_q'])
        best_seen = 0
        # never treat the robot as "settled" before it could even have started moving
        min_time = START_DELAY + 1.0

        for attempt in range(MAX_CORRECTIONS + 1):
            move_timeout = max(MOVE_TIMEOUT_MIN, est * 1.6 + 5.0)
            deadline = min(rospy.get_time() + move_timeout, hard_deadline)
            rospy.loginfo('-> cp %d attempt %d cmd_pos=(%.3f, %.3f, %.3f)', cid, attempt,
                          cmd_pos[0], cmd_pos[1], cmd_pos[2])
            self.__send_move(cmd_pos, cmd_q)
            reported, held, why = self.__watch(cp, deadline, near=cmd_pos, near_q=cmd_q, min_time=min_time)
            best_seen = max(best_seen, held)
            if not reported and why == 'move_done' and not self.__stop_requested:
                # the action ended but the controller keeps holding the target:
                # give it a short time to settle into level 3
                settle_deadline = min(rospy.get_time() + SETTLE_SEC, hard_deadline)
                reported, held, why = self.__watch(cp, settle_deadline, stop_when_done=False)
                best_seen = max(best_seen, held)
            if reported:
                ok = self.__notify_checkpoint_passed(cid)
                # let the ctl finish its own 3 s hold if it is about to; otherwise the next
                # goal simply preempts this one (the ctl stops/holds ~3 s either way)
                pose = self.__current_pose()
                self.__learn_bias(cp, cmd_pos, pose)
                if pose is not None and v_norm(v_sub(pose[0], cmd_pos)) < 0.09:
                    self.__wait_move_end(min(rospy.get_time() + POST_REPORT_WAIT, hard_deadline), cancel=False)
                # learn the motion speed only from clean legs: a correction attempt is not
                # a speed problem (v1.1 bug: one 5 s correction pushed the factor to x1.54
                # and the planner dropped checkpoints that still fitted)
                if attempt == 0:
                    self.__update_speed(est0, rospy.get_time() - t_leg_start)
                return 3 if ok else 0
            if self.__stop_requested or rospy.get_time() > hard_deadline - 2.0:
                break
            # ---- bias compensation for the next attempt
            pose = self.__current_pose()
            if pose is None:
                continue
            level, det = evaluate_pose(cp, pose[0], pose[1])
            rospy.logwarn('cp %d not level 3 yet: level=%d dist=%.3f center=%s roll=%s',
                          cid, level, det['dist'], _fmt(det['center']), _fmt(det['roll']))
            if why in ('settled', 'move_done'):
                self.__learn_bias(cp, cmd_pos, pose)
            err = v_sub(cp['position'], pose[0])
            old_cmd = list(cmd_pos)
            if v_norm(err) > 0.03:
                cmd_pos = clamp_safe(v_add(cmd_pos, err))
            if cp['marker_pos'] is not None:
                # absolute re-aim from where the robot really is (the real ctl reaches the
                # commanded attitude precisely; a relative correction from a pose taken
                # mid-rotation sent it the wrong way in the first Gazebo run)
                cmd_q = aim_quaternion(pose[0], cp['marker_pos'], cp['target_q'])
            # a new goal preempts the running one: ctl stops and holds ~3 s, then moves, then rotates
            min_time = (PREEMPT_HOLD + move_time(v_norm(v_sub(cmd_pos, old_cmd)))
                        + rot_time(q_angle_between(pose[1], cmd_q)) + 2.0)
            est = min_time + 3.0

        # ---- could not reach level 3: report the best level we can still hold
        if best_seen >= 1 or cp.get('force_report'):
            pose = self.__current_pose()
            level = evaluate_pose(cp, pose[0], pose[1])[0] if pose else 0
            if level >= 1 or cp.get('force_report'):
                ok = self.__notify_checkpoint_passed(cid)
                return level if ok else 0
        rospy.logwarn('cp %d skipped (no valid level)', cid)
        return 0

    def __wait_move_end(self, deadline, cancel=True):
        while not rospy.is_shutdown() and not self.__move_done() and rospy.get_time() < deadline:
            rospy.sleep(POLL_DT)
        if cancel and not self.__move_done():
            self.__cancel_move()

    def __learn_bias(self, cp, cmd_pos, pose):
        """Steady-state offset (actual - commanded) caused by disturbances. The ctl has no
        integral term, so a constant disturbance gives a constant offset we can pre-compensate."""
        if pose is None:
            return
        off = v_sub(pose[0], cmd_pos)
        if v_norm(off) > 0.2:
            return
        self.__bias = v_add(v_scale(self.__bias, 1.0 - BIAS_ALPHA), v_scale(off, BIAS_ALPHA))
        rospy.loginfo('bias estimate (%.3f, %.3f, %.3f)', self.__bias[0], self.__bias[1], self.__bias[2])

    def __update_speed(self, est, actual):
        if est > 1.0 and actual > 0.5:
            ratio = max(0.7, min(2.5, actual / est))
            self.__speed_factor = 0.6 * self.__speed_factor + 0.4 * ratio

    # ------------------------------------------------------------------ flow
    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return
            self.__t0 = rospy.get_time()
            rospy.loginfo('[%s] mission started', PROGRAM_VERSION)

            # USER EDIT AREA: route planning and game-specific flow.
            # ***********************************************
            payload = self.__get_checkpoints()
            if payload is None:
                rospy.logerr('no checkpoint data - finishing')
            else:
                cps, goal_id = self.__parse_checkpoints(payload)
                if goal_id is None or goal_id not in [c['id'] for c in cps]:
                    rospy.logwarn('goal id %s not in checkpoints', goal_id)
                self.__execute(cps, goal_id)
            # ***********************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
            self.__summary()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)
            try:
                if self.__t0 is not None and not rospy.is_shutdown():
                    self.__notify_mission_finished()
            except Exception:
                pass

    def __execute(self, cps, goal_id):
        pose = self.__current_pose()
        wait_until = rospy.get_time() + 5.0
        while pose is None and rospy.get_time() < wait_until and not rospy.is_shutdown():
            rospy.sleep(0.1)
            pose = self.__current_pose()
        if pose is None:
            rospy.logwarn('no navigation yet, assuming dock pose')
            pose = ([0.3, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

        goal_cp = None
        for c in cps:
            if c['id'] == goal_id:
                goal_cp = c
                c['force_report'] = True

        hard_deadline = self.__t0 + MISSION_TIME_LIMIT - FINISH_MARGIN
        done = set()
        while not rospy.is_shutdown() and not self.__stop_requested:
            pose = self.__current_pose() or pose
            todo = [c for c in cps if c['id'] not in done]
            if not todo or (goal_cp is not None and goal_cp['id'] in done):
                break
            # re-plan before EVERY leg with the speed measured so far: checkpoints dropped
            # earlier come back when we are faster than expected, and vice versa
            budget = self.__time_left()
            route, est_total = plan_route(pose[0], pose[1], todo, goal_id, budget, self.__speed_factor)
            if not route:
                break
            rospy.loginfo('plan: %d of %d left, est %.0f s (x%.2f), budget %.0f s: %s',
                          len(route), len(todo), est_total, self.__speed_factor, budget,
                          [c['id'] for c in route])
            cp = route[0]
            if cp is goal_cp or goal_cp is None:
                deadline = hard_deadline
            else:
                # keep enough time for the goal leg
                goal_need = leg_time(cp['cmd_pos'], cp['cmd_q'], goal_cp['cmd_pos'], goal_cp['cmd_q']) \
                    * self.__speed_factor
                deadline = hard_deadline - goal_need
            level = self.__visit(cp, deadline)
            done.add(cp['id'])
            self.__results.append((cp['id'], level, round(self.__elapsed(), 1)))

    def __summary(self):
        total = sum(l for _, l, _ in self.__results)
        rospy.loginfo('SUMMARY elapsed=%.1f s results(id, level, t)=%s level_sum=%d',
                      self.__elapsed(), self.__results, total)

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __stop_processing(self, request):
        try:
            self.__stop_requested = True
            if self.__action_active:
                self.__command.cancel_goal()
            if self.__checkpoint_active:
                self.__checkpoint_client.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


def _fmt(v):
    return 'n/a' if v is None else '%.1f' % v


if __name__ == '__main__':
    UserTemplate().run()
