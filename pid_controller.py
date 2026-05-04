#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

# ── PID Gains ─────────────────────────────────────────────────────────────────
KP_ANG, KI_ANG, KD_ANG = 2.0, 0.001, 0.3
KP_LIN, KI_LIN, KD_LIN = 0.8, 0.001, 0.1

MAX_LINEAR   = 0.22   # m/s
MAX_ANGULAR  = 2.84   # rad/s
DIST_THRESH  = 0.05  # metres
ANGLE_THRESH = 0.05   # radians

# ── Shared state ──────────────────────────────────────────────────────────────
robot = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
goal  = {'xr': None, 'yr': None, 'thetar': None, 'mode': None, 'active': False}
phase = {'current': 'idle'}

ang_pid = {'integral': 0.0, 'prev_error': 0.0, 'prev_time': None,
           'kp': KP_ANG, 'ki': KI_ANG, 'kd': KD_ANG, 'max': MAX_ANGULAR}
lin_pid = {'integral': 0.0, 'prev_error': 0.0, 'prev_time': None,
           'kp': KP_LIN, 'ki': KI_LIN, 'kd': KD_LIN, 'max': MAX_LINEAR}

cmd_pub = None

# ── Helpers ───────────────────────────────────────────────────────────────────

def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def pid_compute(pid, error, now):
    dt = 0.05 if pid['prev_time'] is None else max((now - pid['prev_time']).to_sec(), 1e-4)
    pid['integral'] += error * dt
    output = (pid['kp'] * error
              + pid['ki'] * pid['integral']
              + pid['kd'] * (error - pid['prev_error']) / dt)
    pid['prev_error'] = error
    pid['prev_time']  = now
    return max(-pid['max'], min(pid['max'], output))

def pid_reset(pid):
    pid['integral']   = 0.0
    pid['prev_error'] = 0.0
    pid['prev_time']  = None

def stop():
    cmd_pub.publish(Twist())

def dist_to_goal():
    return math.hypot(goal['xr'] - robot['x'], goal['yr'] - robot['y'])

def bearing_to_goal():
    return math.atan2(goal['yr'] - robot['y'], goal['xr'] - robot['x'])

# ── Callbacks ─────────────────────────────────────────────────────────────────

def odom_cb(msg):
    robot['x'] = msg.pose.pose.position.x
    robot['y'] = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, robot['yaw'] = euler_from_quaternion([q.x, q.y, q.z, q.w])

def goal_cb(msg):
    if len(msg.data) < 4:
        rospy.logwarn("Reference pose needs 4 values: xr yr thetar mode")
        return
    goal['xr'], goal['yr'], goal['thetar'] = msg.data[0], msg.data[1], msg.data[2]
    goal['mode']   = int(round(msg.data[3]))
    goal['active'] = True
    pid_reset(ang_pid)
    pid_reset(lin_pid)
    phase['current'] = 'rotate_to_goal' if goal['mode'] == 0 else 'drive'
    rospy.loginfo(f"New goal → x={goal['xr']:.2f} y={goal['yr']:.2f} "
                  f"θ={goal['thetar']:.2f} mode={goal['mode']}")

# ── Control loops ─────────────────────────────────────────────────────────────

def run_mode0(now):
    dist      = dist_to_goal()
    angle_err = normalize_angle(bearing_to_goal() - robot['yaw'])
    final_err = normalize_angle(goal['thetar'] - robot['yaw'])
    cmd = Twist()

    if phase['current'] == 'rotate_to_goal':
        if abs(angle_err) < ANGLE_THRESH:
            rospy.loginfo("Facing goal. Starting drive.")
            pid_reset(lin_pid)
            phase['current'] = 'drive'
        else:
            cmd.angular.z = pid_compute(ang_pid, angle_err, now)

    elif phase['current'] == 'drive':
        if dist < DIST_THRESH:
            rospy.loginfo("Position reached. Rotating to final angle.")
            pid_reset(ang_pid)
            phase['current'] = 'rotate_final'
        else:
            cmd.angular.z = pid_compute(ang_pid, angle_err, now)
            cmd.linear.x  = pid_compute(lin_pid, dist, now)

    elif phase['current'] == 'rotate_final':
        if abs(final_err) < ANGLE_THRESH:
            rospy.loginfo("Goal reached (mode 0).")
            stop()
            goal['active']   = False
            phase['current'] = 'idle'
        else:
            cmd.angular.z = pid_compute(ang_pid, final_err, now)

    cmd_pub.publish(cmd)

def run_mode1(now):
    dist      = dist_to_goal()
    final_err = normalize_angle(goal['thetar'] - robot['yaw'])
    cmd = Twist()

    if phase['current'] == 'drive':
        if dist < DIST_THRESH:
            rospy.loginfo("Position reached. Correcting final angle.")
            pid_reset(ang_pid)
            phase['current'] = 'rotate_final'
        else:
            angle_err     = normalize_angle(bearing_to_goal() - robot['yaw'])
            cmd.angular.z = pid_compute(ang_pid, angle_err, now)
            cmd.linear.x  = pid_compute(lin_pid, dist, now)

    elif phase['current'] == 'rotate_final':
        if abs(final_err) < ANGLE_THRESH:
            rospy.loginfo("Goal reached (mode 1).")
            stop()
            goal['active']   = False
            phase['current'] = 'idle'
        else:
            cmd.angular.z = pid_compute(ang_pid, final_err, now)

    cmd_pub.publish(cmd)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global cmd_pub
    rospy.init_node('pid_controller', anonymous=False)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom',           Odometry,           odom_cb)
    rospy.Subscriber('/reference_pose', Float32MultiArray,  goal_cb)

    rospy.loginfo("PID Controller node started.")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if goal['active']:
            now = rospy.Time.now()
            run_mode0(now) if goal['mode'] == 0 else run_mode1(now)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass