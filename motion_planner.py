#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

DIST_THRESH  = 0.1   # metres
ANGLE_THRESH = 0.1   # radians

robot = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

pose_pub = None

# ── Helpers ───────────────────────────────────────────────────────────────────

def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def goal_reached(xr, yr, thetar):
    ep = math.hypot(xr - robot['x'], yr - robot['y'])
    et = abs(normalize_angle(thetar - robot['yaw']))
    return ep <= DIST_THRESH and et <= ANGLE_THRESH

# ── Callback ──────────────────────────────────────────────────────────────────

def odom_cb(msg):
    robot['x'] = msg.pose.pose.position.x
    robot['y'] = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, robot['yaw'] = euler_from_quaternion([q.x, q.y, q.z, q.w])

# ── User input & publish ──────────────────────────────────────────────────────

def get_user_input():
    print("\n--- Enter reference pose ---")
    try:
        xr     = float(input("  x_r  (metres)  : "))
        yr     = float(input("  y_r  (metres)  : "))
        thetar = float(input("  θ_r  (radians) : "))
        mode   = int(input("  mode (0 or 1)  : "))
        if mode not in (0, 1):
            print("  /!\ Mode must be 0 or 1.")
            return None
        return xr, yr, thetar, mode
    except ValueError:
        print("  /!\ Invalid input — numbers only.")
        return None

def publish_goal(xr, yr, thetar, mode):
    msg = Float32MultiArray()
    msg.data = [float(xr), float(yr), float(thetar), float(mode)]
    pose_pub.publish(msg)
    rospy.loginfo(f"Published goal → x={xr:.2f} y={yr:.2f} θ={thetar:.2f} mode={mode}")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global pose_pub
    rospy.init_node('motion_planner', anonymous=False)
    pose_pub = rospy.Publisher('/reference_pose', Float32MultiArray, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.sleep(1.0)
    rospy.loginfo("Motion Planner node started.")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        result = get_user_input()
        if result is None:
            continue

        xr, yr, thetar, mode = result
        publish_goal(xr, yr, thetar, mode)
        print(f"\n  Driving to ({xr:.2f}, {yr:.2f}, {thetar:.2f}) [mode={mode}] …")

        while not rospy.is_shutdown():
            if goal_reached(xr, yr, thetar):
                ep = math.hypot(xr - robot['x'], yr - robot['y'])
                et = abs(normalize_angle(thetar - robot['yaw']))
                print(f"\n Goal reached!  ep={ep:.4f} m  eθ={et:.4f} rad")
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass