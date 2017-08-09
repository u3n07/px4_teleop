#!/usr/bin/env python

import curses
import rospy
import mavros

from mavros import setpoint

from geometry_msgs.msg import PoseStamped

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandBoolRequest

from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest

from mavros_msgs.msg import State

current_state = State()

def addstrln(scr, string, posx=0, posy=0):
    scr.addstr(posx, posy, string)
    scr.insertln()


def show_key_config(scr):
    addstrln(scr,"ctrl-c: quit")
    addstrln(scr,"right arrow: turn right")
    addstrln(scr,"left arrow: turn left")
    addstrln(scr,"up arrow: upward")
    addstrln(scr,"down arrow: downward")
    addstrln(scr,"d: right")
    addstrln(scr,"s: backward")
    addstrln(scr,"a: left")
    scr.addstr(0,0,"w: forward")


def set_twist_stamped(msg, stamp, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_z=0.0):
    msg.header.stamp = stamp
    msg.twist.linear.x = lin_x
    msg.twist.linear.y = lin_y
    msg.twist.linear.z = lin_z
    msg.twist.angular.z = ang_z


def publish_velocity(pub, msg, key):
    if key==119:
        # w
        set_twist_stamped(msg,stamp=rospy.Time.now(),lin_x=10.0)
        pub.publish(msg)
    elif key==97:
        # a
        set_twist_stamped(msg,stamp=rospy.Time.now(),lin_y=10.0)
        pub.publish(msg)
    elif key==115:
        # s
        set_twist_stamped(msg,stamp=rospy.Time.now(),lin_x=-10.0)
        pub.publish(msg)
    elif key==100:
        # d
        set_twist_stamped(msg,stamp=rospy.Time.now(),lin_y=-10.0)
        pub.publish(msg)
    elif key==260:
        # <-
        set_twist_stamped(msg,stamp=rospy.Time.now(),ang_z=10.0)
        pub.publish(msg)
    elif key==259:
        # up arrow
        set_twist_stamped(msg,stamp=rospy.Time.now(),lin_z=10.0)
        pub.publish(msg)
    elif key==261:
        # ->
        set_twist_stamped(msg,stamp=rospy.Time.now(),ang_z=-10.0)
        pub.publish(msg)
    elif key==258:
        # down arrow
        set_twist_stamped(msg,stamp=rospy.Time.now(),lin_z=-10.0)
        pub.publish(msg)


def state_cb(msg):
    current_state = msg


def px4_teleop_key():
    rospy.init_node("px4_teleop_key_pub", anonymous=True)
    rospy.loginfo("Node Initialized")

    mavros.set_namespace()

    state_sub = rospy.Subscriber("mavros/state", State, state_cb, queue_size=10)
    vel_teleop_pub = setpoint.get_pub_velocity_cmd_vel(queue_size=10)

    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.loginfo("Subscriber, Publisher and Service Clients are initilized")

    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown() and current_state.connected:
        rate.sleep()

    set_mode_req = SetModeRequest()
    set_mode_req.custom_mode = "OFFBOARD"

    arm_cmd_req = CommandBoolRequest()
    arm_cmd_req.value = True

    while not set_mode_client(set_mode_req).success:
        pass

    rospy.loginfo("Offboard enabled")
        
    while not arming_client(arm_cmd_req).success:
        pass

    rospy.loginfo("Vehicle armed")

    try:
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(1)
        show_key_config(stdscr)

        while not rospy.is_shutdown():
            if not current_state.mode=="OFFBOARD":
               set_mode_client(set_mode_req)
            op = stdscr.getch()
            vel_teleop_msg = TwistStamped()
            publish_velocity(vel_teleop_pub, vel_teleop_msg, op)
            rate.sleep()

    finally:
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        curses.endwin()

if __name__=="__main__":
    try:
        px4_teleop_key()
    except rospy.ROSInterruptException:
        pass
