#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign
import PyKDL
from math import pi
import random

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class CalibrateAngular():
    def __init__(self):
        # Give the node a name
        rospy.init_node('calibrate_angular', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)

        self.speed = rospy.get_param('~speed', 0.3) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

        rospy.loginfo("Bring up rqt_reconfigure to control the test.")

        self.performance_test()

    def performance_test(self):
         while not rospy.is_shutdown():
            # Get the current rotation angle from tf
            odom_angle = self.get_odom_angle()

            last_angle = odom_angle
            turn_angle = 0
            test_angle = random.uniform(-3.14,3.14)
            rospy.loginfo('test_angle: %f'%test_angle)
            test_angle = test_angle + odom_angle
            error = test_angle - turn_angle

            r = rospy.Rate(self.rate)
            while abs(error) > self.tolerance:
                if rospy.is_shutdown():
                    return

                # Rotate the robot to reduce the error
                move_cmd = Twist()
                move_cmd.angular.z = copysign(self.speed, error)
                self.cmd_vel.publish(move_cmd)
                r.sleep()

                # Get the current rotation angle from tf                   
                odom_angle = self.get_odom_angle()

                # Compute how far we have gone since the last measurement
                delta_angle = normalize_angle(odom_angle - last_angle)

                # Add to our total angle so far
                turn_angle += delta_angle

                # Compute the new error
                error = test_angle - turn_angle

                # Store the current angle for the next comparison
                last_angle = odom_angle

            # Stop the robot
            self.cmd_vel.publish(Twist())
            rospy.sleep(0.5)

    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))


    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        CalibrateAngular()
    except:
        rospy.loginfo("Calibration terminated.")

