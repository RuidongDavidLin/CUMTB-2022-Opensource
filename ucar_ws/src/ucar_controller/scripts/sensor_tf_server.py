#!/usr/bin/env python  

import rospy
from ucar_controller.srv import *
import threading
import tf

class SensorTFServer:
    def __init__(self):
        rospy.init_node('SensorTFServer', anonymous=True)

        self.camera_pose_x  = rospy.get_param('~camera_pose_x', 0.15)
        self.camera_pose_y  = rospy.get_param('~camera_pose_y', 0.0)
        self.camera_pose_z  = rospy.get_param('~camera_pose_z', 0.15)
        self.camera_euler_r = rospy.get_param('~camera_euler_r', -3.14159 * 115.0/180.0)
        self.camera_euler_p = rospy.get_param('~camera_euler_p', 0.0)
        self.camera_euler_y = rospy.get_param('~camera_euler_y', -3.14159 * 90.0/180.0)

        self.lidar_pose_x  = rospy.get_param('~lidar_pose_x',  -0.11)
        self.lidar_pose_y  = rospy.get_param('~lidar_pose_y',  0.0)
        self.lidar_pose_z  = rospy.get_param('~lidar_pose_z',  0.165)
        self.lidar_euler_r = rospy.get_param('~lidar_euler_r', 0.0)
        self.lidar_euler_p = rospy.get_param('~lidar_euler_p', 0.0)
        self.lidar_euler_y = rospy.get_param('~lidar_euler_y', -0.07)

        self.imu_pose_x  = rospy.get_param('~imu_pose_x',  0.05)
        self.imu_pose_y  = rospy.get_param('~imu_pose_y', -0.05)
        self.imu_pose_z  = rospy.get_param('~imu_pose_z',  0.05)
        self.imu_euler_r = rospy.get_param('~imu_euler_r', 3.14159)
        self.imu_euler_p = rospy.get_param('~imu_euler_p', 0.0)
        self.imu_euler_y = rospy.get_param('~imu_euler_y', 0.0)

        self.camera_frame = rospy.get_param("~camera_frame", "cam")
        self.lidar_frame  = rospy.get_param("~lidar_frame" , "lidar")
        self.imu_frame    = rospy.get_param("~imu_frame"   , "imu")
        self.base_frame   = rospy.get_param("~base_frame"  , "base_link")

        self.tf_rate      = rospy.get_param("~tf_rate"   , 30.0)

        self.set_camera_tf_service = rospy.Service('set_camera_tf', SetSensorTF, self.setCameraTFCB)
        self.set_lidar_tf_service  = rospy.Service('set_lidar_tf' , SetSensorTF, self.setLidarTFCB)
        self.set_imu_tf_service    = rospy.Service('set_imu_tf'   , SetSensorTF, self.setIMUTFCB)
        self.get_camera_tf_service = rospy.Service('get_camera_tf', GetSensorTF, self.getCameraTFCB)
        self.get_lidar_tf_service  = rospy.Service('get_lidar_tf' , GetSensorTF, self.getLidarTFCB)
        self.get_imu_tf_service    = rospy.Service('get_imu_tf'   , GetSensorTF, self.getIMUTFCB)
        self.tf_br = tf.TransformBroadcaster()
        
        send_tf_thread = threading.Thread(target=self.sendTFThread, name='T1')
        send_tf_thread.start()

        print("SensorTFServer ready.")
        rospy.spin()

    def setCameraTFCB(self,req):
        self.camera_pose_x  = req.pose_x
        self.camera_pose_y  = req.pose_y
        self.camera_pose_z  = req.pose_z
        self.camera_euler_r = req.euler_r
        self.camera_euler_p = req.euler_p
        self.camera_euler_y = req.euler_y
        return SetSensorTFResponse(success = True, message = 'set camera tf success')

    def getCameraTFCB(self,req):
        response = GetSensorTFResponse()
        response.pose_x  = self.camera_pose_x
        response.pose_y  = self.camera_pose_y
        response.pose_z  = self.camera_pose_z
        response.euler_r = self.camera_euler_r
        response.euler_p = self.camera_euler_p
        response.euler_y = self.camera_euler_y
        return response
    
    def setLidarTFCB(self,req):
        self.lidar_pose_x  = req.pose_x
        self.lidar_pose_y  = req.pose_y
        self.lidar_pose_z  = req.pose_z
        self.lidar_euler_r = req.euler_r
        self.lidar_euler_p = req.euler_p
        self.lidar_euler_y = req.euler_y
        return SetSensorTFResponse(success = True, message = 'set camera tf success')

    def getLidarTFCB(self,req):
        response = GetSensorTFResponse()
        response.pose_x  = self.lidar_pose_x
        response.pose_y  = self.lidar_pose_y
        response.pose_z  = self.lidar_pose_z
        response.euler_r = self.lidar_euler_r
        response.euler_p = self.lidar_euler_p
        response.euler_y = self.lidar_euler_y
        return response

    def setIMUTFCB(self,req):
        self.imu_pose_x  = req.pose_x
        self.imu_pose_y  = req.pose_y
        self.imu_pose_z  = req.pose_z
        self.imu_euler_r = req.euler_r
        self.imu_euler_p = req.euler_p
        self.imu_euler_y = req.euler_y
        return SetSensorTFResponse(success = True, message = 'set camera tf success')

    def getIMUTFCB(self,req):
        response = GetSensorTFResponse()
        response.pose_x  = self.imu_pose_x
        response.pose_y  = self.imu_pose_y
        response.pose_z  = self.imu_pose_z
        response.euler_r = self.imu_euler_r
        response.euler_p = self.imu_euler_p
        response.euler_y = self.imu_euler_y
        return response

    def sendTFThread(self):
        rate = rospy.Rate(self.tf_rate)
        lock = threading.Lock()   
        while not rospy.is_shutdown():
            lock.acquire()
            # send camera tf
            self.tf_br.sendTransform((self.camera_pose_x, self.camera_pose_y, self.camera_pose_z),
                        tf.transformations.quaternion_from_euler(self.camera_euler_r, self.camera_euler_p, self.camera_euler_y),
                        rospy.Time.now(),
                        self.camera_frame,
                        self.base_frame)
            # send lidar tf
            self.tf_br.sendTransform((self.lidar_pose_x, self.lidar_pose_y, self.lidar_pose_z),
                        tf.transformations.quaternion_from_euler(self.lidar_euler_r, self.lidar_euler_p, self.lidar_euler_y),
                        rospy.Time.now(),
                        self.lidar_frame,
                        self.base_frame)
            # send imu tf
            self.tf_br.sendTransform((self.imu_pose_x, self.imu_pose_y, self.imu_pose_z),
                        tf.transformations.quaternion_from_euler(self.imu_euler_r, self.imu_euler_p, self.imu_euler_y),
                        rospy.Time.now(),
                        self.imu_frame,
                        self.base_frame)
            lock.release()
            rate.sleep()


if __name__ == '__main__':
    server = SensorTFServer()
