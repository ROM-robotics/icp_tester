#!/usr/bin/python3

import os
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion

from rclpy.clock import Clock

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        
    
        pkg = get_package_share_directory('icp_tester')
        # save_file = os.path.join(pkg, 'laser_odom_data.dat')
        
        
        # self.file = open(save_file, "w")
        
        output_file_path = '/ros2_ws/src/developer_packages/icp_tester/test.dat'
        # Open the file for writing
        self.file = open(output_file_path, "w")
        
        self.laser_subscription = self.create_subscription(LaserScan,'/scan',self.laser_callback,10)   # SUBSCRIBER
        self.odom_subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        self.cur_time = self.get_clock().now() #node_time
        self.last_time = self.cur_time

        # scan params
        #scan_no_max = 726 #for simpleURG
        scan_no_max = 352  # (2.3561999797821045*2)/0.013126462697982788 = 352 
        self.scan = [0]*(scan_no_max*2) # 352*2 = 704
        self.scan_no = 0
        # self.angle_min = -2.35619449615 #for simpleURG -135deg
        # self.angle_max = 2.09234976768 #for simpleURG 120deg
        # self.angle_increment = 0.00613592332229 #for simpleURG 0.36deg

        self.angle_min = -2.3561999797821045 #for gazebo lidar
        self.angle_max = 2.3561999797821045 #for 135 deg
        self.angle_increment = 0.013126462697982788 #for simpleURG 0.75209091253218718354 deg

        self.angle_total_n = scan_no_max
        self.scan_time_secs = 0
        self.scan_time_nsecs = 0  

        #odom params

        # self.odom_time_secs 
        # self.odom_time_nsecs

        self.position_x = 0.0           # မလိုတာတွေဖျက်ရန်
        self.position_y = 0.0
        self.position_z = 0.0

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        self.yaw = 0.0
        

    def laser_callback(self, message):
        # self.scan_no = message.header.seq      # ros2 မှာ seq မရှိ

        self.scan_no += 1

        self.scan_time_secs = message.header.stamp.sec
        self.scan_time_nsecs = message.header.stamp.nanosec

        # angle_total_n = 340, scan = 680
        # print(len(message.ranges))
        # print(len(message.intensities))
        for i in range (0, 704-2, 2): 
            #Measurement angle index (DEG)
            self.scan[int(i)] = (self.angle_min + i/2 * self.angle_increment) * 180 / 3.14
            if math.isinf(message.ranges[int(i/2)]):
                self.scan[int((i)+1)] = 0
            else:
                self.scan[int((i)+1)] = message.ranges[(int(i/2))] 
            # if math.isnan(message.ranges[int(i/2)]):                                                        #START FROM HERE        # DOUBLE NO ARRAY ( OUT OF RANGE )
            #     #When the measurement data is nan (outside the measurement range), 0 is assigned
            #     self.scan[int(i)+1] = 0 
            # else:
            #     self.scan[int(i)+1] = message.ranges[int(i/2)] #scan data
            # self.scan[int((i)+1)] = message.ranges[(int(i/2))] 
        # for i in range(0,704,2):
        


    
    def odom_callback(self, message):

        # self.odom_time_secs = message.header.stamp.sec       # odom time stamp ယူရန်
        # self.odom_time_nsecs = message.header.stamp.nanosec

        self.position_x = message.pose.pose.position.x
        self.position_y = message.pose.pose.position.y
        self.position_z = message.pose.pose.position.z

        self.orientation_x = message.pose.pose.orientation.x
        self.orientation_y = message.pose.pose.orientation.y
        self.orientation_z = message.pose.pose.orientation.z
        self.orientation_w = message.pose.pose.orientation.w

        orientation_q = message.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
    )
        
        self.write_data()
    
    # Convert quaternion to Euler angles
        roll, pitch, self.yaw = euler_from_quaternion(quaternion) # yaw ထုတ်


        
        # position = message.pose.pose.position
        # orientation = message.pose.pose.orientation
        # odom_data = (
        #     f"Odometry: Position -> x: {position.x}, y: {position.y}, z: {position.z} "
        #     f"Orientation -> x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}\n"
        # )
        # self.file.write(odom_data)
        # self.file.flush()  # Ensures data is written immediately
    
    def write_data(self):

        if self.scan_no > 0 : #Make the same format as the input data file of SLAM in this book
            output_list =["LASERSCAN"]
            output_list.append(self.scan_no)
            output_list.append(self.cur_time.nanoseconds/1e9)
            output_list.append(self.cur_time.nanoseconds )
            # output_list.append(self.scan_time_secs)     #မူရင်းမှာ scan time မယူ (node time ပဲယူ)
            # output_list.append(self.scan_time_nsecs)
            output_list.append('334') #(352-12) Save scan data from the sensor.
            for i in range (12, 704-24): #Roughly saves scan data from -120deg to 120deg
                output_list.append(self.scan[i])
            output_list.append(self.position_x) #Odometry X direction
            output_list.append(self.position_y) #Odometry Y direction
            output_list.append(self.yaw) #Save with Odometry rotation angle RAD
            output_list.append(self.scan_time_secs)
            output_list.append(self.scan_time_nsecs)  # မူရင်းမှာ odom time stamp မယူထား (scan_time ယူ)

            # output_list.append(1)
            # output_list.append(2.97)     #မူရင်းမှာ scan time မယူ (node time ပဲယူ)
            # output_list.append(432)
            # output_list.append('340') #(723-44)/2 Save scan data from the sensor in half.
            # for i in range (44, 724, 1): #Roughly saves scan data from -120deg to 120deg
            #     output_list.append(4)
            # output_list.append(self.position_x) #Odometry X direction
            # output_list.append(self.position_y) #Odometry Y direction
            # output_list.append(self.yaw) #Save with Odometry rotation angle RAD
            # output_list.append(self.scan_time_secs)
            # output_list.append(self.scan_time_nsecs)  # မူရင်းမှာ odom time stamp မယူထား (scan_time ယူ)
            for d in output_list:
                self.file.write("%s " %d)
            self.file.write("\n")


    def destroy_node(self):
        # Ensure the file is closed when the node is destroyed
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    data_logger_node = DataLoggerNode()
    
    try:
        rclpy.spin(data_logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()