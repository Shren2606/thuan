import rospy
from std_msgs.msg import String
import subprocess
import os

# Biến toàn cục để lưu trữ các subprocess
sub_processes = []


def start_slam_processes():
    global sub_processes

    # Thiết lập biến môi trường cho ROS
    env = os.environ.copy()
    env['TURTLEBOT3_MODEL'] = 'burger'

    # Bắt đầu các tiến trình SLAM
    slam_process = subprocess.Popen(['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch', 'slam_methods:=gmapping'], env=env)
    gazebo_process = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch'], env=env)

    # Lưu trữ các tiến trình vào danh sách
    sub_processes.extend([slam_process, gazebo_process])

def init_multiple_waypoints():
    global subprocesses
    env = os.environ.copy()
    env['TURTLEBOT3_MODEL'] = 'burger'
    
    gazebo_process = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch'], env=env)
    run_navigation = subprocess.Popen(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch','map_file:=/home/vm/map.yaml'], env=env)
    init_pose = subprocess.Popen(['rosrun', 'auto_nav', 'init_pose.py']) 
    save_key = subprocess.Popen(['rosrun', 'auto_nav', 'keysave_flash.py'])       
    sub_processes.extend([gazebo_process,run_navigation, init_pose,save_key])

def callback(data):
    global sub_processes

    rospy.loginfo("Nhận được thông điệp từ topic /subscriber: %s", data.data)
    if data.data == "1":
        rospy.loginfo("Chạy subprocess...")
        if not sub_processes:
            start_slam_processes()
            #------chay thuc te---------------------
            # process = subprocess.Popen('roslaunch auto_nav slam_hector_savemap.launch', shell=True)
            # sub_processes.append(process)
        else:
            rospy.loginfo("Có subprocess đang chạy, không thể khởi chạy lại subprocess có data là 1")
    elif data.data == "2":
        rospy.loginfo("Chạy subprocess...")
        init_multiple_waypoints()
        #------chay thuc te-------
        #process = subprocess.Popen('roslaunch auto_nav navigation_ttb2.launch', shell=True)
        #sub_processes.append(process)
    elif data.data == "3":
        rospy.loginfo("Chạy subprocess...")
        #--------------chay thuc te------------
        #process = subprocess.Popen('rosrun map_server map_saver -f /home/vm/catkin_ws/src/turtlebot_apps/turtlebot_navigation/maps/map', shell=True)       
        #------------chay mo phong-------------
        process = subprocess.Popen('rosrun map_server map_saver -f ~/map', shell=True)
        sub_processes.append(process)
    elif data.data == "4":
        process = subprocess.Popen('rosrun auto_nav multiple_waypoints_read.py', shell=True)
        sub_processes.append(process)
    elif data.data == "10":
        for proc in sub_processes:
            rospy.loginfo("Chấm dứt subprocess...")
            proc.terminate()
            proc.wait()
        sub_processes.clear()

def listener():
    subprocess.Popen('roscore', shell=True)
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber("/subscriber", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
