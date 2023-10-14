#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import matplotlib.pyplot as plt
import argparse
import atexit 

class intermediate(Node):
    """
    Create a subscriber node
    """
    def __init__(self, args):
    
        # Initiate the Node class's constructor and give it a name
        super().__init__('intermediate')

        #save args
        self.args = args

        self.gps_poses = []
        self.fused_poses = []
        self.gps_poses_denied = []
        self.manual_predictions = []

        self.gps_denied = False
        self.gps_denied_start = None
        self.gps_denied_end = None

        #publishers
        self.gps_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/intermediate/gps_pose', 10)
        self.gps_orientation_pub = self.create_publisher(PoseWithCovarianceStamped, '/intermediate/gps_orientation', 10)

        #subscribers
        self.gps_pose_sub = self.create_subscription(
        PoseWithCovarianceStamped,
        '/gps_top/pose',
        self.gps_pose_callback,
        qos_profile_sensor_data)

        self.gps_orientation_sub = self.create_subscription(
        PoseWithCovarianceStamped,
        '/gps_top/orientation',
        self.gps_orientation_callback,
        qos_profile_sensor_data)
        
        self.fused_pose_sub = self.create_subscription(
        PoseStamped,
        '/filtered_publisher/pose',
        self.fused_pose_callback,
        qos_profile_sensor_data)

        self.imu_sub = self.create_subscription(
        Imu,
        '/gps_top/imu',
        self.imu_callback,
        qos_profile_sensor_data)

    
    def gps_pose_callback(self, msg):
        time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.gps_poses.append([msg.pose.pose.position.x, msg.pose.pose.position.y,time_ns])
        if self.gps_denied_start is None and self.args.start_time_gps_denied is not None:
            self.gps_denied_start = time_ns + self.args.start_time_gps_denied * 1e9
        if self.gps_denied_end is None and self.args.duration_gps_denied is not None:
            self.gps_denied_end = self.gps_denied_start + self.args.duration_gps_denied * 1e9
        #check if gps denied
        if self.gps_denied_start is not None and self.gps_denied_end is not None:
            time_to_gps_denied = self.gps_denied_start - time_ns
            time_to_gps_denied_end = self.gps_denied_end - time_ns
            self.gps_denied = time_to_gps_denied <= 0 and time_to_gps_denied_end >= 0
            self.get_logger().debug(str('time to gps_denied: ' + str(time_to_gps_denied)))
            self.get_logger().debug(str('time to gps_denied_end: ' + str(time_to_gps_denied_end)))
        else:
            #no gps denied zone
            self.get_logger().debug('No gps denied, set gps_denied_start and gps_denied_end to not None to activate gps denied zone')

        # publish if not gps_denied
        if not self.gps_denied:
            self.gps_pose_pub.publish(msg)
            self.manual_predictions.append([msg.pose.pose.position.x, msg.pose.pose.position.y, time_ns])
        else:
            self.gps_poses_denied.append([msg.pose.pose.position.x, msg.pose.pose.position.y,time_ns])



    def gps_orientation_callback(self, msg):
        #add orientation for performance comparison TODO
        # publish if not gps_denied
        if not self.gps_denied:
            self.gps_orientation_pub.publish(msg)

    def fused_pose_callback(self, msg):
        #save fused_pose for error comparison
        time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.fused_poses.append([msg.pose.position.x, msg.pose.position.y,time_ns])

    def imu_callback(self, msg):
        time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        if self.gps_denied:
            a_x = msg.linear_acceleration.x
            a_y = msg.linear_acceleration.y
            dt_prev = (self.manual_predictions[-1][2] - self.manual_predictions[-2][2]) / 1e9
            dt = (time_ns - self.manual_predictions[-1][2]) / 1e9
            v_x = (self.manual_predictions[-1][0] - self.manual_predictions[-2][0]) / dt_prev
            v_y = (self.manual_predictions[-1][1] - self.manual_predictions[-2][1]) / dt_prev
            predicted_x = self.manual_predictions[-1][0] + v_x * dt + 0.5 * a_x * dt**2
            predicted_y = self.manual_predictions[-1][1] + v_y * dt + 0.5 * a_y * dt**2

            self.manual_predictions.append([predicted_x, predicted_y, time_ns])


    def plot_results(self):
        if not gps_poses:
            self.get_logger().info('Intermediate node did not receive any gps data, aborting plot.')
            return
        gps_poses = np.array(self.gps_poses)
        fused_poses = np.array(self.fused_poses)
        gps_poses_denied = np.array(self.gps_poses_denied)
        manual_predictions = np.array(self.manual_predictions)

        #compute errors between fused and gps when time is the closest
        errors = []
        for i in range(len(fused_poses)):
            #find closest gps pose
            closest_index = np.argmin(np.abs(gps_poses[:,2] - fused_poses[i][2]))
            errors.append(np.linalg.norm(fused_poses[i][:2] - gps_poses[closest_index][:2]))
        errors = np.array(errors)
        self.get_logger().info('Error analysis for entire trajectory:')
        self.get_logger().info('Mean error: ' + str(np.mean(errors)))
        self.get_logger().info('Max error: ' + str(np.max(errors)))
        self.get_logger().info('Min error: ' + str(np.min(errors)))
        self.get_logger().info('Std error: ' + str(np.std(errors)))

        #Error analysis for gps denied zone
        if len(gps_poses_denied) > 0:
            errors = []
            for i in range(len(gps_poses_denied)):
                #find closest fused pose
                closest_index = np.argmin(np.abs(fused_poses[:,2] - gps_poses_denied[i][2]))
                errors.append(np.linalg.norm(fused_poses[closest_index][:2] - gps_poses_denied[i][:2]))
            errors = np.array(errors)
            self.get_logger().info('Error analysis for gps denied zone:')
            self.get_logger().info('Mean error: ' + str(np.mean(errors)))
            self.get_logger().info('Max error: ' + str(np.max(errors)))
            self.get_logger().info('Min error: ' + str(np.min(errors)))
            self.get_logger().info('Std error: ' + str(np.std(errors)))
                

        if self.args.show_plot or self.args.save_plot is not None:
        #once the node is shutdown, plot/save the data
            #plot the data
            figs, axs = plt.subplots(2,1)
            axs[0].plot(gps_poses[:,0], gps_poses[:,1], label='gps')
            axs[0].plot(fused_poses[:,0], fused_poses[:,1], label='fused')
            axs[0].plot(manual_predictions[:,0], manual_predictions[:,1], label='manual prediction')
            axs[0].axvline(x=gps_poses_denied[0][0], color='r', linestyle='--', label='gps denied')
            axs[0].axvline(x=gps_poses_denied[-1][0], color='r', linestyle='--')
            axs[0].set_title('Full Trajectory')
            #axs[0].axis('equal')
            axs[0].legend()
            #zoom on gps denied zone
            #all i such that fused_poses[i][2] is superior or equal to gps_denied[0][2] and inferior or equal to gps_denied[-1][2]
            fused_poses_in_gps_denied = fused_poses[(fused_poses[:,2] >= gps_poses_denied[0][2]) & (fused_poses[:,2] <= gps_poses_denied[-1][2])]
            manual_predictions_in_gps_denied = manual_predictions[(manual_predictions[:,2] >= gps_poses_denied[0][2]) & (manual_predictions[:,2] <= gps_poses_denied[-1][2])]
            axs[1].plot(gps_poses_denied[:,0], gps_poses_denied[:,1], label='gps')
            axs[1].plot(fused_poses_in_gps_denied[:,0], fused_poses_in_gps_denied[:,1], label='fused')
            axs[1].plot(manual_predictions_in_gps_denied[:,0], manual_predictions_in_gps_denied[:,1], label='manual prediction')
            axs[1].set_title('Zoom on GPS denied zone')
            #axs[1].axis('equal')
            axs[1].legend()
            #tight layout
            figs.tight_layout()

            if self.args.save_plot is not None:
                name = self.args.save_plot
                if not name.endswith('.png'):
                    name += '.png'
                figs.savefig(name)
            if self.args.show_plot:
                plt.show()
            self.get_logger().info('Plot done!')

 
def main(args=None):

    parser = argparse.ArgumentParser(description='Intermediate Node')
    parser.add_argument('--start_time_gps_denied', '-s', type=int, default=30, help='Starting time for gps denied in seconds after first gps message. None for no gps denied. Default: None.')
    parser.add_argument('--duration_gps_denied', '-d', type=int, default=5, help='Duration for gps denied in seconds after starting gps denied zone. None for no gps denied. Default: None.')
    parser.add_argument('--show_plot', type=bool, default=True, help='Show plot after execution. Default: True.')
    parser.add_argument('--save_plot', type=str, default='results_localization_test', help="Specifiy the file name to save the plot. Use 'None' to not save the plot. Default: None.")
    parser.add_argument('--manual_prediction', type=bool, default=True, help="Compute and plot manual prediction for comparison. Default: False.")
    
    #don't mix up with --ros-args
    args_node = parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])

    assert args_node.duration_gps_denied is None or args_node.duration_gps_denied > 0, "Duration of gps denied must be positive"

    print("Starting intermediate node to publish/hide gps data and evaluate sensor fusion")
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create a subscriber
    intermediate_node = intermediate(args_node)

    atexit.register(intermediate_node.plot_results)
    
    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    rclpy.spin(intermediate_node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    intermediate_node.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
