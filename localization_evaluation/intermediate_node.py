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


class area:
    def __init__(self, cx, cy, w, h):
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
    
    def is_inside(self, x, y):
        return self.cx - self.w/2 <= x <= self.cx + self.w/2 and self.cy - self.h/2 <= y <= self.cy + self.h/2


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
        self.fused_poses_gps_denied = []
        self.manual_predictions = []
        self.manual_predictions_gps_denied = []

        self.count_zones = 0
        self.was_gps_denied = False

        self.gps_denied = False
        self.gps_denied_time_intervals = None
        self.time_first_gps = None

        self.gps_denied_areas = None
        if self.args.areas_gps_denied is not None:
            self.gps_denied_areas = [area(self.args.areas_gps_denied[i], self.args.areas_gps_denied[i+1], self.args.areas_gps_denied[i+2], self.args.areas_gps_denied[i+3]) for i in range(0, len(self.args.areas_gps_denied), 4) ]

        if args.random_gps_denied:
            self.last_random_gps_denied_end = 0

        #publishers
        self.gps_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/intermediate/gps_pose', 10)
        self.gps_orientation_pub = self.create_publisher(PoseWithCovarianceStamped, '/intermediate/gps_orientation', 10)

        #subscribers
        self.gps_pose_sub = self.create_subscription(
        PoseWithCovarianceStamped,
        '/gps_bot/pose',
        self.gps_pose_callback,
        qos_profile_sensor_data)

        self.gps_orientation_sub = self.create_subscription(
        PoseWithCovarianceStamped,
        '/gps_bot/orientation',
        self.gps_orientation_callback,
        qos_profile_sensor_data)
        
        self.fused_pose_sub = self.create_subscription(
        PoseStamped,
        '/filtered_publisher/pose',
        self.fused_pose_callback,
        qos_profile_sensor_data)

        self.imu_sub = self.create_subscription(
        Imu,
        '/gps_bot/imu',
        self.imu_callback,
        qos_profile_sensor_data)
    
    def log(self, msg):
        if self.args.verbose:
            self.get_logger().info(msg)
        else:
            self.get_logger().debug(msg)
    
    def check_area(self, x, y, area):
        #check if point (x, y) is inside area
        #area is a list of 4 points (x, y)
        #area must be clockwise
        #starting from upper left corner
        inside = x 

    
    def gps_pose_callback(self, msg):
        time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.gps_poses.append([msg.pose.pose.position.x, msg.pose.pose.position.y,time_ns])
        if self.time_first_gps is None:
            self.time_first_gps = time_ns
            if self.args.random_gps_denied and self.last_random_gps_denied_end==0:
                self.last_random_gps_denied_end = time_ns

        ## manual gps denied zones based on TIME (done here and not in init to have time_first_gps)
        if self.gps_denied_time_intervals is None and self.args.time_interval_gps_denied is not None:
            self.gps_denied_time_intervals = [[time_ns + self.args.time_interval_gps_denied[i] * 1e9, time_ns + self.args.time_interval_gps_denied[i+1] * 1e9] for i in range(0, len(self.args.time_interval_gps_denied), 2)]
            #self.gps_poses_denied list of n enpty lists
            self.gps_poses_denied = [[] for i in range(len(self.gps_denied_time_intervals))]
            self.fused_poses_gps_denied = [[] for i in range(len(self.gps_denied_time_intervals))]
            self.manual_predictions_gps_denied = [[] for i in range(len(self.gps_denied_time_intervals))]
        
        ## random gps denied zones
        elif self.args.random_gps_denied:
            #probability growing exponentially with time since last gps denied zone
            delta_t = (time_ns - self.last_random_gps_denied_end) / 1e9
            prob = 1 - np.exp(-delta_t / 1000)
            if np.random.rand() < prob:
                #create a new gps denied zone of random length between 0.5 and 2 seconds
                self.last_random_gps_denied_start = time_ns
                self.last_random_gps_denied_end = time_ns + (np.random.rand() * 1.5 + 0.5) * 1e9
                self.gps_denied_time_intervals = [[self.last_random_gps_denied_start, self.last_random_gps_denied_end]]
                self.log('Random gps denied zone created from ' + str((self.last_random_gps_denied_start-self.time_first_gps)/1e9) + ' to ' + str((self.last_random_gps_denied_end-self.time_first_gps)/1e9) + ', current time: ' + str((time_ns-self.time_first_gps)/1e9))
                self.gps_poses_denied.append([])
                self.fused_poses_gps_denied.append([])
                self.manual_predictions_gps_denied.append([])


        
        ## check if gps denied based on TIME
        if self.gps_denied_time_intervals is not None:
            for interval in self.gps_denied_time_intervals:
                if interval[0] <= time_ns <= interval[1]:
                    self.gps_denied = True
                    if not self.was_gps_denied:
                        self.count_zones += 1
                        self.count_zones = min(self.count_zones, len(self.gps_poses_denied))
                        self.was_gps_denied = True
                        self.log('GPS denied zone ' + str(self.count_zones) + ' detected from ' + str((interval[0]-self.time_first_gps)/1e9) + ' to ' + str((interval[1]-self.time_first_gps)/1e9) + ', current time: ' + str((time_ns-self.time_first_gps)/1e9))
                    break
                else:
                    self.gps_denied = False

        ## check if gps denied based on AREA
        elif self.gps_denied_areas is not None:
            for area in self.gps_denied_areas:
                if area.is_inside(msg.pose.pose.position.x, msg.pose.pose.position.y):
                    self.gps_denied = True
                    if not self.was_gps_denied:
                        self.count_zones += 1
                        self.gps_poses_denied.append([])
                        self.fused_poses_gps_denied.append([])
                        self.manual_predictions_gps_denied.append([])
                        self.count_zones = min(self.count_zones, len(self.gps_poses_denied))
                        self.was_gps_denied = True
                        self.log('GPS denied zone ' + str(self.count_zones) + ' detected in area centered on (' + str(area.cx) + ', ' + str(area.cy) + ') of size ' + str(area.w) + 'x' + str(area.h) + ', current time: ' + str((time_ns-self.time_first_gps)/1e9))
                    break
                else:
                    self.gps_denied = False
            

        elif not self.args.random_gps_denied:
            #no gps denied zone
            self.log('No gps denied, use --time_interval_gps_denied or --random_gps_denied to create gps denied zones')
        
        # publish if not gps_denied
        if not self.gps_denied:
            self.gps_pose_pub.publish(msg)
            self.manual_predictions.append([msg.pose.pose.position.x, msg.pose.pose.position.y, time_ns])
            if self.was_gps_denied:
                self.log('Not gps denied, publishing gps pose, current time: ' + str((time_ns-self.time_first_gps)/1e9))
            self.was_gps_denied = False
        else:
            self.gps_poses_denied[self.count_zones-1].append([msg.pose.pose.position.x, msg.pose.pose.position.y, time_ns])



    def gps_orientation_callback(self, msg):
        #add orientation for performance comparison TODO
        # publish if not gps_denied
        if not self.gps_denied:
            self.gps_orientation_pub.publish(msg)

    def fused_pose_callback(self, msg):
        #save fused_pose for error comparison
        time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.fused_poses.append([msg.pose.position.x, msg.pose.position.y,time_ns])
        if self.gps_denied:
            self.fused_poses_gps_denied[self.count_zones-1].append([msg.pose.position.x, msg.pose.position.y, time_ns])

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
            self.manual_predictions_gps_denied[self.count_zones-1].append([predicted_x, predicted_y, time_ns])

    def plot_results(self):
        if not self.gps_poses:
            self.get_logger().info('Intermediate node did not receive any gps data, aborting plot.')
            return
        gps_poses = np.array(self.gps_poses)
        fused_poses = np.array(self.fused_poses)
        gps_poses_denied = [np.array(zone) for zone in self.gps_poses_denied]
        fused_poses_gps_denied = [np.array(zone) for zone in self.fused_poses_gps_denied]
        manual_predictions = np.array(self.manual_predictions)
        manual_predictions_gps_denied = [np.array(zone) for zone in self.manual_predictions_gps_denied]

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
                #zone i+1
                for j in range(len(gps_poses_denied[i])):
                    #find closest fused pose for each gps pose in the zone
                    closest_index = np.argmin(np.abs(fused_poses_gps_denied[i][:,2] - gps_poses_denied[i][j][2]))
                    errors.append(np.linalg.norm(fused_poses_gps_denied[i][closest_index][:2] - gps_poses_denied[i][j][:2]))
            errors = np.array(errors)
            self.get_logger().info('Error analysis for gps denied zone:')
            self.get_logger().info('Mean error: ' + str(np.mean(errors)))
            self.get_logger().info('Max error: ' + str(np.max(errors)))
            self.get_logger().info('Min error: ' + str(np.min(errors)))
            self.get_logger().info('Std error: ' + str(np.std(errors)))
                

        if self.args.show_plot or self.args.save_plot is not None:
        #once the node is shutdown, plot/save the data
            #plot the data
            n = len(gps_poses_denied)
            #n+1 subplots, use large figsize to have enough space between subplots
            figs, axs = plt.subplots(n+1, 1, figsize=(10, 5*(n+1)))


            axs[0].plot(gps_poses[:,0], gps_poses[:,1], label='gps')
            axs[0].plot(fused_poses[:,0], fused_poses[:,1], label='fused')
            axs[0].plot(manual_predictions[:,0], manual_predictions[:,1], label='manual prediction')
            # axs[0].axvline(x=gps_poses_denied[0][0], color='r', linestyle='--', label='gps denied')
            # axs[0].axvline(x=gps_poses_denied[-1][0], color='r', linestyle='--')
            axs[0].set_title('Full Trajectory')
            axs[0].set_xlabel('x [m]')
            axs[0].set_ylabel('y [m]')
            #axs[0].axis('equal')
            axs[0].legend()
            #zoom on gps denied zone
            for k in range(n):
                axs[k+1].plot(gps_poses_denied[k][:,0], gps_poses_denied[k][:,1], label='gps')
                axs[k+1].plot(fused_poses_gps_denied[k][:,0], fused_poses_gps_denied[k][:,1], label='fused')
                axs[k+1].plot(manual_predictions_gps_denied[k][:,0], manual_predictions_gps_denied[k][:,1], label='manual prediction')
                axs[k+1].set_title('Zoom on GPS denied zone '+str(k+1))
                axs[k+1].set_xlabel('x [m]')
                axs[k+1].set_ylabel('y [m]')
                axs[k+1].legend()
            #tight layout
            figs.tight_layout()

            if self.args.save_plot is not None:
                name = self.args.save_plot
                if not name.endswith('.png'):
                    name += '.png'
                figs.savefig(name)
            if self.args.show_plot:
                plt.tight_layout()
                plt.show()
            self.get_logger().info('Plot done!')

 
def main(args=None):

    parser = argparse.ArgumentParser(description='Intermediate Node')
    parser.add_argument('--time_interval_gps_denied', '-t', nargs='+' , type=int, default=None, help='List of time intervals (relative to first gps message received) in seconds where gps is denied. Example: -t 30 60 90 120 will create 2 gps denied zones: 30s to 60s and 90s to 120s. Default: None.')
    parser.add_argument('--areas_gps_denied', '-a', nargs='+' , type=float, default=None, help='List of areas with center coordinates, widht and height x1 y1 w h .... (in meters) where gps is denied. Make sure to start Example: -a 0 0 10 10 will create a gps denied zone of 10x10m centered on (0,0). Default: None.')
    parser.add_argument('--random_gps_denied', type=bool, default=False, help='Randomly set gps denied zones. Default: False.')
    parser.add_argument('--show_plot', type=bool, default=True, help='Show plot after execution. Default: True.')
    parser.add_argument('--save_plot', type=str, default='results_localization_test', help="Specifiy the file name to save the plot. Use 'None' to not save the plot. Default: None.")
    parser.add_argument('--manual_prediction', type=bool, default=True, help="Compute and plot manual prediction for comparison. Default: False.")
    parser.add_argument('--verbose', type=bool, default=True, help="Print debug messages. Default: True.")
    
    #don't mix up with --ros-args
    args_node = parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])

    #check time_interval_gps_denied is a list of even length or None
    assert args_node.time_interval_gps_denied is None or len(args_node.time_interval_gps_denied) % 2 == 0, 'time_interval_gps_denied must be a list of even length or None'

    #check areas_gps_denied is a list of multiples of 4 elements or None
    assert args_node.areas_gps_denied is None or len(args_node.areas_gps_denied) == 4, 'areas_gps_denied must be a list of k*4 elements or None'

    #check that only one of time_interval_gps_denied, areas_gps_denied and random_gps_denied is used
    assert (args_node.time_interval_gps_denied is None) + (args_node.areas_gps_denied is None) + (not args_node.random_gps_denied) == 2, 'Only one of time_interval_gps_denied, areas_gps_denied and random_gps_denied can be used'

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
