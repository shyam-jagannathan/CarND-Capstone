#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32
import math, sys
import yaml
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SPEED = 10*0.447 #8.94 #16 #20*0.447 

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)

        # The /base_waypoints topic repeatedly publishes a list of all waypoints for the track, 
        # so this list includes waypoints both before and after the vehicle. 
        self.base_waypoints_sub=rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # useful parameters from other notes (for future work)
        self.max_dec = abs(rospy.get_param('/dbw_node/decel_limit'))
        self.max_acc = rospy.get_param('/dbw_node/accel_limit')
        config_string = rospy.get_param("/traffic_light_config") 
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        #rospy.loginfo(self.stop_line_positions)

        self.current_pose = None
        self.waypoints = None
        self.traffic_light_red_waypoint = -1
        self.previous_closest_waypoint_index = None

        self.tf_listener = tf.TransformListener()
        self.default_velocity = rospy.get_param('~velocity', 1) * .44
        rospy.loginfo(self.default_velocity)


        rospy.spin()

    def loop(self):
        next_wps = []
        update_velocity = 0
        
        if self.waypoints is None or self.current_pose is None:
            return

        start = 0
        end = len(self.waypoints)
        #rospy.loginfo(len(self.waypoints)) # 10902

        #just to avoid overshooting/undershooting (we will fix this later)
        if self.previous_closest_waypoint_index is not None:
            start = self.previous_closest_waypoint_index - 20
            end = self.previous_closest_waypoint_index + 20
            #rospy.loginfo(end)
            # Fixes crash issue at end of lap
            if (end>len(self.waypoints)-1):
                rospy.loginfo(end)
                start = 0
                end = len(self.waypoints)
                #self.set_waypoint_velocity(next_wps, i, 0.0)
            

        # Closest waypoint to our vechicle
        closest_waypoint_loc= self.get_closet_waypoint_index(self.waypoints,self.current_pose, start,end)
        
        closest_waypoint = self.waypoints[closest_waypoint_loc]
        current_velocity = self.get_waypoint_velocity(closest_waypoint)
        self.previous_closest_waypoint_index = closest_waypoint_loc 

        # Create a lookahead points sized list for final waypoints    
        for i in range (closest_waypoint_loc, closest_waypoint_loc + LOOKAHEAD_WPS):
            # Handle wrap around
            index = i % len (self.waypoints)
            wpi = self.waypoints[index]
            next_wps.append (wpi)
        
        waypoint_distance_to_traffic_light = self.traffic_light_red_waypoint - closest_waypoint_loc
        upcoming_red_light = (self.traffic_light_red_waypoint != -1 and waypoint_distance_to_traffic_light < 75)
        #rospy.loginfo(waypoint_distance_to_traffic_light)
        #rospy.loginfo(upcoming_red_light)


        # This section is still in WIP
        for i in range(len(next_wps) - 1):
 
            if not upcoming_red_light or i > self.traffic_light_red_waypoint:
                self.set_waypoint_velocity(next_wps, i, MAX_SPEED)
            
            # 2 simple methods to slow down car (we will choose one of these after TL detector is ready and tested)
            # Method 1: Stop line method
            else:
                stop_line_distance = self.find_stop_line(self.traffic_light_red_waypoint,self.lane)
                waypoint_remaining = self.traffic_light_red_waypoint - closest_waypoint_loc - i
                # Check if car can cross the signal based on current velocity and its current distance to stop line
                not_safe_to_cross = (current_velocity/stop_line_distance < 2 and stop_line_distance >= 4)
                rospy.loginfo(stop_line_distance)
                
                if not_safe_to_cross:
                    #update_velocity = MAX_SPEED - (75 - waypoint_remaining)*(MAX_SPEED/75)
                    update_velocity=min(max(abs(stop_line_distance*0.2),0.0),current_velocity)
                elif stop_line_distance < 2: #waypoint_remaining < 6: #Maintain a safe distance
                        update_velocity = 0.0
                else:
                    update_velocity = MAX_SPEED - (75 - waypoint_remaining)*(MAX_SPEED/75)
                new_velocity = max(0.0, update_velocity)
                self.set_waypoint_velocity(next_wps, i, new_velocity)

            # Method 2: Waypoint method    
            #else:
                # waypoint_remaining = self.traffic_light_red_waypoint - closest_waypoint_loc - i

                # if waypoint_remaining < 6: #Maintain a safe distance
                #     update_velocity = 0.0
                # else:
                #     # Slowly reduce velocity based on number of waypoints covered
                #     update_velocity = MAX_SPEED - (75 - waypoint_remaining)*(MAX_SPEED/75)
                # new_velocity = max(0.0, update_velocity)
                # self.set_waypoint_velocity(next_wps, i, new_velocity)
 

        # Publish waypoints        
        lane = Lane()
        lane.waypoints = next_wps
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)


    def pose_cb(self, msg):
        self.current_pose = msg.pose
        #rospy.loginfo(current_pose)
        self.loop()

    def waypoints_cb(self, msg):
        
        if self.waypoints is None:
            self.lane = msg
            self.waypoints = msg.waypoints
            self.loop()
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        #rospy.loginfo(msg)
        #msg = -1 means no traffic light
        self.traffic_light_red_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2  + (pos1.z-pos2.z)**2)

    def get_closet_waypoint_index(self,waypoints,car,start,end):
        min_dist = 99999
        min_ind = None
        for i in range(start, end):
            waypoint = waypoints[i]
            dist = self.get_euclidean_distance(car.position,waypoint.pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                min_ind = i
        return min_ind

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance1(waypoints,wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist) * 3.6
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints
        

    def find_stop_line(self,traffic_light,lane):
        gap = 3
        lane.waypoints[traffic_light].pose.header.frame_id = lane.header.frame_id
        traffic_light_red_waypoint_tf = self.tf_listener.transformPose("base_link", lane.waypoints[traffic_light].pose)
        line_distance = traffic_light_red_waypoint_tf.pose.position.x - gap
        return line_distance


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
