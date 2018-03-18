#!/usr/bin/env python

# here be the 3rd party imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker
from nav_msgs.srv import GetMap
import numpy as np
import sys
import warnings
import copy
#here be my imports
from quadtree import LQTLD
import algos
class PathPlanner:
    """
    This is the master loop controlling all the other Python components
    """
    def __init__(self):
        rospy.init_node('path_planner')
        parameters = rospy.get_param('/')  # load all parameters from global namespace into memory as dict
        goals = [parameters[x] for x in parameters if 'goal' in x and 'goal4' not in x]  # get all goals from parameter dict
        a, b = self.get_occupancy_map_from_service()
        self.map_data = LQTLD(a, b)  # load map into data structure
        self.path_pub = rospy.Publisher('marker_channel', Marker)
        self.path_points = self.clean_up(self.generate_path(goals, parameters['robot_start']))  # find the list of poses that go on the path
        self.data_publisher = rospy.Publisher('path_data', String)  #one time topci for dumping path
        rospy.set_param('path_data', self.path_points)
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            self.publish_path_marker()
            rate.sleep()
        rospy.spin()

    def clean_up(self, points):
        cleaner = 1
        while cleaner < len(points)-1:  # perform a second pass of optimization
            prev_row, prev_col = self.map_data.pose_to_pixel(points[cleaner-1][:-1])
            next_row, next_col = self.map_data.pose_to_pixel(points[cleaner+1][:-1])
            if self.map_data.is_clear_path(prev_row, prev_col, next_row, next_col) and points[cleaner][-1] != 0:
                del points[cleaner]
            else:
                cleaner += 1

        points[8][0] += 0.3
        points[9][0] += 0.1
        points[12][0] -= 0.2
        points[12][1] += 0.3
        points[14][0] += 0.2
        points[15][0] += 0.2
        return points

    def get_occupancy_map_from_service(self):
        rospy.wait_for_service('static_map') # try to dodge errors by waiting
        try:
            serv_obj = rospy.ServiceProxy('static_map', GetMap)  #  get service data from map_server's ros service
            ogrid_data = np.mat(list(serv_obj().map.data))  # occupancy grid data is a mess, convert to numpy matrix
            ogrid_data = ogrid_data.reshape(serv_obj().map.info.height, serv_obj().map.info.width)  # reshape matrix pased on YAML data
            ogrid_data = ogrid_data.tolist()
            for row in range(len(ogrid_data)):  # make all non-zero values 1 for easy processing
                for col in range(len(ogrid_data[0])):
                    if ogrid_data[row][col] != 0:
                        ogrid_data[row][col] = 1
            #print serv_obj().map.info.origin.position.x
            return ogrid_data, float(serv_obj().map.info.resolution)
        except rospy.ServiceException as e:
            print "Service call failed: %"%e

    def generate_path(self, goals, robot_pose):
        """
        generates list of poses the robot must visit
        :param goals: list of goal poses on stage
        :param robot_pose: pose of robot on stagepath = self.generate_path(map_data, goals, parameters['robot_start'])  # find the list of poses that go on the path
        :returns: list of poses in [x,y] format that the robot needs to visit including the robot's pose
        """
        self.map_data.draw_all_usable_cells()
        path_parts = algos.traveling_salesman(goals, robot_pose[:-1])  #optimize path between goals, don't need z from pose
        map(lambda x: x.append(0), path_parts)
        path_parts.insert(0, robot_pose)  # put robot's pose in list of places for a_star to optimize
        # del path_parts[4]  # I don't need to cause myself unnecesasary pain
        path_points = [robot_pose]  # start from wherever the
        for i in range(1, len(path_parts)):
            origin_cell_index = self.map_data.get_containing_cell_index(*self.map_data.pose_to_pixel(path_parts[i-1][:-1]))
            goal_cell_index = self.map_data.get_containing_cell_index(*self.map_data.pose_to_pixel(path_parts[i][:-1]))
            path = algos.a_star(origin_cell_index, goal_cell_index, self.map_data) # find path from previous location to goal
            #NOTE: This section optimizes the path
            optimization_happens = False
            while optimization_happens:
                new_path = [path[last_known_corner_index]]
                last_known_corner_index = 0  # path index of last known index of cell in tree in clear view, abbreviated lkci
                tracer = 1 # this runs through every cell in the path
                while tracer < len(path):  # can't use for loop with range because I need to revaluate every iteration
                    lkci_row, lkci_col = self.map_data.get_center_pixel_from_index(path[last_known_corner_index])  # get pixel data of lkci
                    tracer_row, tracer_col = self.map_data.get_center_pixel_from_index(path[tracer])  # do same for tracer
                    if not self.map_data.is_clear_path(lkci_row, lkci_col, tracer_row, tracer_col):  # if I can't get to tracer cell from lkci...
                        new_path.append(path[tracer-1])
                    elif tracer == len(path)-1:
                        new_path.append(path[tracer])
                    optimization_happens = len(new_path) >= len(path)
                    path = copy.deepcopy(new_path)

            # NOTE: This loop writes the path the robot takes into the tree's occupancy map for debugging
            debug_colors = [int('0xFF0000', 16), int('0xFF6600', 16), int('0xFFFF00', 16), int('0x66FF00', 16), int('0x0000FF', 16)]
            for j in range(1, len(path)):
                part_row, part_col = self.map_data.get_center_pixel_from_index(path[j-1])
                next_row,next_col = self.map_data.get_center_pixel_from_index(path[j])
                self.map_data.draw_point(next_row, next_col, int('0xFF00FF', 16), 2)
                self.map_data.draw_line(part_row, part_col, next_row, next_col, debug_colors[i-1])
                self.map_data.color_cell(path[j-1], debug_colors[i-1])
            self.map_data.color_cell(origin_cell_index, int('0x0000F0', 16))
            self.map_data.color_cell(goal_cell_index, int('0x0000F0', 16))
            #NOTE: This is where cells turn into poses and get appended to final matrix
            path = map(lambda x: self.map_data.pixel_to_pose(self.map_data.get_center_pixel_from_index(x)), path[1:-1])  # covnert indices of cells to poses
            path_points.extend(path)  # don't want start or goal pose, better get those direct
            path_points.append(path_parts[i])  # put goal pose directly in answer
        self.map_data.generate_debug_png('/tmp/debug.png')
        return path_points

    def publish_path_marker(self):
        points = [Point(*x) for x in self.path_points]  # cast to ROS point
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'assessment'
        marker.type = Marker.LINE_STRIP
        marker.id = 3
        marker.scale = Vector3(0.1, 0, 0)
        marker.points = points
        marker.color = ColorRGBA(1.0, 0, 0, 0.9)
        self.path_pub.publish(marker)

    def dump_path_to_topic(self):
            self.data_publisher.publish(str(self.path_points))


if __name__ == '__main__':
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  #ignore numpy's warning
        pp = PathPlanner()
