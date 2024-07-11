import rospy
from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import Point, Polygon, Point32
import numpy as np
from queue import LifoQueue
from matplotlib import pyplot as plt
from utils_planning import plan
import random
import os
from copy import deepcopy
from __tools__ import global2occupancy, occupancy2global, global_to_robot
from __global__ import  start_point, goal_point, num, start_points_list, path_robot


def map_call_back(data):
        
        print('Get the occupancy map!')
        map = data.data
        map_array_1d = np.array(map)
        map_h = data.info.height
        map_w = data.info.width 
        bias_x, bias_y, bias_z = data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z
        ori_x, ori_y, ori_z = data.info.origin.orientation.x, data.info.origin.orientation.y, data.info.origin.orientation.z

        print("------ MAP INFORAMTION ------")
        print("map shape: ", map_h, map_w)
        print("oris: ", ori_x, ori_y, ori_z )
        print("bias: ", bias_x, bias_y, bias_z)
        map_array = map_array_1d.reshape(map_h, map_w)
        print("map array shape:", map_array.shape)

        free_index = np.where(map_array == 0)
        resolution = data.info.resolution
        global2occupancy_func = lambda x: global2occupancy(x, bias_x, bias_y, resolution)
        occupancy2global_func = lambda x: occupancy2global(x, bias_x, bias_y, resolution)

        global start_point
        global goal_point

        # DEBUG: manually set the start and goal point
        goal_point = np.array([-1.0, 1.0])
        # start_point = np.array([-0.0, 0.0])

        print("start point: ", start_point)
        print("goal point: ", goal_point)

        if start_point is not None and goal_point is not None:
                
                #print("Start point: ", start_point)
                # start_point -= np.array([bias_x, bias_y])
                # start_point /= resolution
                #start_points_list.append(start_point)
                #print("start_points_list: ", len(start_points_list))
                #start_points_ori = deepcopy(start_points_list)
                #global start_points
                # start_point_map = start_point - np.array([bias_x, bias_y])
                # start_point_map /= resolution
                
                # goal_point_map = goal_point - np.array([bias_x, bias_y])
                # goal_point_map /= resolution

                
                # start_point  = None
                # goal_point = None
                
                
                points = np.array([start_point, goal_point])
                print("start point and goal points in global map: ", points)

                points_map = global2occupancy_func(points)
                start_valid, goal_valid = points_map[0], points_map[1]
                print("start point and goal point in occupancy map: ", points_map)

                print("------ POINT VALIDATION CHECK (VALID IF THE VALUE IS 0) ------")                
                # if the start point is not valid, enforce the start point to be free space
                map_array[start_valid[1], start_valid[0]] = 0
                print(f"start point check: {map_array[start_valid[1], start_valid[0]]}, goal point check: {map_array[goal_valid[1], goal_valid[0]]}")

                # print("start_points: ", start_points[:5,], start_points.shape, bias_x, bias_y, map_array.shape)
                # plt.imshow(map_array, interpolation='nearest')
                # plt.scatter(start_points[:,0], start_points[:,1], c='r', s=5)
                # global num
                # save_path = os.path.join('fig_debug', 'start_point_{}.png'.format(num))   
                # plt.savefig(save_path)        
                # plt.close()
                # print("Start point: ", start_point)
                # print("value", map_array[int(start_point[1]), int(start_point[0])])
                #assert map_array[int(start_point[1]), int(start_point[0])] ==0, "The start point is not free space"
                # if map_array[int(start_point[1]), int(start_point[0])] ==0:
                #         print("can receive global info now")

                # plt.imshow(map_array, interpolation='nearest')
                # plt.scatter(int(start_point[1]), int(start_point[0]), c='r', s=5)  
                # global num
                # save_path = os.path.join('fig_debug', 'start_point_{}.png'.format(num))   
                # plt.savefig(save_path)        
                # num += 1
                # plt.close()
                # if sg_flag:
                #         print("can receive global info now")
                #         s_g_flag = False
                # Check if there are at least two indices
                # if free_index[0].size < 2:
                #         print("Not enough zero elements in the array")
                # else:
                #         # Randomly select two indices
                #         index_choices = random.sample(range(free_index[0].size), 2)
                #         row_col = np.array((free_index[0][index_choices], free_index[1][index_choices]))
                #         selected_indices = row_col.T
                # print("Selected indices: ", selected_indices)
                # assert map_array[selected_indices[0,0], selected_indices[0,1]] == 0 and map_array[selected_indices[1,0], selected_indices[1,1]] == 0, "The selected indices are not free space"
                # #print(map_array[selected_indices[0,0], selected_indices[1,0]], map_array[selected_indices[0,1], selected_indices[1,1]])
                # selected_indices = np.array([[51, 193], [219, 202]])
                # points = (selected_indices+1)*resolution
                #print("Selected indices: ", selected_indices.shape, selected_indices)
                #print(free_index)
                #np.save('map.npy', map_array)
                #unique_elements = np.unique(map_array)
                #num_unique_elements = unique_elements.shape[0]
                #print("Number of unique elements in the array: ", unique_elements)
                #(242,175)(229, 373)
                #points = np.array([[12.15,8.8], [11.5, 18.7]])
                #points = selected_indices
        
                # path planning
                path = plan(points, 2.0, 'lazyprmstar', 'PathClearance', map_array, resolution, global2occupancy_func, occupancy2global_func)
                print(path)
                
                plt.imshow(map_array, interpolation='nearest')
                
                # points_map = global2occupancy(points, bias_x, bias_y, resolution)
                plt.scatter(points_map[1,0], points_map[1,1], c='r', s=5)
                plt.scatter(points_map[0,0], points_map[0,1], c='g', s=5)

                if path is not None:
                        path_map = global2occupancy(path, bias_x, bias_y, resolution)
                        plt.scatter(path_map[1:-1,0], path_map[1:-1,1], c='b', s=5)
                        plt.show()
                        global path_robot
                        path_robot = global_to_robot(path)
                        print("path_robot: ", path_robot)

                start_point = None
                goal_point = None


def goal_point_call_back(data):
        
        global goal_point
        goal_point = np.array([data.x, data.y])

        #TODO: get the goal point from the preception module


def start_point_call_back(data):

        print('Get the start point!')
        print(data.x, data.y)
        global start_point
        start_point = np.array([data.x, data.y])

def ros_initialize():
        
        rospy.init_node('planning', anonymous=True)
        rospy.Subscriber('/projected_map', OccupancyGrid, map_call_back, queue_size=10)
        rospy.Subscriber('orbslam2/startpoint', Point, start_point_call_back, queue_size=1)
        # rospy.Subscriber('visual_grounding/goalpoint', Point, goal_point_call_back, queue_size=10)

        pub = rospy.Publisher('planner/trajectory', Polygon, queue_size=1)

        while not rospy.is_shutdown():
                global path_robot
                if path_robot is not None:
                        path_robot_msg = Polygon()
                        path_robot_msg.points = [Point32(x=path_robot[i,0], y=path_robot[i,1]) for i in range(path_robot.shape[0])]
                        pub.publish(path_robot_msg)
                        #path_robot = None
                        print("------ TRAJECTORY PUBLISHED ------")
                rospy.sleep(0.1)

        # global num
        # while True:
        #         #print("num: ", num)
        #         if num >= 100:
        #                 plt.imshow(map_array, interpolation='nearest')
        #                 plt.scatter(start_points[:,0], start_points[:,1], c='r', s=5)
        #                 save_path = os.path.join('fig_debug', 'start_points_{}.png'.format(num))
        #                 plt.savefig(save_path)
        #                 plt.close()
                        #print("Save the start points")
        #rospy.Subscriber('/test', OccupancyGrid, sg_call_back)
        ########for debug
        # pub = rospy.Publisher('/projected_map', OccupancyGrid, queue_size=1)
        # #pub2 = rospy.Publisher('/test', OccupancyGrid, queue_size=1)
        # rate = rospy.Rate(10)
        # temp = OccupancyGrid()
        # map_array = np.load("map.npy")


        # pub1 = rospy.Publisher('orbslam2/startpoint', Point, queue_size=1)
        # rate = rospy.Rate(10)
        # temp1 = Point()
        # temp1.x = 12.15
        # temp1.y = 8.8

        # while not rospy.is_shutdown():
        #         temp.header.frame_id = 'map'
        #         temp.header.stamp = rospy.Time.now()
        #         temp.info.width = map_array.shape[1]
        #         temp.info.height = map_array.shape[0]
        #         temp.info.resolution = 0.05
        #         #numpy initialize to -1 
        #         #data = np.full((20, 20), -1)
        #         data = map_array.astype(int)
        #         temp.data = data.flatten()
        #         #pub2.publish(temp)
        #         pub.publish(temp)
        #         pub1.publish(temp1)
        #         rate.sleep()

        rospy.spin()

if __name__=='__main__':

        ros_initialize()
        #path_plan()