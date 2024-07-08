import numpy as np

def global2occupancy(points, bias_x, bias_y, resolution):
    """
    points: [x, y], denoting the global coordinates(e.g. in SLAM coordinates)
    bias_x, bias_y: the left bottom corner of the occupancy grid in global coordinates
    resolution: transfrom the meters to the occupancy grid(e.g. X meters = 1 grid)
    return: [x, y], denoting the occupancy grid coordinates
    
    """
    points_map = points - np.array([bias_x, bias_y])
    points_map /= resolution
    points_map = points_map.astype(int)
    return points_map

def occupancy2global(points_map, bias_x, bias_y, resolution):
    """
    points_map: [x, y], denoting the occupancy grid coordinates
    bias_x, bias_y: the left bottom corner of the occupancy grid in global coordinates
    resolution: transfrom the meters to the occupancy grid(e.g. X meters = 1 grid)
    return: [x, y], denoting the global coordinates(e.g. in SLAM coordinates)
    
    """
    points = points_map * resolution
    points += np.array([bias_x, bias_y])
    return points   


def global_to_robot(path):
    """
    path: [[x1, y1], [x2, y2], ...], denoting the global coordinates(e.g. in SLAM coordinates)
    return: [[x1, y1], [x2, y2], ...], denoting the robot coordinates(e.g. in the AlienGo IMU coordinates)
    
    """
    # path_swapaxes = np.swapaxes(path, 0, 1)
    # path_swapaxes[1] = -path_swapaxes[1]
    # path_robot = path_swapaxes.T
    path_robot = path.copy()
    path_robot[:, [0,1]]= path_robot[:, [1,0]]
    path_robot[:,-1] = -path_robot[:,-1]
    return path_robot
    

#traj_arr = np.array([[0,0], [1.5, 0], [0, 0]])

#test = gloabl_to_robot(traj_arr)