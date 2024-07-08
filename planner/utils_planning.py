 
import sys

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
import numpy as np



class ValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the
    # circular obstacle
    def __init__(self, si, occupancy_map, global2occupancy_func):
        super(ValidityChecker, self).__init__(si)
        self.occupancy_map = occupancy_map
        self.global2occupancy_func = global2occupancy_func    

    def isValid(self, state):
        return self.clearance(state)
    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.

    def clearance(self, state):
        # Extract the robot's (x,y) position from its state
        dof = len(self.occupancy_map.shape)
        state_array = np.array([state[i] for i in range(dof)])
        x, y = self.global2occupancy_func(state_array)
        # x = int(state[0]/self.resolution) #int for the floor
        # y = int(state[1]/self.resolution)
        #print(x,y)
        free_value = 0
        # Distance formula between two points, offset by the circle's
        # radius
        try:
            return (self.occupancy_map[y][x]==free_value)
        except:
            return False



def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)


def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj


class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
                            sys.float_info.min))


def getClearanceObjective(si):
    return ClearanceObjective(si)


def getBalancedObjective1(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(clearObj, 1.0)

    return opt





def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    elif plannerType.lower() == "rrtconnect":
        return og.RRTConnect(si)
    elif plannerType.lower() == "lazyprmstar":
        return og.LazyPRMstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")



def plan(points, runTime, plannerType, objectiveType, occupancy_map, resolution, global2occupancy_func, occupancy2global_func, dof=2):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.RealVectorStateSpace(dof)

    # Set the bounds of space to be in [0,1].
    left_bottom = occupancy2global_func(np.array([0, 0]))
    right_top = occupancy2global_func(np.array(occupancy_map.shape))
    low = left_bottom[0]
    #TODO: should check
    high = right_top[1]
    #bounds = np.min(occupancy_map.shape)*resolution
    space.setBounds(low, high)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si, occupancy_map, global2occupancy_func)
    si.setStateValidityChecker(validityChecker)

    si.setup()

    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    goal = ob.State(space)
    #get the start and goal point
    start_point = points[0]
    goal_point = points[1]
    
    for i in range(dof):
        start[i] = start_point[i]
        goal[i] = goal_point[i]
    # if dof == 2:
    #     start[0] = start_point[0]
    #     start[1] = start_point[1]
    #     goal[0] = goal_point[0]
    #     goal[1] = goal_point[1]
        # s_x, s_y = int(start_point[0]/0.05),int(start_point[1]/0.05)
        # g_x, g_y = int(goal_point[0]/0.05),int(goal_point[1]/0.05)
        # start_value = occupancy_map[s_x][s_y]
        # goal_value = occupancy_map[g_x][g_y]
        # print(start_value, goal_value)
    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()
    #breakpoint()
    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)
    

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization ' \
            'objective value of {2:.4f}'.format( \
            optimizingPlanner.getName(), \
            pdef.getSolutionPath().length(), \
            pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))

        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        # if fname:
        #     with open(fname, 'w') as outFile:
        #         outFile.write(pdef.getSolutionPath().printAsMatrix())
        pdef.getSolutionPath().interpolate()
        path_osteram = pdef.getSolutionPath().printAsMatrix()
        #flag = pdef.getSolutionPath().check()
        #print(flag)
        cleaned_path = path_osteram.replace("\n", " ").strip()
        path_list = [float(num) for num in cleaned_path.split()]
        path = np.array(path_list).reshape(-1, dof)
    else:
        path = None

    return path

# if __name__ == "__main__":
#     # Create an argument parser
#     parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

#     # Add a filename argument
#     parser.add_argument('-t', '--runtime', type=float, default=1.0, help=\
#         '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
#     parser.add_argument('-p', '--planner', default='RRTstar', \
#         choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
#         'SORRTstar','RRTconnect'], \
#         help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
#     parser.add_argument('-o', '--objective', default='PathLength', \
#         choices=['PathClearance', 'PathLength', 'ThresholdPathLength', \
#         'WeightedLengthAndClearanceCombo'], \
#         help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
#     parser.add_argument('-f', '--file', default=None, \
#         help='(Optional) Specify an output path for the found solution path.')
#     parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
#         help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
#         ' Defaults to WARN.')

#     # Parse the arguments
#     args = parser.parse_args()

#     # Check that time is positive
#     if args.runtime <= 0:
#         raise argparse.ArgumentTypeError(
#             "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
#             % (args.runtime,))

#     # Set the log level
#     if args.info == 0:
#         ou.setLogLevel(ou.LOG_WARN)
#     elif args.info == 1:
#         ou.setLogLevel(ou.LOG_INFO)
#     elif args.info == 2:
#         ou.setLogLevel(ou.LOG_DEBUG)
#     else:
#         ou.OMPL_ERROR("Invalid log-level integer.")

#     # Solve the planning problem
#     plan(args.runtime, args.planner, args.objective, args.file)