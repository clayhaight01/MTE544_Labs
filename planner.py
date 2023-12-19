from mapUtilities import *
from a_star import *
from rrt import *
from rrt_star import *
import time
import csv

POINT_PLANNER=0; A_STAR_PLANNER=1; RRT_PLANNER=2; RRT_STAR_PLANNER=3


# TODO Modify this class so that is uses the RRT* planner with virtual obstacles

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        self.m_utilites=mapManipulator(laser_sig=0.4)    
        self.costMap=self.m_utilites.make_likelihood_field()

        #TODO Remember to initialize the rrt_star
        
    
    def trajectory_planner(self, startPoseCart, endPoseCart, type):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        #### If not using the map (no bonus), you can just call the function in rrt_star with the appropriate arguments and get the returned path
        #### then you can put the necessary measure to bypass the map stuff down here.
        # Map scaling factor (to save planning time)
        scale_factor = 1 # this is the downsample scale, if set 2, it will downsample the map by half, and if set x, it will do the same as 1/x


        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        

        start_time = time.time()
        
        startPose = [int(i/scale_factor) for i in startPose]
        endPose   = [int(j/scale_factor) for j in endPose]

        mazeOrigin = self.m_utilites.position_2_cell([0,0])

        obstacle_list = [
            (5, 5, 1),
            (3, 6, 2),
            (3, 8, 2),
            (3, 10, 2),
            (7, 5, 2),
            (9, 5, 2),
            (8, 10, 1),
            (6, 12, 1),
        ]  # [x,y,size(radius)]

        # obstacle_list = [
        #     (1, 2.5, 1),
        #     (6, 2.5, 1),
        #     (11, 2.5, 1),
        #     (3.5, 5, 1),
        #     (8.5, 5, 1),
        #     (1, 7.5, 1),
        #     (6, 7.5, 1),
        #     (11, 7.5, 1),
        # ]  # [x,y,size(radius)]

        if self.type==A_STAR_PLANNER:
            path = search(self.costMap, startPose, endPose, scale_factor)
        elif self.type==RRT_PLANNER:
            rrt = RRT(
                    start=startPose,
                    goal=endPose,
                    rand_area=[-2, 15],
                    obstacle_list=obstacle_list,
                    max_iter=1500,
                    robot_radius=0.8)
            path = rrt.planning(animation=show_animation)
        elif self.type==RRT_STAR_PLANNER:
            rrt_star = RRTStar(
                            start=startPoseCart,
                            goal=endPoseCart,
                            rand_area=[-2, 15],
                            obstacle_list=obstacle_list,
                            expand_dis=1,
                            robot_radius=0.4,
                            max_iter=2000)
            path = rrt_star.planning(animation=show_animation)
            path.reverse()
            path = rrt_star.smooth_path(path) # Smooth the path using the function in rrt star


        end_time = time.time()

        # This will display how much time the search algorithm needed to find a path
        print(f"the time took for planning was {end_time - start_time}")

        with open('pathData.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['path_x', 'path_y'])
            for point in path:
                writer.writerow(point)

        return path


if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can do your test here ...

