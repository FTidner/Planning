#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from heapq import heappush, heappop
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Path 
from tf.transformations import quaternion_from_euler
from nav_msgs.srv import GetPlan
from part2.srv import PathPlanner, PathPlannerResponse


def is_closed(node_x,node_y,closed_list):

    for x,y in closed_list:
        if x==node_x and y==node_y:
            return True
            break

    return False

#req: 
# start :geometry_msgs/Posestamped
# goal: geometry_msgs/poseStamped
# grid: nav_msgs/OccupancyGrid
# ---
# path: nav_msgs/Path 

def pathPlanner(req):
    start = req.start
    goal= req.start
    grid = req.grid


    start_cost = 0

    start_pose = req.start
    goal_pose = req.goal
    

    start=(int(round(start.pose.position.x/grid.info.resolution)), int(round(start.pose.position.y/grid.info.resolution)))
    goal = (int(round(goal.pose.position.x/grid.info.resolution)), int(round(goal.pose.position.y/grid.info.resolution)))

    if (start[0]<0 or start[0]>grid.info.width or start[1]< 0 or start[1]>grid.info.height or grid.data[start[0]+grid.info.width*start[1]] == 100):
        rospy.loginfo('Invalid Start node')
    if (goal[0]<0 or goal[0]>grid.info.width or goal[1]< 0 or goal[1]>grid.info.height or grid.data[goal[0]+grid.info.width*goal[1]] == 100):
        rospy.loginfo('Invalid Goal node ')
    


    start_cost2go = np.sqrt(pow(goal[0]-start[0], 2) + pow(goal[1]-start[1],2))

    open_list = [(start_cost2go,start_cost,start,None)]

    closed=[]


    # dict used to keep track of path
    Camefrom = {}
    movements = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]

    while open_list:
        node_element=heappop(open_list) 

        cost2go, cost , curr_node , prev = node_element
        if is_closed(curr_node[0],curr_node[1],closed):
            continue

        Camefrom[curr_node] = prev

        if curr_node == goal:
            rospy.loginfo('Goal reached!')
            break
        k=0
        for dx,dy in movements:
            k+=1
            next_node = (curr_node[0] + dx, curr_node[1] + dy)
            
             
            # Test if position is legit
            # needs to check if new node is visited
            if (next_node[0]<0 or next_node[0]>grid.info.width or next_node[1]< 0 or next_node[1]>grid.info.height or grid.data[next_node[0]+grid.info.width*next_node[1]] == 100)==False:

                if k<5:
                    step_cost=1
                else:
                    step_cost=np.sqrt(2)

                
                next_cost = grid.data[next_node[0]+grid.info.width*next_node[1]] + step_cost + cost2go
                next_cost2go = next_cost + np.sqrt(pow(goal[0]-next_node[0], 2) + pow(goal[1]-next_node[1],2))
                heappush(open_list,(next_cost2go,next_cost,next_node,curr_node))
                
        closed.append(curr_node)
       
    

    if curr_node ==goal:

        path =Path()
        path_node= curr_node
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        while path_node:

            node_pose = PoseStamped()
            node_pose.header.frame_id = "map"
            node_pose.header.stamp = rospy.Time() 
            node_pose.pose.position.x = (path_node[0]*grid.info.resolution)
            node_pose.pose.position.y = (path_node[1]*grid.info.resolution)
            node_pose.pose.position.z = start_pose.pose.position.z
            node_pose.pose.orientation = goal_pose.pose.orientation

            '''
            node_pose.pose.orientation.x,node_pose.pose.orientation.y,node_pose.pose.orientation.z, node_pose.pose.orientation.w = quaternion_from_euler(0.0,0.0,0.0)
            '''
            
       
            path.poses.append(node_pose)
            path_node = Camefrom[path_node]


        
            
        path.poses.reverse()
        #pathpub.publish(path)

        #shortens the number of setpoints
        xdiff_prev = round(path.poses[0].pose.position.x-path.poses[1].pose.position.x,1)
        ydiff_prev = round(path.poses[0].pose.position.y-path.poses[1].pose.position.y,1)
        k=2
        for i in range(path.poses):

            if path.poses[k]:
                xdiff = round(path.poses[k-1].pose.position.x-path.poses[k].pose.position.x,1)
                ydiff = round(path.poses[k-1].pose.position.y-path.poses[k].pose.position.y,1)
                if xdiff==xdiff_prev and ydiff==ydiff_prev:
                    path.poses=path.poses.pop(k-1)
                else:
                    k+=1
                xdiff_prev=xdiff
                ydiff_prev=ydiff



        

        rospy.loginfo('Path published')
    else:
        rospy.loginfo('No path found')
    
    
    return PathPlannerResponse(path)

    





def main():
    rospy.init_node('PathPlanner')
    get_plan = rospy.Service('Get_plan', PathPlanner, pathPlanner)
    rospy.loginfo('Server initialized')
    rospy.spin()

    

    


if __name__ == "__main__":
    main()  