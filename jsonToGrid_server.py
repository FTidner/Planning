#!/usr/bin/env python

import sys
import rospy
import json
import numpy as np
from nav_msgs.srv import GetMap,GetMapResponse
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header



def create_Grid(req):
    
    argv=sys.argv

    args = rospy.myargv(argv=argv)

    with open(args[1], 'rb') as f:
        world = json.load(f)

    
 
    airspace = world['airspace']
    x= airspace['max'][0]-airspace['min'][0]
    y=airspace['max'][1]-airspace['min'][1]
    grid = OccupancyGrid()
    grid.header.frame_id="map"
    grid.header.stamp = rospy.Time.now()
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = 0.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.x = 0.0
    grid.info.origin.orientation.y = 0.0
    grid.info.origin.orientation.z = 0.0
    grid.info.origin.orientation.w=1.0

    grid.info.map_load_time = rospy.Time.now()

    grid.info.resolution=0.05
    
    grid.info.width = 40  #int(x/grid.info.resolution)
    grid.info.height = 40 #int(y/grid.info.resolution)
    
    occupancy = [[1 for col in range(grid.info.width)] for row in range(grid.info.height)]


    for xCell in range(0,grid.info.width):
        for yCell in range(0,grid.info.height):
            xcellPose = xCell*grid.info.resolution
            ycellPose = yCell*grid.info.resolution 
            for wall in world['walls']:
                wall_xlb = (wall['plane']['start'][1]-0.1)
                wall_xub = (wall['plane']['stop'][1]+0.1)
                wall_ylb = (wall['plane']['start'][0]-0.1)
                wall_yub = (wall['plane']['stop'][0]+0.1)
                if  xcellPose >= wall_xlb and  xcellPose <= wall_xub and ycellPose >= wall_ylb and ycellPose <= wall_yub:
                    occupancy[xCell][yCell]= 100
                    break 
                else:
                    occupancy[xCell][yCell]=0       
            grid.data.append(occupancy[xCell][yCell])
    
    
    
    return GetMapResponse(grid)    # response

            


    #gridPub.publish(grid)

    
#rospy.init_node('OccupancyGrid')
#gridPub=rospy.Publisher('/map', OccupancyGrid, queue_size=1)


def main():
    rospy.init_node('Occupancy_Grid_server')
    rospy.loginfo('Server initialized')
    get_map = rospy.Service('set_Occupancy_grid',GetMap,create_Grid)
    rospy.spin()
    
        
if __name__ == "__main__":
    main()      
    







