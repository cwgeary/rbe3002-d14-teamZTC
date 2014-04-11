
import rospy, tf, math
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from nav_msgs.msg import GridCells #format for publishing to rviz display
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header


#takes a gridPoint and converts it into a global point for display in rviz.
def gridToGlobal(gridPoint, mapInfo):
    
    point = Point()
    point.x = gridPoint[0] * mapInfo.resolution + mapInfo.origin.position.x + mapInfo.resolution/2
    point.y = gridPoint[1] * mapInfo.resolution + mapInfo.origin.position.y + mapInfo.resolution/2
    point.z = 0

    return point

#takes a point in the global frame and returns the point value for the given cell in the map frame as a tuple.
#note that zero is set as the LOWER left corner, with +x to the right and +y down.
def globalToGrid(point, mapInfo):
    
    offsetX = mapInfo.origin.position.x
    offsetY = mapInfo.origin.position.y

    gridPoint = (int((point.x - offsetX)/ mapInfo.resolution), int((point.y - offsetY)/ mapInfo.resolution))

    return gridPoint

def indexToGrid(index, mapInfo):
    gridPoint = [index % mapInfo.width , index / mapInfo.width]
    return gridPoint


#convert the grid point into a map index value, and outputs it.
def gridToIndex(gridPoint, mapInfo, mapData):

    return gridPoint[0] + gridPoint[1] * (mapInfo.width)

def pubMap(publisher, mapInfo, mapData):
    cells = []
    for i in range(mapInfo.width * mapInfo.height):
        if mapData[i] == 100:
        	gridPoint = indexToGrid(i, mapInfo)
        	gp = gridToGlobal(gridPoint, mapInfo)
  
        	cells.append(gp)

        	gridCells = GridCells()
        	gridCells.header = Header()
        	gridCells.header.frame_id = "map"
        	gridCells.cell_width = mapInfo.resolution
        	gridCells.cell_height = mapInfo.resolution
        	gridCells.cells = cells
    publisher.publish(gridCells)

def publishGridList(list_of_cells, mapInfo, pub):
    cells = []
    for x in list_of_cells:
        cells.append(gridToGlobal(x, mapInfo))
    
    gridCells = GridCells()
    gridCells.header = Header()
    gridCells.header.frame_id = "map"
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    gridCells.cells = cells

    pub.publish(gridCells)
