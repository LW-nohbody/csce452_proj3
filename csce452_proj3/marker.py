import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
import math



# tracks person position
class Person():
    def __init__(self, id: int, curr_pos: Point):
        self.id = id 
        self.curr_pos = Point()
        self.pos: list [Point] = []
        self.updatePos(curr_pos)
    
    def updatePos(self, new_pos:Point): #Creates a copy of the point and appends that to the array
        temp = Point()
        temp.x = new_pos.x 
        temp.y = new_pos.y 
        temp.z = new_pos.z
        self.curr_pos = temp
        self.pos.append(temp)

# Creates markers for each group of points sent over the /moved_positions topic
class Sim_Marker(Node):
    def __init__(self):
        super().__init__('marker')
        self.marker_pub = self.create_publisher(MarkerArray, '/sim_markers', 10)
        self.point_sub = self.create_subscription(PoseArray, '/moved_positions', self.translate, 10)
        self.timer = self.create_timer(1.0, self.publishMarkers)
        self.marker_arr = []
        self.point_arr: list[Point] = []
        self.people: list[Person] = []
        self.curr_id = 0
        self.max_dist_for_person = 1.0 # max distance for a point to be considered apart of an existing person
    
    def publishMarkers(self):
        markerList = MarkerArray()
        markerList.markers = self.marker_arr
        self.marker_pub.publish(markerList)
    
    def translate(self, msg):
        # Gets point of groups with significant movement
        self.point_arr = []
        pose_arr = msg.poses
        for i in pose_arr:
            self.point_arr.append(i.position)

        grouped_points = self.point_arr[:]

        # Handles first seen case
        if(self.people == []):
            for group in grouped_points:
                self.get_logger().info(f"Creating new person with {self.curr_id} id for point ({group.x}, {group.y})") #TODO:REMOVE
                self.people.append(Person(self.curr_id, group))
                self.curr_id += 1
        else:
            #Find the person closest to each group and assign it to them
            #TODO: Take into account person's velocity when assigning groups (should only check previous 2-3 points for it)
            temp_people = self.people[:]
            assigned_groups: list[Point] = []
            for group in grouped_points:
                closest_p = None
                closest_dist = -1
                self.get_logger().info(f"temp_people size: {len(temp_people)}, self.people size: {len(self.people)}") #TODO:REMOVE
                for p in temp_people:
                    dist_to_point = getDist(group, p.curr_pos)
                    is_close_to_point = dist_to_point <= self.max_dist_for_person
                    if closest_dist == -1 and is_close_to_point:
                        closest_p = p
                        closest_dist = getDist(group, p.curr_pos)
                    elif (getDist(group, p.curr_pos) < closest_dist) and is_close_to_point:
                        closest_p = p 
                        closest_dist = getDist(group, p.curr_pos)
                if(closest_p == None): continue
                temp_people.remove(closest_p)
                p.updatePos(group)
                self.get_logger().info(f"Updating person {p.id} with point: ({group.x}, {group.y}), for {len(p.pos)} point total") #TODO: REMOVE
                assigned_groups.append(group)
            
            if(len(assigned_groups) < len(grouped_points)): #Handles new people
                for group in grouped_points:
                    if not (group in assigned_groups):
                        self.get_logger().info(f"Creating new person with {self.curr_id} id for point ({group.x}, {group.y})") #TODO: REMOVE
                        self.people.append(Person(self.curr_id, group))
                        self.curr_id += 1
                        assigned_groups.append(group)


        #clear marker array and repopulate it
        self.marker_arr = []
        for p in self.people:
            self.createMarker(p)
        
        self.publishMarkers()
        
    
    def createMarker(self, person:Person):
        marker = Marker()
        marker.header.frame_id = "/laser"
        marker.header.stamp = Node.get_clock(self).now().to_msg()
        
        marker.ns = "person"
        marker.id = person.id

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.0

        id_mod_4 = person.id % 4
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        if(id_mod_4 == 0):
            marker.color.r = 1.0
        elif(id_mod_4 == 1):
            marker.color.g = 1.0
        elif(id_mod_4 == 2):
            marker.color.b = 1.0
        elif(id_mod_4 == 3):
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.points = person.pos

        marker.lifetime.sec = 0

        self.marker_arr.append(marker)


def getDist(point1:Point, point2:Point):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def main():
    rclpy.init()

    mark = Sim_Marker()

    # startPoint = Point()
    # startPoint.x = 0.0
    # startPoint.y = 0.0
    # startPoint.z = 0.0
    # test = Person(0, startPoint)


    # testPoint = Point()
    # testPoint.x = 2.49
    # testPoint.y = -1.65
    # #R = 2.987
    # #theta = -0.58
    # test.updatePos(testPoint)

    # #R = 2.09 theta = -1.847
    # testPoint.x = -0.57
    # testPoint.y = -2.01

    # test.updatePos(testPoint)    
    

    # mark.createMarker(test)

    rclpy.spin(mark)

    mark.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
     
