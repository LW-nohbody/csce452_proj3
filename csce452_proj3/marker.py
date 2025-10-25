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
        if math.hypot(self.curr_pos.x - new_pos.x, self.curr_pos.y - new_pos.y) < 1e-4:
            return
        temp = Point()
        temp.x = new_pos.x 
        temp.y = new_pos.y 
        temp.z = new_pos.z
        self.curr_pos = temp
        self.pos.append(temp)
    
    def getVel(self):
        vel_r:float = 0.0
        vel_theta:float = 0.0
        if(len(self.pos) >= 5):
            vel_x = 0
            vel_y = 0
            for i in range(1, 5):
                vel_x += (self.pos[len(self.pos)-i].x - self.pos[len(self.pos) - i - 1].x)
                vel_y += (self.pos[len(self.pos)-i].y - self.pos[len(self.pos) - i - 1].y)
            vel_x = vel_x / 4
            vel_y = vel_y / 4
        elif(len(self.pos) >= 2):
            vel_x = self.pos[len(self.pos)-1].x - self.pos[len(self.pos)-2].x
            vel_y = self.pos[len(self.pos)-1].y - self.pos[len(self.pos)-2].y
        else:
            vel_x = 0
            vel_y = 0
            vel_r = 0.0 
            vel_theta = 0.0
            return (vel_r, vel_theta)
    
        vel_r = math.sqrt(vel_x**2 + vel_y**2)
        if vel_x != 0:
            vel_theta = math.atan(vel_y/vel_x)
        elif vel_x == 0 and vel_y==0: #At origin
            vel_theta = 0
        else: #Vel_x is zero, but vel_y is non-zero
            vel_theta = math.pi/2 * (vel_y/abs(vel_y))
        
        if (vel_x < 0) and (vel_y < 0):
            vel_theta = vel_theta-math.pi
        elif (vel_x < 0) and (vel_y >= 0):
            vel_theta = vel_theta + math.pi

        return (vel_r, vel_theta)
    
    def predictPos(self, dt: int = 1):
        vel_r, vel_theta = self.getVel()
        if vel_r == 0.0:
            return self.curr_pos

        predicted = Point()
        predicted.x = self.curr_pos.x + vel_r * math.cos(vel_theta) * dt
        predicted.y = self.curr_pos.y + vel_r * math.sin(vel_theta) * dt
        predicted.z = 0.0
        return predicted
    
# Creates markers for each group of points sent over the /moved_positions topic
class Sim_Marker(Node):
    def __init__(self):
        super().__init__('marker')
        self.marker_pub = self.create_publisher(MarkerArray, '/sim_markers', 10)
        self.point_sub = self.create_subscription(PoseArray, '/moved_positions', self.translate, 10)
        self.marker_arr = []
        self.point_arr: list[Point] = []
        self.people: list[Person] = []
        self.still_people_count: dict[Person:int] = {}
        self.stationary_threshold = 10
        self.curr_id = 0
        self.max_dist_for_person = 0.9
        self.missed_counts = {}
        self.max_missed_frames = 10

        #TODO: find max_angle_diff to use that limits the choices for a point without splitting a person
        self.max_angle_diff = math.pi*2 # max angle a point can change from a person's previous path
        self.movement_threshold = 0.03
    
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
                self.get_logger().info(f"Creating new person with id: {self.curr_id} for point ({group.x}, {group.y})") #TODO:REMOVE
                self.people.append(Person(self.curr_id, group))
                self.curr_id += 1
        else:
            #Find the person closest to each group and assign it to them
            # temp_people = self.people[:]
            # assigned_groups: list[Point] = []
            # for group in grouped_points:
            #     closest_p = None
            #     closest_dist = -1
            #     for p in temp_people:
            #         dist_to_point = getDist(group, p.curr_pos)
            #         is_close_to_point = dist_to_point <= self.max_dist_for_person
            #         if closest_dist == -1 and is_close_to_point:
            #             closest_p = p
            #             closest_dist = getDist(group, p.curr_pos)
            #         elif (getDist(group, p.curr_pos) < closest_dist) and is_close_to_point:
            #             closest_p = p 
            #             closest_dist = getDist(group, p.curr_pos)
            #     if(closest_p == None): continue
            #     temp_people.remove(closest_p)
            #     p.updatePos(group)
            #     assigned_groups.append(group)
            # --- minimal, points-first association ---
            temp_people = self.people[:]
            assigned_groups: list[Point] = []

            for point in grouped_points:
                best_p = None
                best_d = float('inf')
                for p in temp_people:
                    pred = p.predictPos(dt=1)
                    d = getDist(point, pred)

                    speed, _ = p.getVel()
                    dynamic_gate = self.max_dist_for_person + min(1.0, 0.8 * speed)

                    if d <= dynamic_gate and d < best_d:
                        best_d = d
                        best_p = p

                if best_p is not None:
                    best_p.updatePos(point)
                    assigned_groups.append(point)
                else:
                    self.people.append(Person(self.curr_id, point))
                    self.curr_id += 1
                    # assigned_groups.append(point)
            # Merge close people (if person within 0.4 of another remove one that has existed for less time)
            merged = []
            for i, p1 in enumerate(self.people):
                for j, p2 in enumerate(self.people):
                    if i >= j: 
                        continue
                    if getDist(p1.curr_pos, p2.curr_pos) < 0.4:
                        if len(p1.pos) >= len(p2.pos):
                            self.people.remove(p2)
                        else:
                            self.people.remove(p1)
                        break


            updated_ids = {p.id for p in self.people if p.curr_pos in assigned_groups}
            valid_ids = {p.id for p in self.people}
            self.missed_counts = {pid: c for pid, c in self.missed_counts.items() if pid in valid_ids}
            #match points to people now, if any unmatched, then this person has "disappeared" so predict their path
            unmatched_points = [pt for pt in grouped_points if pt not in assigned_groups]
            for p in self.people[:]:
                if len(p.pos) < 3:
                    continue
                if p.id not in updated_ids:
                    reattached = False
                    for pt in unmatched_points:
                        d = getDist(pt, p.curr_pos)
                        if d < self.max_dist_for_person * 1.1: #If point within certain distance and in-line with prev velocity assign to person
                            vel_r, vel_theta = p.getVel()
                            if vel_r == 0:
                                angle_ok = True
                            else:
                                pred_dir = math.atan2(pt.y - p.curr_pos.y, pt.x - p.curr_pos.x)
                                angle_diff = abs((vel_theta - pred_dir + math.pi) % (2 * math.pi) - math.pi)
                                angle_ok = angle_diff < (math.pi / 6)

                            if angle_ok:
                                p.updatePos(pt)
                                unmatched_points.remove(pt)
                                reattached = True
                                break


                    if reattached:
                        continue
                        
                    #if reattached, then we can skip but if not then we must interpolate

                    self.missed_counts[p.id] = self.missed_counts.get(p.id, 0) + 1

                    if self.missed_counts[p.id] <= self.max_missed_frames:
                        coeff = 0.8 ** self.missed_counts[p.id]
                        vel_r, vel_theta = p.getVel()
                        vel_r *= coeff

                        pred = Point()
                        pred.x = p.curr_pos.x + vel_r * math.cos(vel_theta)
                        pred.y = p.curr_pos.y + vel_r * math.sin(vel_theta)
                        pred.z = 0.0

                        drift = getDist(pred, p.curr_pos)
                        if drift < 0.5:
                            p.updatePos(pred)
                        else:
                            self.get_logger().info(
                                f"Interpolation doesn't make sense: too large of drift"
                            )

                    else:
                        self.get_logger().info(f"Removing person {p.id} (missed {self.missed_counts[p.id]} frames)")
                        self.people.remove(p)
                        del self.missed_counts[p.id]
                else:
                    self.missed_counts[p.id] = 0


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
        marker.pose.position.z = 0.01
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.03
        marker.scale.y = 0.01
        marker.scale.z = 0.0

        id_mod_4 = person.id % 4
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.color.r = person.id%3 / 3
        marker.color.g = person.id%5 / 5
        marker.color.b = person.id%7 / 7
        # if(id_mod_4 == 0):
        #     marker.color.r = 1.0
        # elif(id_mod_4 == 1):
        #     marker.color.g = 1.0
        # elif(id_mod_4 == 2):
        #     marker.color.b = 1.0
        # elif(id_mod_4 == 3):
        #     marker.color.g = 1.0
        #     marker.color.b = 1.0
        marker.points = person.pos

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_arr.append(marker)


def getDist(point1:Point, point2:Point):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def main():
    rclpy.init()

    mark = Sim_Marker()

    rclpy.spin(mark)

    mark.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
     
