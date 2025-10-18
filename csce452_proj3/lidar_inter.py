import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
import math
from sklearn.cluster import DBSCAN


#TODO: Determine the sections from the LIDAR that are dynamic
    #NOTE: Can separate dynamic sections by comparing with a static map taken as initial measurement
    #NOTE: Update the past range as a rolling avg? (need to deal with flickering wall values)
class Lidar_Inter(Node):
    def __init__(self):
        super().__init__('interface')
        self.lidar = self.create_subscription(LaserScan, '/scan', self.getVals, 10)
        self.marker_push = self.create_publisher(PoseArray, '/moved_positions', 10)
        self.lidar_ranges: list[list[float]] = []
        self.past_lidar_range: list[list[float]] = []
        self.twice_past_range: list[list[float]] = [] #DEBUG: REMOVE maybe
        self.original_range: list[list[float]] = []

        #NOTE: Old values, keeping for testing purposes
        # self.move_threshold = 0.05 #m
        # self.eps = 0.75
        # self.points_for_group = 5 #T
        # self.group_threshold = 10 #Min size of group to count as a person
        self.move_threshold = 0.05 #how much a point needs to move from previous position to be considered a change
        self.eps = 0.4
        self.points_for_group = 3 #TODO: fine tune, keep high to cut out noise, low enough so we get all people movement (has issue when far from lidar)
        self.group_threshold = 5 #Min size of group to count as a person

    
    def pointPub(self, point_to_pub:list[Point]):
        self.get_logger().info("Publishing different points")
        pose_arr:list[Pose] = []
        for point in point_to_pub:
            pose_arr.append(Pose(position=point))
        msg = PoseArray()
        msg.poses = pose_arr[:]
        self.marker_push.publish(msg)

    def getVals(self, msg):
        max_range = msg.range_max
        min_range = msg.range_min
        max_angle = msg.angle_max #TODO: if no use, delete
        min_angle = msg.angle_min
        angle_step = msg.angle_increment
        self.lidar_ranges = []
        for i in range(len(msg.ranges)):
            if(math.isnan(msg.ranges[i]) or msg.ranges[i] < min_range or msg.ranges[i] > max_range): #Filter out values outside of range
                self.lidar_ranges.append(None)
                continue 
            r = msg.ranges[i]
            theta = min_angle + angle_step*i
            self.lidar_ranges.append(polarToCartesian(r, theta))
        
        #TODO: filter out the stationary points
        # if(self.past_lidar_range == []):
        #     self.past_lidar_range = self.lidar_ranges[:]
        #     return 
        if(self.twice_past_range == []):
            self.twice_past_range = self.past_lidar_range[:]
            self.past_lidar_range = self.lidar_ranges[:]
            if(self.twice_past_range != []):
                #Average the first two scans to get a stationary map scan
                for i in range(len(self.past_lidar_range)):
                    if(self.past_lidar_range[i] == None) or (self.twice_past_range[i] == None):
                        self.original_range.append(None)
                    else:
                        self.original_range.append([(self.twice_past_range[i][0] + self.past_lidar_range[i][0] )/ 2, (self.twice_past_range[i][1] + self.past_lidar_range[i][1] )/ 2])
            return 
        else:
            diff_points: list[list [float]] = []
            
            for i in range(min(len(self.lidar_ranges), len(self.past_lidar_range))):
                if(self.lidar_ranges[i] == None): continue
                elif(self.past_lidar_range[i] != None) and (self.twice_past_range[i] == None):
                    dist =  math.sqrt((self.lidar_ranges[i][1] - self.past_lidar_range[i][1])**2 + (self.lidar_ranges[i][0] - self.past_lidar_range[i][0])**2)
                    dist_2 = self.move_threshold + 1
                elif(self.past_lidar_range[i] == None) and (self.twice_past_range[i] == None):
                    continue
                elif(self.past_lidar_range[i] == None):
                    continue
                else: 
                    dist = math.sqrt((self.lidar_ranges[i][1] - self.past_lidar_range[i][1])**2 + (self.lidar_ranges[i][0] - self.past_lidar_range[i][0])**2)
                    dist_2 = math.sqrt((self.lidar_ranges[i][1] - self.twice_past_range[i][1])**2 + (self.lidar_ranges[i][0] - self.twice_past_range[i][0])**2) #DEBUG: remove maybe
                
                if(self.original_range[i] == None):
                    change_from_orig = self.move_threshold+1
                else:
                    change_from_orig = math.sqrt((self.lidar_ranges[i][0] - self.original_range[i][0])**2 + (self.lidar_ranges[i][1] - self.original_range[i][1])**2) - self.move_threshold
                
                is_new_value = (abs(dist) > self.move_threshold) and (abs(dist_2) > self.move_threshold) and (change_from_orig >= self.move_threshold) #Has point moved further than the threshold distance and is closer than the previous point on that line?
                if(is_new_value):
                    if(self.lidar_ranges[i] == None): self.get_logger().info("ERROR: Appending NONE value")
                    elif(self.lidar_ranges[i][0] == float('nan')): self.get_logger().info("ERROR: contains NONE value")
                    diff_points.append(self.lidar_ranges[i])
            self.get_logger().info(f"got diffs, {len(diff_points)}") #TODO: Remove
            for diff in diff_points:
                self.get_logger().info(f"Difference: ({diff[0]}, {diff[1]})") #TODO:REMOVE
            self.twice_past_range = self.past_lidar_range[:] 
            self.past_lidar_range = self.lidar_ranges[:]
            
            
            #Convert to Point array
            point_arr: list[Point] = []
            for point in diff_points:
                point_arr.append(Point(x=point[0], y=point[1], z=0.0))
            
            grouped_points = self.groupPoints(point_arr)
            self.pointPub(grouped_points)
    
    # Forms the points into groups based on proximity, uses the DBSCAN algorithm
    def groupPoints(self, points: list[Point]): 
        temp_list = []
        for i in points:
            temp_list.append([i.x, i.y])
        if(temp_list == []):
            return []
        list_np = np.array(temp_list)
        db = DBSCAN(eps=self.eps, min_samples=self.points_for_group)
        fitted_list = db.fit(list_np)
        #NOTE: fitted_list.labels_ will give me a list with the associated labels for each point
        
        seen_groups = {}
        group_sums = {}
        group_points: list[Point] = []
        labels, count = np.unique(fitted_list.labels_, return_counts=True)
        label_count = dict(zip(labels, count))

        for i in range(len(points)):
            label = fitted_list.labels_[i]
            num_occurences = np.sum(fitted_list.labels_ == label)

            if(label == -1): continue #noise
            elif (seen_groups.get(label) == None) and (num_occurences > self.group_threshold): #First time seeing this group and group is of large enough size
                seen_groups.update({label: True})
                # group_points.append(points[i])
                temp_point = Point(x=points[i].x, y=points[i].y)
                group_sums.update({label: temp_point})
            elif(seen_groups.get(label) == True):
                prev_point = group_sums.get(label)
                
                temp_point = Point(x=(points[i].x + prev_point.x), y=(points[i].y + prev_point.y))
                group_sums[label] = temp_point
        
        for label in group_sums:
            prev_point = group_sums.get(label)
            occurences:int = label_count.get(label)
            temp_point = Point(x=(prev_point.x/occurences), y=(prev_point.y/occurences))
            group_sums[label] = temp_point
            group_points.append(temp_point)
            
            


        
        self.get_logger().info(f"num groups: {len(group_points)}")
        for point in group_points:
            self.get_logger().info(f"Point: ({point.x}, {point.y})") #TODO: REMOVE
        
        return group_points

    



def polarToCartesian(r: float, theta: float):
    x:float = r * math.cos(theta)
    y:float = r * math.sin(theta) #Why are the y-values negated?
    return [x, y]

def cartToPolar(x:float, y:float):
    r = math.sqrt(x**2 + y**2)
    if(x == 0): theta = math.pi/2 * (y/(abs(y)))
    theta = math.atan(y/x)
    return [r, theta]

def main():
    rclpy.init()

    sensor = Lidar_Inter()

    rclpy.spin(sensor)

    sensor.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
