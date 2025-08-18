import rospy

class GpsGoal:
    def __init__(self):
        self.gps_goal_service = rospy.Service('gps_goal', LatLon, self.handle_gps_goal_service)
    

    def handle_gps_goal_service(self):
        origin_lat = self.start_point_lat
        origin_long = self.start_point_lon

        #origin_lat = 41.1052672
        #origin_long = 29.0233784

        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
        azimuth = g['azi1']
        rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

        rospy.loginfo(f"x: {x}, y: {y}")