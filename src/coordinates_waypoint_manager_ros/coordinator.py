#!/usr/bin/env python
import rospy
import actionlib
from geodesy import utm
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Empty

class Waypoint(object):
  def __init__(self, id, latitude, longitude):
    self.id = id
    self.latitude = latitude
    self.longitude = longitude

class GPSConverter(object):

  def __init__(self):

    rospy.init_node('waypoint_coordinator', anonymous=True)

    if not rospy.has_param("move_base_node"):
      rospy.logerr("Missing parameter with move base node name")
      return

    # Initialize action server of move base
    move_base_node_name = rospy.get_param("move_base_node")

    self.mb_client = actionlib.SimpleActionClient(move_base_node_name, MoveBaseAction)
    self.mb_client.wait_for_server()

    # Initialize tf
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # Variables
    self.frame_id = rospy.get_param("goal_frame_id")
    goal_sub_topic_name = rospy.get_param("goal_sub_topic_name")
    goal_pub_topic_name = rospy.get_param("result_pub_topic_name")
    self.waypoints_list = []
    self.id = 0


    # Publishers
    # Publishes result of goal
    self.goal_pub = rospy.Publisher(goal_pub_topic_name, String, queue_size=5)

    # Subscribers
    # Subscribes to goal requests
    self.goal_sub = rospy.Subscriber(goal_sub_topic_name, NavSatFix, self.goalSubCb)
    self.reset_sub = rospy.Subscriber("reset_waypoints", Empty, self.resetCb)

    self.rate = rospy.Rate(10)

  def execute(self):
    while not rospy.is_shutdown():
      # Manage waypoint list
      if len(self.waypoints_list) > 0:
        # Pops the oldest goal to send to move base
        curr_goal = self.waypoints_list.pop(0)
        msg = "id:" + str(curr_goal.id) + ";latitude:" + str(curr_goal.latitude) + ";longitude:" + str(curr_goal.longitude)
        rospy.loginfo(msg)
        self.goal_pub.publish(String(data=msg))
        result = self.waypoint_pub(curr_goal.latitude, curr_goal.longitude)
        msg = "id:" + str(curr_goal.id) + ";result:" + str(result)
        rospy.loginfo(msg)
        self.goal_pub.publish(String(data=msg))
      self.rate.sleep()

  def waypoint_pub(self, lat, lon):
    utm_conversion = utm.fromLatLong(lat,lon)
    
    rospy.logdebug(utm_conversion)
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "utm"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = utm_conversion.easting
    goal.target_pose.pose.position.y = utm_conversion.northing
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 0.0

    try:
      transform = self.tf_buffer.lookup_transform(self.frame_id,
                                       goal.target_pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(2.0)) #wait for 1 second

      goal.target_pose = tf2_geometry_msgs.do_transform_pose(goal.target_pose, transform)
      #goal.target_pose.pose.position.x = -goal.target_pose.pose.position.x
      print(goal.target_pose) 
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0
      rospy.loginfo("Goal transformed\n", goal)
      # Sends the goal to the action server
      self.mb_client.send_goal(goal)
      # Waits for the server to finish performing the action.
      self.mb_client.wait_for_result()
      # Prints out the result of executing the action
      return self.mb_client.get_result()
    except:
      rospy.logerr("Couldnt find transform from utm to " + self.frame_id)
      return None
  
  def goalSubCb(self, data):
    self.id += 1
    new_waypoint = Waypoint(id=self.id, latitude=data.latitude, longitude=data.longitude)
    self.waypoints_list.append(new_waypoint)

  def resetCb(self):
    self.waypoints_list = []
    self.mb_client.cancel_all_goals()


def main():
  ic = GPSConverter()
  ic.execute()

if __name__ == '__main__':
    main()