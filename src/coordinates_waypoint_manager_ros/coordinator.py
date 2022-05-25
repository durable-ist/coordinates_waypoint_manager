#!/usr/bin/env python
import rospy
import actionlib
from geodesy import utm
import tf
import tf2_ros
import tf2_geometry_msgs
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Empty
from coordinates_waypoint_manager.msg import WaypointRequest

class Waypoint(object):
  def __init__(self, id, latitude, longitude, orientation):
    self.id = id
    self.latitude = latitude
    self.longitude = longitude
    self.orientation = orientation

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
    cmd_vel_topic_name = rospy.get_param("cmd_vel_topic")
    self.param_goal_queue_ns = "~goal_queue/"
    self.waypoints_list = []
    self.id = 0
    self.curr_goal = None
    self.pause = False
    self.repush_goal = False


    # Publishers
    # Publishes result of goal
    self.goal_pub = rospy.Publisher(goal_pub_topic_name, String, queue_size=5)
    self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size=5)

    # Subscribers
    # Subscribes to goal requests
    # self.goal_sub = rospy.Subscriber(goal_sub_topic_name, NavSatFix, self.goalSubCb)
    self.goal_sub = rospy.Subscriber(goal_sub_topic_name, WaypointRequest, self.goalSubCb)
    self.reset_sub = rospy.Subscriber("reset_waypoints", Empty, self.resetCb)
    self.pause_sub = rospy.Subscriber("pause", Empty, self.pauseCb)
    self.play_sub = rospy.Subscriber("play", Empty, self.playCb)

    self.rate = rospy.Rate(10)

  def execute(self):
    while not rospy.is_shutdown():
      self.showList()
      # Manage waypoint list
      if len(self.waypoints_list) > 0 and not self.pause:
        # Pops the oldest goal to send to move base
        curr_goal = self.waypoints_list.pop(0)
        msg = "id:" + str(curr_goal.id) + ";latitude:" + str(curr_goal.latitude) + ";longitude:" + str(curr_goal.longitude) + ";orientation:" + str(curr_goal.orientation)

        rospy.loginfo(msg)
        self.goal_pub.publish(String(data=msg))
        
        self.curr_goal = curr_goal
        self.repush_goal = True
        result = self.waypoint_pub(curr_goal.latitude, curr_goal.longitude, curr_goal.orientation)

        msg = "id:" + str(curr_goal.id) + ";result:" + str(result)
        rospy.loginfo(msg)

        self.goal_pub.publish(String(data=msg))
      elif len(self.waypoints_list) == 0 and self.repush_goal and not self.pause:
        self.repush_goal = False

      self.rate.sleep()

  def waypoint_pub(self, lat, lon, orientation):
    rospy.loginfo("lat: " + str(lat)+ " ;lon: " + str(lon) + " ; " + self.frame_id)
    utm_conversion = utm.fromLatLong(lat,lon)

    # rospy.loginfo(utm_conversion)

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
    except:
      rospy.logerr("Couldnt find transform from utm to " + self.frame_id)
      return None
    rospy.loginfo("Converted")
    rospy.loginfo(goal.target_pose)

    # convert orientation to quaternion
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, orientation)

    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    # rospy.loginfo("Goal transformed")
    # rospy.loginfo(goal)

    # Sends the goal to the action server
    self.mb_client.send_goal(goal)
    # Waits for the server to finish performing the action.
    self.mb_client.wait_for_result()
    # Prints out the result of executing the action
    return self.mb_client.get_state()

  def showList(self):
    if rospy.has_param(self.param_goal_queue_ns):
      rospy.delete_param(self.param_goal_queue_ns)
    for goal in self.waypoints_list:
      name = self.param_goal_queue_ns + "goal" + str(goal.id)
      rospy.set_param(name, str(goal.latitude) + ";" + str(goal.longitude))
  
  def goalSubCb(self, data):
    self.id += 1
    new_waypoint = Waypoint(id=self.id, latitude=data.latitude, longitude=data.longitude, orientation=data.orientation)
    msg = "goal_accepted:" + str(self.id)
    self.goal_pub.publish(msg)
    self.waypoints_list.append(new_waypoint)
    self.showList()

  def resetCb(self, data):
    self.waypoints_list = []
    self.mb_client.cancel_all_goals()
    # Clear goal params
    self.showList()

    self.pause = True

    # send cmd velocity 0 to stop for sure
    cmd_msg = Twist()
    cmd_msg.linear.x = 0
    cmd_msg.linear.y = 0
    cmd_msg.linear.z = 0
    cmd_msg.angular.x = 0
    cmd_msg.angular.y = 0
    cmd_msg.angular.z = 0

    self.goal_pub.publish(String(data="reset"))
    rospy.logwarn("Waypoint List Cleared!")

    time_start = rospy.Time.now()

    # send 0s for x time to force him to stay put while move base stops
    while rospy.Time.now() - time_start < rospy.Duration(2.5):
      self.cmd_vel_pub.publish(cmd_msg)

  
  def pauseCb(self, data):
    self.pause = True
    self.mb_client.cancel_all_goals()

    # send cmd velocity 0 to stop for sure
    cmd_msg = Twist()
    cmd_msg.linear.x = 0
    cmd_msg.linear.y = 0
    cmd_msg.linear.z = 0
    cmd_msg.angular.x = 0
    cmd_msg.angular.y = 0
    cmd_msg.angular.z = 0

    self.goal_pub.publish(String(data="pause"))
    rospy.logwarn("Movement Paused!")

    time_start = rospy.Time.now()

    # send 0s for x time to force him to stay put while move base stops
    while rospy.Time.now() - time_start < rospy.Duration(2.5):
      self.cmd_vel_pub.publish(cmd_msg)


  def playCb(self, data):
    if self.repush_goal:
      self.waypoints_list.insert(0,self.curr_goal)
      self.repush_goal = False
    self.pause = False
    self.goal_pub.publish(String(data="play"))
    rospy.logwarn("To infinity and beyond!")


def main():
  ic = GPSConverter()
  ic.execute()

if __name__ == '__main__':
    main()