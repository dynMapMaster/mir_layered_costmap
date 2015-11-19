#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <initializer_list>

#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG
using geometry_msgs::Pose;
using std::vector;
using std::string;
typedef std::pair<double,double> Point2D;
void poseCb(const nav_msgs::Odometry msg);

ros::Publisher cmd_pub;
unsigned waypoint_i = 0;
std::vector<Point2D> waypoints;
double speed, goal_tolerance, time_at_waypoints;

int main(int argc, char** argv)
{
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
    // Set the logging level manually to DEBUG
    ROSCONSOLE_AUTOINIT;
    log4cxx::LoggerPtr my_logger =
    log4cxx::Logger::getLogger( ROSCONSOLE_DEFAULT_NAME );
    my_logger->setLevel(
    ros::console::g_level_lookup[ros::console::levels::Debug]
    );

#endif
    ros::init(argc, argv, "dynamic_obstacle");
    ros::NodeHandle n("~");
    n.param<double>("translational_speed", speed, 0.5);
    if(speed < 0.0)
    {
        speed = DBL_MAX;
    }
    n.param<double>("goal_tolerance", goal_tolerance, 0.001);
    n.param<double>("time_at_waypoints", time_at_waypoints,0.5);
    {
        ROS_DEBUG("Retriving waypoints");

        XmlRpc::XmlRpcValue my_list;
        n.getParam("waypoints", my_list);
        ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < my_list.size(); ++i)
        {
            ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(my_list[i].size() == 2);
            Point2D point;
            point.first = static_cast<double>(my_list[i][0]);
            point.second = static_cast<double>(my_list[i][1]);
            waypoints.push_back(point);
            ROS_DEBUG("(%f, %f)",point.first, point.second);
        }
    }
    ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1, &poseCb);
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::spin();
    return 0;
}

void poseCb(const nav_msgs::Odometry msg)
{

    Pose robot_pose = msg.pose.pose;
    double dx = waypoints[waypoint_i].first - robot_pose.position.x;
    double dy = waypoints[waypoint_i].second - robot_pose.position.y;
    double dist_from_goal = hypot(dx, dy);
    double vx, vy;
    if(dist_from_goal < goal_tolerance)
    {
        waypoint_i++;
        waypoint_i %= waypoints.size();
        vx = 0; vy = 0;
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = vx;
        cmd_vel_msg.linear.y = vy;
        cmd_pub.publish(cmd_vel_msg);
        ros::Duration d(time_at_waypoints);
        d.sleep();
    }
    else
    {
        double dir_angle = atan2(dy,dx);
        vx = cos(dir_angle) * speed;
        vy = sin(dir_angle) * speed;
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = vx;
        cmd_vel_msg.linear.y = vy;
        cmd_pub.publish(cmd_vel_msg);
    }

}
