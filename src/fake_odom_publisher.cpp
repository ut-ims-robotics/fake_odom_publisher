#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double dt;
double delta_x;
double delta_y;
double delta_th;

bool msg_received = false;

// Callback for processing cmd_vel messages
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
    vx = vel_cmd.linear.x;
    vy = vel_cmd.linear.y;
    vth = vel_cmd.angular.z;

    msg_received = true;
    ROS_DEBUG("Recieved cmd_vel message: vx [%f]; vy [%f]; vth [%f]", vx, vy, vth);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_odom_publisher");

    // Publishers and subscribers
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, cmd_vel_callback);

    // Get frame names from the parameter server
    std::string odom_frame;
    std::string base_frame;

    n.param<std::string>("/fake_odom_publisher/odom_frame", odom_frame, "odom");
    n.param<std::string>("/fake_odom_publisher/base_frame", base_frame, "base_footprint");

    ROS_INFO("frames from the parameter server: odom_frame = [%s]; base_frame = [%s]",
              odom_frame.c_str(), base_frame.c_str());

    // Transform stuff
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;
    tf::StampedTransform frameTransform;

    // Timekeeping variables
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(30.0);
    
    ROS_INFO("Starting main loop ...");
    // Main loop
    while(n.ok())
    {
        // Check for incoming messages
        ros::spinOnce();
        current_time = ros::Time::now();

        // Compute odometry in a typical way given the velocities of the robot
        if (msg_received == true)
        {
            // Calcualate the steps
            dt = (current_time - last_time).toSec();
            delta_x = vx * dt;
            delta_y = vy * dt;
            delta_th = vth * dt;

            // Integrate the steps
            x += delta_x;
            y += delta_y;
            th += delta_th;

            msg_received = false;
        }
        else
        {
            delta_x = 0;
            delta_y = 0;
            delta_th = 0;
        }
        
        /* * * * * * * *
         *
         * Publish "/delta_frame", which is used to get do vel_cmd integration transforms.
         * Basically TF node does not need to know about the "/delta_frame", but it was
         * easier and quicker for me to implement it.
         *
         * HOW IT SHOULD BE DONE:
         *
         *      1) Keep TF (/odom -> /base_footprint) in memory, calculate the
         *         TF (/base_footprint -> /delta_frame), multiply them, set
         *         it to be the new TF (/odom -> /base_footprint), publish the TF.
         *
         *      2) Change the TF node's source to support setting transforms in child frame (!).
         *
         *                  /map -> /odom -> /base_footprint -> ...
         *                                          |
         *                                     /delta_frame
         * * * * * * * */

        geometry_msgs::Quaternion delta_quat = tf::createQuaternionMsgFromYaw(delta_th);

        // First, we'll publish the transform over tf
        geometry_msgs::TransformStamped delta_trans;
        delta_trans.header.stamp = current_time;
        delta_trans.header.frame_id = base_frame;
        delta_trans.child_frame_id = "delta_frame";

        delta_trans.transform.translation.x = delta_x;
        delta_trans.transform.translation.y = delta_y;
        delta_trans.transform.translation.z = 0.0;
        delta_trans.transform.rotation = delta_quat;

        // DEBUG delta_frame TF
        ROS_DEBUG("Sending TF: px [%f]; py [%f] | qx [%f]; qy [%f]; qz [%f]; qw [%f] | from [%s] to [%s] \n",
                  delta_trans.transform.translation.x, delta_trans.transform.translation.y,
                  delta_trans.transform.rotation.x, delta_trans.transform.rotation.y,
                  delta_trans.transform.rotation.z, delta_trans.transform.rotation.w,
                  delta_trans.header.frame_id.c_str(), delta_trans.child_frame_id.c_str());

        // Send the transform
        tf_broadcaster.sendTransform(delta_trans);

        /* * * * * * * *
         *
         * Get the transformation /odom to /delta_frame
         *
         * * * * * * * */

        bool transformRecieved = false;
        ROS_DEBUG("Trying to get the transformation from [%s] to [delta_frame] \n", odom_frame.c_str());

        try
        {
            tf_listener.lookupTransform(odom_frame, "delta_frame", ros::Time(0), frameTransform);
            transformRecieved = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Could not get the transformation: %s It should appear soon, proceeding ...", ex.what());
        }

        /* * * * * * * *
         *
         * Publish tf from /odom to /base_footprint
         *
         * * * * * * * */

        // Get the quaternion from "frameTransform" and store it into "odom_quat"
        geometry_msgs::Quaternion odom_quat;
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = odom_frame;
        odom_trans.child_frame_id = base_frame;

        if (transformRecieved)
        {
          tf::Quaternion tf_odom_quat = frameTransform.getRotation().normalized();

          odom_quat.x = (double)tf_odom_quat.getX();
          odom_quat.y = (double)tf_odom_quat.getY();
          odom_quat.z = (double)tf_odom_quat.getZ();
          odom_quat.w = (double)tf_odom_quat.getW();

          odom_trans.transform.translation.x = frameTransform.getOrigin().x();
          odom_trans.transform.translation.y = frameTransform.getOrigin().y();
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;
        }
        else
        {
          odom_quat.x = 0;
          odom_quat.y = 0;
          odom_quat.z = 0;
          odom_quat.w = 1;

          odom_trans.transform.translation.x = 0;
          odom_trans.transform.translation.y = 0;
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;
        }

        // DEBUG odom_frame TF
        ROS_DEBUG("Sending TF: px [%f]; py [%f] | qx [%f]; qy [%f]; qz [%f]; qw [%f] | from [%s] to [%s] \n",
                  odom_trans.transform.translation.x, odom_trans.transform.translation.y,
                  odom_trans.transform.rotation.x, odom_trans.transform.rotation.y,
                  odom_trans.transform.rotation.z, odom_trans.transform.rotation.w,
                  odom_trans.header.frame_id.c_str(), odom_trans.child_frame_id.c_str());

        // Send the transform
        tf_broadcaster.sendTransform(odom_trans);

        /* * * * * * * *
         *
         * Provide odometry data for move_base node
         *
         * * * * * * * */

        // Create a "nav_msgs::Odometry" message and start filling it
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame;

        // Set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // Set the velocity
        odom.child_frame_id = base_frame;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // DEBUG "nav_msgs::Odometry odom"
        ROS_DEBUG("Sending odometry to move_base: px [%f]; py [%f] | qx [%f]; qy [%f]; qz [%f]; qw [%f] | to [%s] \n",
                  odom.pose.pose.position.x, odom.pose.pose.position.y,
                  odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                  odom.pose.pose.orientation.z, odom.pose.pose.orientation.w,
                  odom.header.frame_id.c_str());

        // Publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}
