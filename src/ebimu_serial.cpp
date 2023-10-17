/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include "ros/init.h"
#include <cstddef>
#include <serial/serial.h>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

serial::Serial ser;
using namespace std;

class imuSerial
{
public:
  imuSerial() : nh_(""), nh_priv_("~"), spinner_(0), pub1_(), pub2_(), imuTimer_(), timer_()
  {
    // Log
    ROS_INFO_STREAM("Start to initialize robofrien serial node.");

    // Spinner
    spinner_.start();

    // Load parameters
    if (!nh_priv_.getParam("imu_port", port_))
    {
      port_ = "/dev/ttyUSB0";
      ROS_WARN_STREAM("Parameter missing: imu_port, set default: " << port_);
    }
    if (!nh_priv_.getParam("imu_baud_rate", baud_rate_))
    {
      baud_rate_ = 115200;
      ROS_WARN_STREAM(
          "Parameter missing: imu_baud_rate, set default: " << baud_rate_);
    }
    if (!nh_priv_.getParam("imu_rate", imuRate_))
    {
      imuRate_ = 10;
      ROS_WARN_STREAM("Parameter missing: imu rate, set default: " << imuRate_);
    }
    if (!nh_priv_.getParam("covariance/x", cov_x_))
    {
      cov_x_ = 0.1;
      ROS_WARN_STREAM(
          "Parameter missing: covariance/x, set default: " << cov_x_);
    }
    if (!nh_priv_.getParam("covariance/y", cov_y_))
    {
      cov_y_ = 0.1;
      ROS_WARN_STREAM(
          "Parameter missing: covariance/y, set default: " << cov_y_);
    }
    if (!nh_priv_.getParam("covariance/z", cov_z_))
    {
      cov_z_ = 0.1;
      ROS_WARN_STREAM(
          "Parameter missing: covariance/z, set default: " << cov_z_);
    }

    if (!nh_priv_.getParam("robot_num", robot_num_))
    {
      robot_num_ = 1;
      ROS_WARN_STREAM(
          "Parameter missing: robot_num, set default: " << robot_num_);
    }
    if (!nh_priv_.getParam("yawSet", yawSet_))
    {
      yawSet_ = 0;
      ROS_WARN_STREAM(
          "Parameter missing: yawSet_, set default: " << yawSet_);
    }

    // Publisher
    pub1_ = nh_.advertise<nav_msgs::Odometry>("imuData_robot1", 1000);
    pub2_ = nh_.advertise<nav_msgs::Odometry>("imuData_robot2", 1000);

    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);

    // Timer
    imuTimer_ = nh_.createTimer(ros::Rate(imuRate_), &imuSerial::imuTimer_callback,
                                this);
    imuTimer_.stop();
    timer_.stop();
  }

  virtual ~imuSerial()
  {
    imuTimer_.stop();
    pub1_.shutdown();
    pub2_.shutdown();
    spinner_.stop();
    nh_priv_.shutdown();
    nh_.shutdown();
  }
  virtual bool init_serial()
  {
    try
    {
      ser_.setPort(port_);
      ser_.setBaudrate(baud_rate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR_STREAM("Unable to open port ");
      return false;
    }

    if (ser_.isOpen())
    {
      ROS_INFO_STREAM("Serial Port initialized. (EBIMU)");
      return true;
    }
    else
    {
      return false;
    }
  }

  virtual void startImu() { imuTimer_.start(); }
  virtual void startTimer() { timer_.start(); }

private:
  virtual void imuTimer_callback(const ros::TimerEvent &event)
  {
    if (ser_.available())
    {
      imu_data();
    }
  }

  virtual void imu_data()
  {
    std_msgs::String result;
    result.data = ser_.read(ser_.available());
    vector<string> split_data;
    istringstream ss(result.data);
    string stringBuffer;
    int cnt = 0;

    cout << "split data: " << endl;
    while (getline(ss, stringBuffer, ','))
    {
      split_data.push_back(stringBuffer);
      cout << split_data[cnt++] << ", ";
    }
    cout << endl;
    cout << "split_data size: " << split_data.size() << endl;
    cout <<"robot_num_: "<< robot_num_<<endl;

    if (robot_num_ == 1)
    {
      if (split_data.size() == 8)
      {
        pub1_odom1(split_data);
      }
    }
    else if (robot_num_ == 2)
    {
      if (split_data.size() == 15)
      {
        pub1_odom2(split_data);
      }
    }
  }

  virtual void pub1_odom1(vector<string> split_data)
  {
    double yaw1 = std::stof(split_data[3]);
    if (yawSet_ == 1)
    {
      past_yaw1 += std::stof(split_data[3]);
      yawSet_ = 0;
    }

    yaw1 -= past_yaw1;
    if (yaw1 > 180)
      yaw1 -= 360;
    else if (yaw1 <= -180)
      yaw1 += 360;
    cout << "ID: " << std::stof(split_data[0]) << endl;

    nav_msgs::Odometry odomMsg;
    odomMsg.header.stamp = ros::Time::now();
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";
    odomMsg.twist.twist.angular.x = std::stof(split_data[1]); // imu - roll
    odomMsg.twist.twist.angular.y = std::stof(split_data[2]); // imu - pitch
    odomMsg.twist.twist.angular.z = yaw1;                     // imu - yaw
    odomMsg.twist.twist.linear.x = std::stof(split_data[4]);  // imu - x velocity
    odomMsg.twist.twist.linear.y = std::stof(split_data[5]);  // imu - y velocity
    odomMsg.twist.twist.linear.z = std::stof(split_data[6]);  // imu - z velocity

    ROS_DEBUG_STREAM(odomMsg.pose.pose);
    odomMsg.pose.covariance[0] = cov_x_;
    odomMsg.pose.covariance[7] = cov_y_;
    odomMsg.pose.covariance[14] = cov_z_;
    pub1_.publish(odomMsg);
  }
  virtual void pub1_odom2(vector<string> split_data)
  {
    double yaw1 = std::stof(split_data[3]);
    double yaw2 = std::stof(split_data[10]);
    if (yawSet_ == 1)
    {
      past_yaw1 += std::stof(split_data[3]);
      past_yaw2 += std::stof(split_data[10]);
      yawSet_ = 0;
    }

    yaw1 -= past_yaw1;
    if (yaw1 > 180)
      yaw1 -= 360;
    else if (yaw1 <= -180)
      yaw1 += 360;

    yaw2 -= past_yaw2;
    if (yaw2 > 180)
      yaw2 -= 360;
    else if (yaw2 <= -180)
      yaw2 += 360;
    cout << "ID: " << std::stof(split_data[7]) << endl;
    if (std::stof(split_data[0]) == (50 - 0))
    {
      nav_msgs::Odometry odomMsg1;
      odomMsg1.header.stamp = ros::Time::now();
      odomMsg1.header.frame_id = "odom";
      odomMsg1.child_frame_id = "base_link_robot1";
      odomMsg1.twist.twist.angular.x = std::stof(split_data[1]); // imu - roll
      odomMsg1.twist.twist.angular.y = std::stof(split_data[2]); // imu - pitch
      odomMsg1.twist.twist.angular.z = yaw1;                     // imu - yaw
      odomMsg1.twist.twist.linear.x = std::stof(split_data[4]);  // imu - x velocity
      odomMsg1.twist.twist.linear.y = std::stof(split_data[5]);  // imu - y velocity
      odomMsg1.twist.twist.linear.z = std::stof(split_data[6]);  // imu - z velocity

      ROS_DEBUG_STREAM(odomMsg1.pose.pose);
      odomMsg1.pose.covariance[0] = cov_x_;
      odomMsg1.pose.covariance[7] = cov_y_;
      odomMsg1.pose.covariance[14] = cov_z_;
      pub1_.publish(odomMsg1);


      nav_msgs::Odometry odomMsg2;
      odomMsg2.header.stamp = ros::Time::now();
      odomMsg2.header.frame_id = "odom";
      odomMsg2.child_frame_id = "base_link_robot2";
      odomMsg2.twist.twist.angular.x = std::stof(split_data[8]); // imu - roll
      odomMsg2.twist.twist.angular.y = std::stof(split_data[9]); // imu - pitch
      odomMsg2.twist.twist.angular.z = yaw2;                     // imu - yaw
      odomMsg2.twist.twist.linear.x = std::stof(split_data[11]); // imu - x velocity
      odomMsg2.twist.twist.linear.y = std::stof(split_data[12]); // imu - y velocity
      odomMsg2.twist.twist.linear.z = std::stof(split_data[13]); // imu - z velocity
      ROS_DEBUG_STREAM(odomMsg2.pose.pose);
      odomMsg2.pose.covariance[0] = cov_x_;
      odomMsg2.pose.covariance[7] = cov_y_;
      odomMsg2.pose.covariance[14] = cov_z_;
      pub2_.publish(odomMsg2);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::AsyncSpinner spinner_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Timer imuTimer_;
  ros::Timer timer_;

  serial::Serial ser_;
  string port_;
  int baud_rate_;
  int robot_num_;
  double imuRate_; // Timer callback rate
  double cov_x_;   // Timer callback rate
  double cov_y_;   // Timer callback rate
  double cov_z_;   // Timer callback rate
  int yawSet_;

  double past_yaw1 = 0;
  double past_yaw2 = 0;
};

void write_callback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("Writing to serial port" << msg->data);
  // ser.write(msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ebimu_serial_node");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("serial example node start");
  imuSerial ebimu;
  bool ret = ebimu.init_serial();
  if (ret == false)
    return -1;

  ebimu.startImu();
  ros::waitForShutdown();

  return 0;
}
