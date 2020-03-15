/*
  Author: Modulabs
  File Name: legged_robot_teleop_joystick.cpp
*/

#include "legged_robot_teleop/legged_robot_teleop_joystick.h"


LeggedRobotTeleopJoystick::LeggedRobotTeleopJoystick()
: _nodeHandle("")
{
   _velPub = _nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  _joyCommandSub = _nodeHandle.subscribe("joy", 10, &LeggedRobotTeleopJoystick::joyCommandCallback, this);
  _uiCommandSrv = _nodeHandle.serviceClient<legged_robot_msgs::UICommand>("/hyq/main_controller/ui_command");
}

LeggedRobotTeleopJoystick::~LeggedRobotTeleopJoystick()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void LeggedRobotTeleopJoystick::joyCommandCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->axes.at(1) >= 0.9) setGoal("linear.x+");
  else if(msg->axes.at(1) <= -0.9) setGoal("linear.x-");
  else if(msg->axes.at(0) >=  0.9) setGoal("linear.y+");
  else if(msg->axes.at(0) <= -0.9) setGoal("linear.y-");
  else if(msg->axes.at(5) >=  0.9) setGoal("linear.z+");
  else if(msg->axes.at(5) <= -0.9) setGoal("linear.z-");
  else if(msg->axes.at(2) >=  0.9) setGoal("angular.z+");
  else if(msg->axes.at(2) <= -0.9) setGoal("angular.z-");
  else if(msg->axes.at(3) <= -0.9) setGoal("Stop");
  else if(msg->axes.at(4) <= -0.9) setGoal("FallRecovery");
  else if(msg->buttons.at(3) == 1) setGoal("VSD");
  else if(msg->buttons.at(0) == 1) setGoal("z-");
  else if(msg->buttons.at(5) == 1) setGoal("w+");
  else if(msg->buttons.at(4) == 1) setGoal("w-");
  else if(msg->buttons.at(2) == 1) setGoal("a+");
  else if(msg->buttons.at(1) == 1) setGoal("a-");
}

void LeggedRobotTeleopJoystick::setGoal(const char* str)
{
  geometry_msgs::Twist twist;
  if(str == "linear.x+")
  {
    twist.linear.x = 1.0;
    _velPub.publish(twist);
  }
  else if(str == "linear.x-")
  {
    twist.linear.x = -1.0;
    _velPub.publish(twist);
  }
  else if(str == "linear.y+")
  {
    twist.linear.y = 1.0;
    _velPub.publish(twist);
  }
  else if(str == "linear.y-")
  {
    twist.linear.y = -1.0;
    _velPub.publish(twist);
  }
  else if(str == "linear.z+")
  {
    twist.linear.z = 1.0;
    _velPub.publish(twist);
  }
  else if(str == "linear.z-")
  {
    twist.linear.z = -1.0;
    _velPub.publish(twist);
  }
  else if(str == "angular.z+")
  {
    twist.angular.z = 1.0;
    _velPub.publish(twist);
  }
  else if(str == "angular.z-")
  {
    twist.angular.z = -1.0;
    _velPub.publish(twist);
  }
  else if(str == "Stop")
  {
    _velPub.publish(twist);
  }
  else if(str == "FallRecovery")
  {
    sendCommand("ChgCtrl", "FallRecovery");
  }
  else if(str == "VSD")
  {
    sendCommand("ChgCtrl", "VSD");
  }
  else if(str == "z-")
  {
    sendCommand("ChgCtrl", "BalQP");
    // sendCommand("ChgCtrl", "BalMPC");
    // sendCommand("ChgCtrl", "BalMPCWB");
  }
  else if(str == "w+")
  {
    sendCommand("Body", "", 0);
    // sendCommand("Body", "", 1);
    // sendCommand("Body", "", 2);
  }
  else if(str == "w-")
  {
    sendCommand("Body", "", 3);
  }
  else if(str == "a+")
  {
    sendCommand("Order", "", 0);
  }
  else if(str == "a-")
  {
    sendCommand("Order", "", 1);
  }
}

bool LeggedRobotTeleopJoystick::sendCommand(std::string mainCommand, std::string subCommand, long int paramInt64, double paramFloat64)
{
  legged_robot_msgs::UICommand srv;
  srv.request.main_command = mainCommand;
  srv.request.sub_command = subCommand;
  srv.request.param_int64 = paramInt64;
  srv.request.param_float64 = paramFloat64;
  if (_uiCommandSrv.call(srv))
  {
    if (srv.response.result)
      return true;
  }

  ROS_INFO("Failed to send a command");
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "legged_robot_teleop_joystick");
  LeggedRobotTeleopJoystick leggedRobotTeleopJoystick;

  ros::spin();

  return 0;
}
