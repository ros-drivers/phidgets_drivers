#include "phidgets_ik/ik_ros_i.h"

namespace phidgets {

IKRosI::IKRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
  IK(),
  n_in(0),
  n_out(0),
  n_sensors(0),
  nh_(nh),
  nh_private_(nh_private),
  VREF(5.0f)
{
  ROS_INFO ("Starting Phidgets IK");

  initDevice();
}

void IKRosI::initDevice()
{
  ROS_INFO("Opening device");
  int serial_num;
  if (!nh_private_.getParam("serial", serial_num))
    serial_num = -1; //defalut open any device
  open(serial_num);

  ROS_INFO("Waiting for IK %d to be attached...", serial_num);
  int result = waitForAttachment(10000);
  if(result)
  {
    const char *err;
    CPhidget_getErrorDescription(result, &err);
    ROS_FATAL("Problem waiting for IK attachment: %s", err);
    ros::shutdown();
    return;
  }

  CPhidgetInterfaceKit_getInputCount(ik_handle_, &n_in);
  CPhidgetInterfaceKit_getOutputCount(ik_handle_, &n_out);
  CPhidgetInterfaceKit_getSensorCount(ik_handle_, &n_sensors);
  ROS_INFO("%d inputs, %d outputs, %d sensors", n_in, n_out, n_sensors);
  for (int i = 0; i < n_in; i++) {
    char topicname[] = "digital_input00";
    snprintf(topicname, sizeof(topicname), "digital_input%02d", i);
    in_pubs_.push_back(nh_.advertise<std_msgs::Bool>(topicname, 1));
  }
  for (int i = 0; i < n_out; i++) {
    char topicname[] = "digital_output00";
    snprintf(topicname, sizeof(topicname), "digital_output%02d", i);
    boost::shared_ptr<OutputSetter> s (new OutputSetter(ik_handle_, i));
    s->subscription = nh_.subscribe(topicname, 1, &OutputSetter::set_msg_callback, s);
    out_subs_.push_back(s);
  }
  out_srv_ = nh_.advertiseService("set_digital_output", &IKRosI::set_srv_callback, this);
  for (int i = 0; i < n_sensors; i++) {
    char topicname[] = "analog_input00";
    snprintf(topicname, sizeof(topicname), "analog_input%02d", i);
    sensor_pubs_.push_back(nh_.advertise<std_msgs::Float32>(topicname, 1));
  }
}

void IKRosI::sensorHandler(int index, int sensorValue)
{
  //get rawsensorvalue and divide by 4096, which according to the documentation
  //for both the IK888 and IK222 are the maximum sensor value
  //Multiply by VREF=5.0V to get voltage
  int rawval = 0;
  CPhidgetInterfaceKit_getSensorRawValue(ik_handle_, index, &rawval);
  std_msgs::Float32 msg;
  msg.data = VREF*float(rawval)/4095.0f;
  if ((static_cast<int>(sensor_pubs_.size()) > index) && (sensor_pubs_[index])) {
    sensor_pubs_[index].publish(msg);
  }
}

void IKRosI::inputHandler(int index, int inputValue)
{
  std_msgs::Bool msg;
  msg.data = inputValue != 0;
  if ((static_cast<int>(in_pubs_.size()) > index) && (in_pubs_[index])) {
    in_pubs_[index].publish(msg);
  }
}

bool IKRosI::set_srv_callback(phidgets_ik::SetDigitalOutput::Request& req,
  phidgets_ik::SetDigitalOutput::Response &res)
{
  ROS_INFO("Setting output %d to %d", req.index, req.state);
  res.success = !CPhidgetInterfaceKit_setOutputState(ik_handle_, req.index, req.state);
  return true;
}

void OutputSetter::set_msg_callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Setting output %d to %d", index, msg->data);
  CPhidgetInterfaceKit_setOutputState(ik_handle_, index, msg->data);
}

OutputSetter::OutputSetter(CPhidgetInterfaceKitHandle ik_handle, int index)
{
    this->ik_handle_ = ik_handle;
    this->index = index;
}

} // namespace phidgets
