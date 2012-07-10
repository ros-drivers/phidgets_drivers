#include "phidgets_imu/phidgets_imu.h"

namespace phidgets {

PhidgetsImu::PhidgetsImu(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  handle_id_(0)
{
  ROS_INFO ("Starting Phidgets IMU");

  // **** get parameters

  if (!nh_private_.getParam ("rate", rate_))
    rate_ = 8; // 8 ms
  if (!nh_private_.getParam ("frame_id", frame_id_))
    frame_id_ = "imu";
  if (!nh_private_.getParam ("angular_velocity_stdev", angular_velocity_stdev_))
    angular_velocity_stdev_ = 0.02 * (M_PI / 180.0); // 0.02 deg/s resolution, as per manual
  if (!nh_private_.getParam ("linear_acceleration_stdev", linear_acceleration_stdev_))
    linear_acceleration_stdev_ = 300.0 * 1e-6 * G; // 300 ug as per manual

  // **** advertise topics

  imu_publisher_ = nh_.advertise<ImuMsg>(
    "imu/data_raw", 5);
  mag_publisher_ = nh_.advertise<MagMsg>(
    "imu/mag", 5);
  cal_publisher_ = nh_.advertise<std_msgs::Bool>(
    "imu/is_calibrated", 5);

  // **** advertise services

  cal_srv_ = nh_.advertiseService(
    "imu/calibrate", &PhidgetsImu::calibrate, this);

  // **** initialize variables and device

  orientation_.setRPY(0.0, 0.0, 0.0);
  
  imu_msg_.header.frame_id = frame_id_;

  // build covariance matrices

  double ang_vel_var = angular_velocity_stdev_ * angular_velocity_stdev_;
  double lin_acc_var = linear_acceleration_stdev_ * linear_acceleration_stdev_;

  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
  {
    int idx = j*3 +i;

    if (i == j)
    {
      imu_msg_.angular_velocity_covariance[idx]    = ang_vel_var;
      imu_msg_.linear_acceleration_covariance[idx] = lin_acc_var;
    }
    else
    {
      imu_msg_.angular_velocity_covariance[idx]    = 0.0;
      imu_msg_.linear_acceleration_covariance[idx] = 0.0;
    }
  }

  initDevice();
}

PhidgetsImu::~PhidgetsImu()
{
	ROS_INFO("Closing connection to Phidget IMU...\n");
	CPhidget_close ((CPhidgetHandle)handle_id_);
	CPhidget_delete((CPhidgetHandle)handle_id_);
	ROS_INFO("Phidget IMU closed.\n");
}

void PhidgetsImu::initDevice()
{
	//create the spatial object
	CPhidgetSpatial_create(&handle_id_);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)handle_id_, AttachHandler, this);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)handle_id_, DetachHandler, this);
	CPhidget_set_OnError_Handler ((CPhidgetHandle)handle_id_, ErrorHandler,  this);

	//Registers a callback that will run according to the set data rate that will return the spatial data changes
	//Requires the handle for the Spatial, the callback handler function that will be called, 
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetSpatial_set_OnSpatialData_Handler(handle_id_, SpatialDataHandler, this);

	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)handle_id_, -1);

	//get the program to wait for a spatial device to be attached
	int result;
	ROS_INFO("Waiting for IMU to be attached...");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)handle_id_, 10000)))
	{
	  const char *err;
		CPhidget_getErrorDescription(result, &err);
		ROS_FATAL("Problem waiting for IMU attachment: %s", err);
	}

  // assuming magnetic field bounds are the same for all axes
  //CPhidgetSpatial_getMagneticFieldMin((CPhidgetSpatialHandle)handle_id_, 0, &field_min_);
  //CPhidgetSpatial_getMagneticFieldMax((CPhidgetSpatialHandle)handle_id_, 0, &field_max_);

	//Set the data rate for the spatial events
	CPhidgetSpatial_setDataRate(handle_id_, rate_);

  // zero out
  zero();
}

void PhidgetsImu::zero()
{
  // zero (calibrate) gyro
  ROS_INFO("Calibrating IMU...");
  CPhidgetSpatial_zeroGyro(handle_id_);
  ROS_INFO("Calibrating IMU done.");

  // publish message
  std_msgs::Bool is_calibrated_msg;
  is_calibrated_msg.data = true;
  cal_publisher_.publish(is_calibrated_msg);
}

bool PhidgetsImu::calibrate(std_srvs::Empty::Request  &req,
                            std_srvs::Empty::Response &res)
{
  zero();
  return true;
}

void PhidgetsImu::processImuData(CPhidgetSpatial_SpatialEventDataHandle * data, int count)
{
  ros::Time time_now = ros::Time::now();

  if (!initialized_)
  { 
    last_imu_time_ = time_now;
    initialized_ = true;
  }

  for(int i = 0; i < count; i++)
	{
    // **** create and publish imu message

    boost::shared_ptr<ImuMsg> imu_msg = 
      boost::make_shared<ImuMsg>(imu_msg_);

    imu_msg->header.stamp = time_now;

    // set linear acceleration
    imu_msg->linear_acceleration.x = - data[i]->acceleration[0] * G;
    imu_msg->linear_acceleration.y = - data[i]->acceleration[1] * G;
    imu_msg->linear_acceleration.z = - data[i]->acceleration[2] * G;

    // set angular velocities
    imu_msg->angular_velocity.x = data[i]->angularRate[0] * (M_PI / 180.0);
    imu_msg->angular_velocity.y = data[i]->angularRate[1] * (M_PI / 180.0);
    imu_msg->angular_velocity.z = data[i]->angularRate[2] * (M_PI / 180.0);

    // integrate the angular velocities
    float dt = (time_now - last_imu_time_).toSec();
    last_imu_time_ = time_now;

    tf::Quaternion dq;
    dq.setRPY(data[i]->angularRate[0]*dt,
              data[i]->angularRate[1]*dt,
              data[i]->angularRate[2]*dt);

    orientation_ = dq * orientation_;

    imu_publisher_.publish(imu_msg);

    // **** create and publish magnetic field message

    boost::shared_ptr<MagMsg> mag_msg = 
      boost::make_shared<MagMsg>();
    
    mag_msg->header.frame_id = frame_id_;
    mag_msg->header.stamp = time_now;

    if (data[i]->magneticField[0] != PUNK_DBL)
    {
      mag_msg->vector.x = data[i]->magneticField[0];
      mag_msg->vector.y = data[i]->magneticField[1];
      mag_msg->vector.z = data[i]->magneticField[2];
    }
    else
    {
      double nan = std::numeric_limits<double>::quiet_NaN();

      mag_msg->vector.x = nan;
      mag_msg->vector.y = nan;
      mag_msg->vector.z = nan;
    }
     
    mag_publisher_.publish(mag_msg);
	}
}

int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serial_no;
	CPhidget_getSerialNumber(spatial, &serial_no);
	ROS_INFO("IMU Phidget attached (serial# %d)", serial_no);
	return 0;
}

int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serial_no;
	CPhidget_getSerialNumber(spatial, &serial_no);
	ROS_INFO("IMU Phidget detached (serial# %d)", serial_no);
	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	ROS_WARN("Phidget Error, code: %d\n", ErrorCode);
	return 0;
}

int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  PhidgetsImu * imu = (PhidgetsImu *) userptr;
  imu->processImuData(data, count);
	return 0;
}

} //namespace phidgets

