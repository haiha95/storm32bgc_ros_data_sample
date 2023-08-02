/*
  Publish all data
*/

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "storm32bgc_ros_data_sample_node");
  ros::NodeHandle nh;
  ros::Rate rate(60.f);

  ros::Publisher gimbal_state_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/state", 4);
  ros::Publisher gimbal_status_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/status", 4);
  ros::Publisher gimbal_status2_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/status2", 4);
  ros::Publisher gimbal_errors_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/errors", 4);
  ros::Publisher gimbal_lipo_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/lipo", 4);
  ros::Publisher gimbal_timestamp_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/timestamp", 4);
  ros::Publisher gimbal_cycle_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/cycle", 4);
  ros::Publisher gimbal_gx_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/gx", 4);
  ros::Publisher gimbal_gy_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/gy", 4);
  ros::Publisher gimbal_gz_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/gz", 4);
  ros::Publisher gimbal_ax_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/ax", 4);
  ros::Publisher gimbal_ay_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/ay", 4);
  ros::Publisher gimbal_az_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/az", 4);
  ros::Publisher gimbal_rx_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/rx", 4);
  ros::Publisher gimbal_ry_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/ry", 4);
  ros::Publisher gimbal_rz_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/rz", 4);
  ros::Publisher gimbal_imu1pitch_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/imu1pitch", 4);
  ros::Publisher gimbal_imu1roll_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/imu1roll", 4);
  ros::Publisher gimbal_imu1yaw_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/imu1yaw", 4);
  ros::Publisher gimbal_pidpitch_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/pidpitch", 4);
  ros::Publisher gimbal_pidroll_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/pidroll", 4);
  ros::Publisher gimbal_pidyaw_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/pidyaw", 4);
  ros::Publisher gimbal_rcpitch_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/rcpitch", 4);
  ros::Publisher gimbal_rcroll_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/rcroll", 4);
  ros::Publisher gimbal_rcyaw_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/rcyaw", 4);
  ros::Publisher gimbal_imu2pitch_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/imu2pitch", 4);
  ros::Publisher gimbal_imu2roll_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/imu2roll", 4);
  ros::Publisher gimbal_imu2yaw_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/imu2yaw", 4);
  ros::Publisher gimbal_mag2yaw_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/mag2yaw", 4);
  ros::Publisher gimbal_mag2pitch_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/mag2pitch", 4);
  ros::Publisher gimbal_ahrsconf_pub_ = nh.advertise<std_msgs::Float32>("storm32bgc/ahrsconf", 4);
  ros::Publisher gimbal_fcninput_pub_ = nh.advertise<std_msgs::Int16>("storm32bgc/fcninput", 4);

  serial::Serial serial;
  try
  {
    serial.setPort("/dev/ttyACM0");
    serial.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial.setTimeout(timeout);
    serial.open();
  }
  catch (serial::IOException &exception)
  {
    ROS_ERROR("Unable to open port.");
    ros::shutdown();
    return -1;
  }
  if (serial.isOpen()) ROS_INFO("Serial port initialized.");
  else
  {
    ROS_INFO("Unable to open port");
    return -1;
  }
  serial.flush();
  /* After connected serial, keep sending a "t" and looking for an "o" */
  bool connection_established = false;
  while (!connection_established)
  {
    serial.write("t");
    while (!serial.waitReadable());
    uint8_t byte_read[1];
    serial.read(byte_read, 1);
    if (byte_read[0] == 'o')
    {
      ROS_INFO("Connection established.");
      connection_established = true;
    }
    else ROS_INFO_THROTTLE(0.5f, "Waiting for connection...");
  }

  const uint8_t CMD_GET_DATA_[6] = {0xFA, 0x01, 0x05, 0x00, 0x3F, 0x73};
  uint8_t buffer[99];
  while (ros::ok())
  {
    serial.flush();
    serial.write(CMD_GET_DATA_, 6);
    while (!serial.waitReadable());
    uint8_t byte_read[1];
    uint8_t index = 0;
    while (serial.available() && index < 99)
    {
      serial.read(byte_read, 1);
      buffer[index] = byte_read[0];
      index++;
    }
    if (buffer[0] != 0xFB || buffer[1] != 0x42 || buffer[2] != 0x05)
      continue;
    /* Publish every data in sequence */
    std_msgs::Int16 state; state.data = static_cast<int16_t>(buffer[6] << 8 | buffer[5]);
    gimbal_state_pub_.publish(state);
    std_msgs::Int16 status; status.data = static_cast<int16_t>(buffer[8] << 8 | buffer[7]);
    gimbal_status_pub_.publish(status);
    std_msgs::Int16 status2; status2.data = static_cast<int16_t>(buffer[10] << 8 | buffer[9]);
    gimbal_status2_pub_.publish(status2);
    std_msgs::Int16 errors; errors.data = static_cast<int16_t>(buffer[12] << 8 | buffer[11]);
    gimbal_errors_pub_.publish(errors);
    std_msgs::Int16 lipo; lipo.data = static_cast<int16_t>(buffer[14] << 8 | buffer[13]);
    gimbal_lipo_pub_.publish(lipo);
    std_msgs::Int16 timestamp; timestamp.data = static_cast<int16_t>(buffer[16] << 8 | buffer[15]);
    gimbal_timestamp_pub_.publish(timestamp);
    std_msgs::Float32 cycle; cycle.data = static_cast<int16_t>(buffer[18] << 8 | buffer[17])/1000000.f;
    gimbal_cycle_pub_.publish(cycle);
    std_msgs::Int16 gx; gx.data = static_cast<int16_t>(buffer[20] << 8 | buffer[19]);
    gimbal_gx_pub_.publish(gx);
    std_msgs::Int16 gy; gy.data = static_cast<int16_t>(buffer[22] << 8 | buffer[21]);
    gimbal_gy_pub_.publish(gy);
    std_msgs::Int16 gz; gz.data = static_cast<int16_t>(buffer[24] << 8 | buffer[23]);
    gimbal_gz_pub_.publish(gz);
    std_msgs::Float32 ax; ax.data = static_cast<int16_t>(buffer[26] << 8 | buffer[25])/10000.f;
    gimbal_ax_pub_.publish(ax);
    std_msgs::Float32 ay; ay.data = static_cast<int16_t>(buffer[28] << 8 | buffer[27])/10000.f;
    gimbal_ay_pub_.publish(ay);
    std_msgs::Float32 az; az.data = static_cast<int16_t>(buffer[30] << 8 | buffer[29])/10000.f;
    gimbal_az_pub_.publish(az);
    std_msgs::Float32 rx; rx.data = static_cast<int16_t>(buffer[32] << 8 | buffer[31])/10000.f;
    gimbal_rx_pub_.publish(rx);
    std_msgs::Float32 ry; ry.data = static_cast<int16_t>(buffer[34] << 8 | buffer[33])/10000.f;
    gimbal_ry_pub_.publish(ry);
    std_msgs::Float32 rz; rz.data = static_cast<int16_t>(buffer[36] << 8 | buffer[35])/10000.f;
    gimbal_rz_pub_.publish(rz);
    std_msgs::Float32 imu1pitch; imu1pitch.data = static_cast<int16_t>(buffer[38] << 8 | buffer[37])/100.f;
    gimbal_imu1pitch_pub_.publish(imu1pitch);
    std_msgs::Float32 imu1roll; imu1roll.data = static_cast<int16_t>(buffer[40] << 8 | buffer[39])/100.f;
    gimbal_imu1roll_pub_.publish(imu1roll);
    std_msgs::Float32 imu1yaw; imu1yaw.data = static_cast<int16_t>(buffer[42] << 8 | buffer[41])/100.f;
    gimbal_imu1yaw_pub_.publish(imu1yaw);
    std_msgs::Float32 pidpitch; pidpitch.data = static_cast<int16_t>(buffer[44] << 8 | buffer[43])/100.f;
    gimbal_pidpitch_pub_.publish(pidpitch);
    std_msgs::Float32 pidroll; pidroll.data = static_cast<int16_t>(buffer[46] << 8 | buffer[45])/100.f;
    gimbal_pidroll_pub_.publish(pidroll);
    std_msgs::Float32 pidyaw; pidyaw.data = static_cast<int16_t>(buffer[48] << 8 | buffer[47])/100.f;
    gimbal_pidyaw_pub_.publish(pidyaw);
    std_msgs::Int16 rcpitch; rcpitch.data = static_cast<int16_t>(buffer[50] << 8 | buffer[49]);
    gimbal_rcpitch_pub_.publish(rcpitch);
    std_msgs::Int16 rcroll; rcroll.data = static_cast<int16_t>(buffer[52] << 8 | buffer[51]);
    gimbal_rcroll_pub_.publish(rcroll);
    std_msgs::Int16 rcyaw; rcyaw.data = static_cast<int16_t>(buffer[54] << 8 | buffer[53]);
    gimbal_rcyaw_pub_.publish(rcyaw);
    std_msgs::Float32 imu2pitch; imu2pitch.data = static_cast<int16_t>(buffer[56] << 8 | buffer[55])/100.f;
    gimbal_imu2pitch_pub_.publish(imu2pitch);
    std_msgs::Float32 imu2roll; imu2roll.data = static_cast<int16_t>(buffer[58] << 8 | buffer[57])/100.f;
    gimbal_imu2roll_pub_.publish(imu2roll);
    std_msgs::Float32 imu2yaw; imu2yaw.data = static_cast<int16_t>(buffer[60] << 8 | buffer[59])/100.f;
    gimbal_imu2yaw_pub_.publish(imu2yaw);
    std_msgs::Float32 mag2yaw; mag2yaw.data = static_cast<int16_t>(buffer[62] << 8 | buffer[61])/100.f;
    gimbal_mag2yaw_pub_.publish(mag2yaw);
    std_msgs::Float32 mag2pitch; mag2pitch.data = static_cast<int16_t>(buffer[64] << 8 | buffer[63])/100.f;
    gimbal_mag2pitch_pub_.publish(mag2pitch);
    std_msgs::Float32 ahrsconf; ahrsconf.data = static_cast<int16_t>(buffer[66] << 8 | buffer[65])/10000.f;
    gimbal_ahrsconf_pub_.publish(ahrsconf);
    std_msgs::Int16 fcninput; fcninput.data = static_cast<int16_t>(buffer[68] << 8 | buffer[67]);
    gimbal_fcninput_pub_.publish(fcninput);

    ros::spinOnce();
    rate.sleep();
  }

  serial.close();
  return 0;
}