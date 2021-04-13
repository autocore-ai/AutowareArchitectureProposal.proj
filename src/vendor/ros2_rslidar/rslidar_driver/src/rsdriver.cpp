/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *	Copyright (C) 2019, Dyno Robotics, Fredrik Löfgren
*
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rslidar_driver/rsdriver.hpp"

namespace rslidar_driver
{
  static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 18000;
  static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

  rslidarDriver::rslidarDriver() : rslidarDriver(rclcpp::NodeOptions()) {}

  rslidarDriver::rslidarDriver(const rclcpp::NodeOptions& options) : 
    Node("rsdriver", options), 
    diagnostics_(this)
{
  // this->declare_parameter("frame_id", rclcpp::ParameterValue("rslidar"));
  // this->get_parameter("frame_id", config_.frame_id);

  config_.frame_id = declare_parameter("frame_id", "rslidar");
  RCLCPP_INFO(this->get_logger(), "[driver] LIDAR frame_id: %s", config_.frame_id.c_str());

  this->declare_parameter("time_synchronization", rclcpp::ParameterValue(false));

  skip_num_ = 0;
  // use private node handle to get parameters

  // get model name, validate string, determine packet rate
  // this->get_parameter_or("model", config_.model, std::string("RS32"));
  // this->get_parameter_or("model", config_.model, std::string("RS16"));
  config_.model = declare_parameter("model", "RS16");
  RCLCPP_INFO(this->get_logger(), "[driver] LIDAR model: %s", config_.model.c_str());
  
  devip_str_ = declare_parameter("device_ip", "");
  if (!devip_str_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "[driver] LIDAR ip: %s", devip_str_.c_str());
  }else
  {
    RCLCPP_INFO(this->get_logger(), "[driver] LIDAR ip not available, get data from pcap file");
  }
  
  

  double packet_rate;  // packet frequency (Hz)
  std::string model_full_name;

  // product model
  if (config_.model == "RS16")
  {
    // for 0.18 degree horizontal angle resolution
    packet_rate = 840;
    // for 0.2 degree horizontal angle resolution
    // packet_rate = 750;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    //for 0.18 degree horizontal angle resolution
    packet_rate = 1690;
    //for 0.2 degree horizontal angle resolution
    // packet_rate = 1500;
    model_full_name = "RS-LiDAR-32";
  }
  else if (config_.model == "RSBPEARL")
  {
    packet_rate = 1500;
    model_full_name = "RSBPEARL";
  }
  else if (config_.model == "RSBPEARL_MINI")
  {
    packet_rate = 1500;
    model_full_name = "RSBPEARL_MINI";
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[driver] unknown LIDAR model: %s", config_.model.c_str());
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  this->get_parameter_or("rpm", config_.rpm, 600.0);
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);
  this->get_parameter_or("npackets", config_.npackets, npackets);
  RCLCPP_INFO(this->get_logger(), "[driver] publishing %d packets per scan", config_.npackets);

  std::string dump_file;
  // this->get_parameter_or("pcap", dump_file, std::string(""));
  dump_file = declare_parameter("pcap", "");
  RCLCPP_INFO(this->get_logger(), "[driver] pcap file path: %s", dump_file.c_str());

  int msop_udp_port;
  // this->get_parameter_or("msop_port", msop_udp_port, (int)5566);// (int)MSOP_DATA_PORT_NUMBER);
  // this->get_parameter_or("msop_port", msop_udp_port, (int)6699);// (int)MSOP_DATA_PORT_NUMBER);
  msop_udp_port = declare_parameter("msop_port", (int)6699);
  RCLCPP_INFO(this->get_logger(), "[driver] LIDAR msop port: %d", msop_udp_port);

  int difop_udp_port;
  // this->get_parameter_or("difop_port", difop_udp_port, (int)8899);// (int)DIFOP_DATA_PORT_NUMBER);
  // this->get_parameter_or("difop_port", difop_udp_port, (int)7788);// (int)DIFOP_DATA_PORT_NUMBER);
  difop_udp_port = declare_parameter("difop_port", (int)7788);
  RCLCPP_INFO(this->get_logger(), "[driver] LIDAR difop port: %d", difop_udp_port);

  double cut_angle;
  this->get_parameter_or("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0)
  {
    RCLCPP_INFO(this->get_logger(), "[driver] Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < 360)
  {
    RCLCPP_INFO(this->get_logger(), "[driver] Cut at specific angle feature activated. Cutting rslidar points always at %f degree.", cut_angle);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[driver] cut_angle parameter is out of range. Allowed range is between 0.0 and 360 negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in rslidar packets
  config_.cut_angle = static_cast<int>(cut_angle * 100);

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  // Setup callback for changes to parameters.
  auto sub = parameters_client->on_parameter_event(
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
      on_parameter_event(event, this->get_logger());
    });

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  // RCLCPP_INFO("expected frequency: %.3f (Hz)", diag_freq);

  diag_topic_.reset(new diagnostic_updater::TopicDiagnostic("rslidar_packets", diagnostics_,
                                                            diagnostic_updater::FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                                                            diagnostic_updater::TimeStampStatusParam()));

  // open rslidar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    msop_input_.reset(new rslidar_driver::InputPCAP(this, msop_udp_port, packet_rate, dump_file));
    difop_input_.reset(new rslidar_driver::InputPCAP(this, difop_udp_port, packet_rate, dump_file));
  }
  else
  {
    // read data from live socket
    msop_input_.reset(new rslidar_driver::InputSocket(this, msop_udp_port, devip_str_));
    difop_input_.reset(new rslidar_driver::InputSocket(this, difop_udp_port, devip_str_));
  }

  // raw packet output topic
  std::string output_packets_topic;
  this->get_parameter_or("output_packets_topic", output_packets_topic, std::string("rslidar_packets"));
  msop_output_ = this->create_publisher<rslidar_msgs::msg::RslidarScan>(output_packets_topic, 10);

  std::string output_difop_topic;
  this->get_parameter_or("output_difop_topic", output_difop_topic, std::string("rslidar_packets_difop"));
  difop_output_ = this->create_publisher<rslidar_msgs::msg::RslidarPacket>(output_difop_topic, 10);

  difop_thread_ = std::thread(std::bind(&rslidarDriver::difopPollThread, this));
  poll_thread_ = std::thread(std::bind(&rslidarDriver::pollThread, this));



  //difop_poll_timer_ = this->create_wall_timer(0s, std::bind(&rslidarDriver::difopPoll, this));
  //poll_timer_ = this->create_wall_timer(0s, std::bind(&rslidarDriver::poll, this));
  
  this->get_parameter("time_synchronization", time_synchronization_);

  if (time_synchronization_)
  {
    output_sync_ = this->create_publisher<sensor_msgs::msg::TimeReference>("sync_header", 1);
    skip_num_sub_ = this->create_subscription<std_msgs::msg::Int32>("skippackets_num", 1, std::bind(&rslidarDriver::skipNumCallback, this, _1));
  }
}


rslidarDriver::~rslidarDriver()
{
  // Make sure to join the thread on shutdown.
  if (difop_thread_.joinable()) {
    difop_thread_.join();
  }
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
}

void rslidarDriver::pollThread(void)
{
  while (rclcpp::ok())
  {
    this->poll();
  }
}

void rslidarDriver::difopPollThread(void)
{
  // reading and publishing scans as fast as possible.
  while (rclcpp::ok())
  {
    this->difopPoll();
  }
}
  
/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool rslidarDriver::poll(void)
{  
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  rslidar_msgs::msg::RslidarScan::UniquePtr scan(new rslidar_msgs::msg::RslidarScan());

  // Since the rslidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);
    rslidar_msgs::msg::RslidarPacket tmp_packet;
    while (true)
    {
      while (true)
      {
        int rc = msop_input_->getPacket(tmp_packet, config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      scan->packets.push_back(tmp_packet);
      
      static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
      static int last_azimuth = ANGLE_HEAD;

      int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
      // int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      // Handle overflow 35999->0
      if (azimuth < last_azimuth)
      {
        last_azimuth -= 36000;
      }
      // Check if currently passing cut angle
      if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
      {
        last_azimuth = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;
    }
  }
  else  // standard behaviour
  {
    if (difop_input_->getUpdateFlag())
    {
      int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND/BLOCKS_ONE_CHANNEL_PER_PKT);
      int mode = difop_input_->getReturnMode();
      if (config_.model == "RS16" && (mode == 1 || mode == 2))
      {
        packets_rate = ceil(packets_rate/2);
      }
      else if ((config_.model == "RS32" || config_.model == "RSBPEARL" || config_.model == "RSBPEARL_MINI") && (mode == 0))
      {
        packets_rate = packets_rate*2;
      }
      config_.rpm = difop_input_->getRpm();
      config_.npackets = ceil(packets_rate*60/config_.rpm);

      difop_input_->clearUpdateFlag();

      RCLCPP_INFO(this->get_logger(), "[driver] update npackets. rpm: %f, npkts: %d", config_.rpm, config_.npackets);
    }
    scan->packets.resize(config_.npackets);
    // use in standard behaviour only
    while (skip_num_)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(scan->packets[0], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      --skip_num_;
    }

    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(scan->packets[i], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
    }

    if (time_synchronization_)
    {
      sensor_msgs::msg::TimeReference sync_header;
      // it is already the msop msg
      // if (pkt->data[0] == 0x55 && pkt->data[1] == 0xaa && pkt->data[2] == 0x05 && pkt->data[3] == 0x0a)
      // use the first packets
      rslidar_msgs::msg::RslidarPacket pkt = scan->packets[0];
      struct tm stm;
      memset(&stm, 0, sizeof(stm));
      stm.tm_year = (int)pkt.data[20] + 100;
      stm.tm_mon  = (int)pkt.data[21] - 1;
      stm.tm_mday = (int)pkt.data[22];
      stm.tm_hour = (int)pkt.data[23];
      stm.tm_min  = (int)pkt.data[24];
      stm.tm_sec  = (int)pkt.data[25];

      // FIXME(sam): This puts seconds in the nanosecond field, 1 in seconds and appears not synced with computer time
      // double stamp_double = mktime(&stm) + 0.001 * (256 * pkt.data[26] + pkt.data[27]) +
      //                       0.000001 * (256 * pkt.data[28] + pkt.data[29]);
      // sync_header.header.stamp = rclcpp::Time(stamp_double);

      sync_header.header.stamp = get_clock()->now();

      output_sync_->publish(sync_header);
    }
  }

  // publish message using time of last packet read
  // RCLCPP_DEBUG("[driver] Publishing a full rslidar scan.");

  // FIXME(sam): This puts seconds in the nanosecond field, 1 in seconds and appears not synced with computer time
  // scan->header.stamp = scan->packets.back().stamp;

  scan->header.stamp = get_clock()->now();

  scan->header.frame_id = config_.frame_id;


  // notify diagnostics that a message has been published, updating its status
  diag_topic_->tick(scan->header.stamp);

  //std::weak_ptr<std::remove_pointer<decltype(msop_output_.get())>::type> captured_pub = msop_output_;
  //auto pub_ptr = captured_pub.lock();
  //pub_ptr->publish(std::move(scan));

  msop_output_->publish(std::move(scan));

  return true;
}

void rslidarDriver::difopPoll(void)
{

  // keep reading
  rslidar_msgs::msg::RslidarPacket::UniquePtr difop_packet_ptr(new rslidar_msgs::msg::RslidarPacket());
  int rc = difop_input_->getPacket(*difop_packet_ptr, config_.time_offset);
  if (rc == 0)
  {
    //RCLCPP_DEBUG("[driver] Publishing a difop data.");
    difop_output_->publish(std::move(difop_packet_ptr));
  }
  if (rc < 0)
    return;  // end of file reached?

}


void rslidarDriver::on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "[driver] Reconfigure Request");

  for (auto & changed_parameter : event->changed_parameters) {
    if (changed_parameter.name == "time_offset"){
      config_.time_offset = changed_parameter.value.double_value;
    }
  }
}





  
// add for time synchronization
void rslidarDriver::skipNumCallback(const std_msgs::msg::Int32::SharedPtr skip_num)
{
  // std::cout << "Enter skipNumCallback: " << skip_num->data << std::endl;
  skip_num_ = skip_num->data;
}
}  // namespace rslidar_driver
