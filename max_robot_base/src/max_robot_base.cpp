#include "max_robot_base/max_robot_base.hpp"

MaxRobotBase::MaxRobotBase(rclcpp::Logger &logger) : logger_(logger) {
  voltage = 0.0f;
  memset(&vel_data, 0, sizeof(vel_data));
  memset(&rx_data_, 0, sizeof(rx_data_));
  memset(&tx_data_, 0, sizeof(tx_data_));
  memset(&imu_data, 0, sizeof(imu_data));
  buffer_.resize(256); // Buffer for multiple bytes
}

MaxRobotBase::~MaxRobotBase() {
  stop();
  if (io_thread_.joinable()) {
    io_service_.stop();
    io_thread_.join();
  }
}

bool MaxRobotBase::initialize_serial(const std::string& port, int baud_rate) {
  RCLCPP_INFO(logger_, "Attempting to open serial port: %s at %d baud rate...", port.c_str(), baud_rate);

  try {
    base_serial_ = std::make_unique<boost::asio::serial_port>(io_service_, port);
    base_serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    base_serial_->set_option(boost::asio::serial_port_base::character_size(8));
    base_serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    base_serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    base_serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    // Start async read
    start_async_read();
    
    // Run io_service in a separate thread
    io_thread_ = std::thread([this]() { io_service_.run(); });
    
    RCLCPP_INFO(logger_, "Serial port: %s opened.", port.c_str());
    return true;
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(logger_, "Serial initialization failed: %s", e.what());
    return false;
  }
}

bool MaxRobotBase::is_serial_opened() {
  return base_serial_ && base_serial_->is_open();
}

void MaxRobotBase::stop() {
  RCLCPP_INFO(logger_, "Sending stop command...");

  tx_data_.tx[0] = FRAME_HEADER;
  tx_data_.tx[1] = 0;
  tx_data_.tx[2] = 0;

  tx_data_.tx[4] = 0;
  tx_data_.tx[3] = 0;

  tx_data_.tx[6] = 0;
  tx_data_.tx[5] = 0;

  tx_data_.tx[8] = 0;
  tx_data_.tx[7] = 0;

  tx_data_.tx[9] = checksum(9, TX_DATA_CHECK);
  tx_data_.tx[10] = FRAME_TAIL;

  try {
    if (is_serial_opened()) {
      base_serial_->write_some(boost::asio::buffer(tx_data_.tx, sizeof(tx_data_.tx)));
    }
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(logger_, "Unable to send data through serial port: %s", e.what());
  }

  if (is_serial_opened()) {
    base_serial_->close();
  }
  RCLCPP_INFO(logger_, "Serial port closed.");
}

void MaxRobotBase::write(float linear_x, float steer_angle) {
  tx_data_.tx[0] = FRAME_HEADER;
  tx_data_.tx[1] = 0;
  tx_data_.tx[2] = 0;

  int16_t data_x = linear_x * 1000;
  tx_data_.tx[4] = data_x;
  tx_data_.tx[3] = data_x >> 8;

  tx_data_.tx[6] = 0;
  tx_data_.tx[5] = 0;

  int16_t data_z = steer_angle * 1000;
  tx_data_.tx[8] = data_z;
  tx_data_.tx[7] = data_z >> 8;

  tx_data_.tx[9] = checksum(9, TX_DATA_CHECK);
  tx_data_.tx[10] = FRAME_TAIL;

  try {
    if (is_serial_opened()) {
      base_serial_->write_some(boost::asio::buffer(tx_data_.tx, sizeof(tx_data_.tx)));
    }
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(logger_, "Unable to send data through serial port: %s", e.what());
  }
}

// count is payload size
unsigned char MaxRobotBase::checksum(unsigned char count, unsigned char mode) {
  unsigned char check_sum = 0;
  for (unsigned char k = 0; k < count; k++) {
    check_sum ^= (mode == 0 ? rx_data_.rx[k] : tx_data_.tx[k]);
  }
  return check_sum;
}

int16_t MaxRobotBase::imu_trans(uint8_t high, uint8_t low) {
  int16_t value = (high << 8) | low;
  return value;
}

float MaxRobotBase::odom_trans(uint8_t high, uint8_t low) {
  int16_t value = (high << 8) | low;
  return (float)(value / 1000 + (value % 1000) * 0.001f);
}

void MaxRobotBase::start_async_read() {
  if (!is_serial_opened()) return;
  
  base_serial_->async_read_some(
      boost::asio::buffer(buffer_, buffer_.size()),
      [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
        if (!ec) {
          std::lock_guard<std::mutex> lock(buffer_mutex_);
          for (size_t i = 0; i < bytes_transferred; ++i) {
            switch (packet_.state) {
              case ParseState::HEADER:
                if (buffer_[i] == FRAME_HEADER) {
                  packet_.data.clear();
                  packet_.data.push_back(buffer_[i]);
                  packet_.index = 1;
                  packet_.state = ParseState::PAYLOAD;
                }
                break;
              case ParseState::PAYLOAD:
                packet_.data.push_back(buffer_[i]);
                packet_.index++;
                if (packet_.index == RX_DATA_SIZE - 2) { // 22 bytes (header + 20 payload)
                  packet_.state = ParseState::CHECKSUM;
                }
                break;
              case ParseState::CHECKSUM:
                packet_.data.push_back(buffer_[i]);
                packet_.index++;
                packet_.state = ParseState::TAIL;
                break;
              case ParseState::TAIL:
                packet_.data.push_back(buffer_[i]);
                if (buffer_[i] == FRAME_TAIL && validate_checksum()) {
                  process_packet();
                  data_ready_ = true;
                } else {
                  RCLCPP_WARN(logger_, "Invalid packet: tail (0x%02X) or checksum mismatch", buffer_[i]);
                }
                packet_.state = ParseState::HEADER;
                break;
            }
          }
          start_async_read(); // Continue reading
        } else {
          RCLCPP_ERROR(logger_, "Serial read error: %s", ec.message().c_str());
        }
      });
}

bool MaxRobotBase::validate_checksum() {
  std::memcpy(rx_data_.rx, packet_.data.data(), RX_DATA_SIZE);
  uint8_t check = checksum(22, RX_DATA_CHECK);
  return check == packet_.data[22];
}

void MaxRobotBase::process_packet() {
  rx_data_.frame_header = packet_.data[0];
  rx_data_.flag_stop = packet_.data[1];
  rx_data_.frame_tail = packet_.data[23];

  vel_data.linear_x = odom_trans(packet_.data[2], packet_.data[3]);
  vel_data.linear_y = odom_trans(packet_.data[4], packet_.data[5]);
  vel_data.angular_z = odom_trans(packet_.data[6], packet_.data[7]);

  imu_data.accel_x = imu_trans(packet_.data[8], packet_.data[9]) / ACCEl_RATIO;
  imu_data.accel_y = imu_trans(packet_.data[10], packet_.data[11]) / ACCEl_RATIO;
  imu_data.accel_z = imu_trans(packet_.data[12], packet_.data[13]) / ACCEl_RATIO;
  imu_data.gyro_x = imu_trans(packet_.data[14], packet_.data[15]) * GYRO_RATIO;
  imu_data.gyro_y = imu_trans(packet_.data[16], packet_.data[17]) * GYRO_RATIO;
  imu_data.gyro_z = imu_trans(packet_.data[18], packet_.data[19]) * GYRO_RATIO;

  int16_t voltage_data = (packet_.data[20] << 8) | packet_.data[21];
  voltage = (float)(voltage_data / 1000 + (voltage_data % 1000) * 0.001f);
}

bool MaxRobotBase::read() {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (data_ready_) {
    data_ready_ = false; // Reset for next packet
    return true;
  }
  return false;
}
