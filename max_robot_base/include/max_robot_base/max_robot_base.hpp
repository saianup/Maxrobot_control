#ifndef __MAX_ROBOT_BASE_H_
#define __MAX_ROBOT_BASE_H_

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <mutex>
#include <vector>

#define TX_DATA_CHECK   1           // Send data check flag bits 
#define RX_DATA_CHECK   0           // Receive data to check flag bits
#define RX_DATA_SIZE    24          // The length of the data sent by the lower computer
#define TX_DATA_SIZE    11          // The length of data sent by ROS to the lower machine
#define FRAME_HEADER    0X7B        // Frame head
#define FRAME_TAIL      0X7D        // Frame tail
#define ACCEl_RATIO     1671.84f
#define GYRO_RATIO      0.00026644f

typedef struct {
  float linear_x;
  float linear_y;
  float angular_z;
} Vel_Data;

typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} IMU_Data;

typedef struct {
  uint8_t tx[TX_DATA_SIZE];
  float linear_x;
  float linear_y;
  float angular_z;
  unsigned char frame_tail;
} TX_Data;

typedef struct {
  uint8_t rx[RX_DATA_SIZE];
  uint8_t flag_stop;
  unsigned char frame_header;
  float linear_x;
  float linear_y;
  float angular_z;
  float voltage;
  unsigned char frame_tail;
} RX_Data;

class MaxRobotBase
{
public:
  MaxRobotBase(rclcpp::Logger &logger);           
  ~MaxRobotBase();
  bool initialize_serial(const std::string& port, const int baud_rate);
  bool is_serial_opened();
  void stop();
  void write(float linear_x, float steer_angle);
  bool read();

  float voltage; 
  Vel_Data vel_data;   
  IMU_Data imu_data; 
  
private:
  enum class ParseState { HEADER, PAYLOAD, CHECKSUM, TAIL }; // State machine states
  struct Packet {
    ParseState state = ParseState::HEADER;
    std::vector<uint8_t> data;
    size_t index = 0;
  };

  std::unique_ptr<boost::asio::serial_port> base_serial_;
  boost::asio::io_service io_service_;
  RX_Data rx_data_; 
  TX_Data tx_data_;
  rclcpp::Logger logger_; 
  std::vector<uint8_t> buffer_; // Serial read buffer
  std::mutex buffer_mutex_;     // Thread safety
  Packet packet_;               // Current packet being parsed
  bool data_ready_ = false;     // Flag for valid packet
  std::thread io_thread_;       // Thread for running io_service

  unsigned char checksum(unsigned char count, unsigned char mode);
  int16_t imu_trans(uint8_t high, uint8_t low); 
  float odom_trans(uint8_t high, uint8_t low); 
  void start_async_read();
  bool validate_checksum();
  void process_packet();
};

#endif // __MAX_ROBOT_BASE_H_
