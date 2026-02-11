#include <chrono>
//#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "max_robot_base/max_robot_system.hpp"

namespace max_robot_base
{
hardware_interface::CallbackReturn MaxRobotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("hardware_interface.max_robot"));

  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
  
  robot_base_ = std::make_unique<MaxRobotBase>(*logger_);

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "MaxRobotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  try {
    serial_port_ = info.hardware_parameters.at("serial_port");
    baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
    wheel_radius_ = std::stof(info.hardware_parameters.at("wheel_radius"));
    wheel_base_ = std::stof(info.hardware_parameters.at("wheel_base"));
    wheel_track_ = std::stof(info.hardware_parameters.at("wheel_track"));
  } catch (const std::out_of_range& e) {
    RCLCPP_ERROR(get_logger(), "Missing 'serial_port' or 'baud_rate' parameter in hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument& e) {
    RCLCPP_ERROR(get_logger(), "Invalid 'baud_rate' parameter: must be an integer");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");
  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

  hw_imu_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MaxRobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.state.velocity));
    }
  }

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_imu_sensor_states_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name, &hw_battery_sensor_state_));

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_DEBUG(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MaxRobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_POSITION,
          &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_DEBUG(
      get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MaxRobotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }

  if (!robot_base_->initialize_serial(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize serial device");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MaxRobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!robot_base_->is_serial_opened()) {
    RCLCPP_ERROR(get_logger(), "Serial port not open");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
     robot_base_->stop();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MaxRobotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!robot_base_->is_serial_opened()) {
    RCLCPP_ERROR(get_logger(), "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }

  if (robot_base_->read()) {
    float linear_x = robot_base_->vel_data.linear_x;
    // the feedback angular_z is based on differential drive, using commanded steer angle
    float commanded_steering_angle = hw_interfaces_["steering"].command.position;

    hw_interfaces_["steering"].state.position = commanded_steering_angle;
    hw_interfaces_["traction"].state.velocity = linear_x / wheel_radius_;
    hw_interfaces_["traction"].state.position += (linear_x / wheel_radius_) * period.seconds();

    hw_imu_sensor_states_[0] = robot_base_->imu_data.accel_x;
    hw_imu_sensor_states_[1] = robot_base_->imu_data.accel_y;
    hw_imu_sensor_states_[2] = robot_base_->imu_data.accel_z;
    hw_imu_sensor_states_[3] = robot_base_->imu_data.gyro_x;
    hw_imu_sensor_states_[4] = robot_base_->imu_data.gyro_y;
    hw_imu_sensor_states_[5] = robot_base_->imu_data.gyro_z;

    hw_battery_sensor_state_ = robot_base_->voltage;
    
    RCLCPP_DEBUG(get_logger(), "linear_x = %.4f, steer_angle = %.2f deg, battery = %.2f V",
                            linear_x, commanded_steering_angle * 57.2958f,  hw_battery_sensor_state_);
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type max_robot_base ::MaxRobotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!robot_base_->is_serial_opened()) {
    RCLCPP_ERROR(get_logger(), "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }

  float vx = hw_interfaces_["traction"].command.velocity * wheel_radius_;
  float steer_angle = hw_interfaces_["steering"].command.position;
  float vz = std::tan(steer_angle) * vx / wheel_base_;

  RCLCPP_DEBUG(get_logger(), "command: linear_x = %f, angular_z = %f, steer_angle = %f", 
		  vx, vz, steer_angle);
  
  const float min_turn_radius_ = 0.750f;
  float angle_r = 0.0;

  if (std::abs(vz) > 1e-6 && std::abs(vx) > 1e-6) {
      float R = vx / vz;
      RCLCPP_INFO(get_logger(), "R = %0.4f", R);
      // Enforce minimum turning radius
      if (std::abs(R) < min_turn_radius_) {
          vz = std::copysign(std::abs(vx) / min_turn_radius_, vz);
          R = vx / vz;
          RCLCPP_INFO(get_logger(), "updated R = %0.4f", R);
      }
      // Compute right wheel steering angle
      float denominator = R + 0.5 * wheel_track_;
      angle_r = (std::abs(denominator) > 1e-6) ? std::atan2(wheel_base_, denominator) : 0.0;
      if (angle_r > M_PI / 2.0) {
          angle_r -= M_PI;
      } else if (angle_r < -M_PI / 2.0) {
          angle_r += M_PI;
      }

      // Apply STM32's steering angle limits
      angle_r = std::clamp(angle_r, -0.50f, 0.34f);
      // Adjust vz to match limited angle
      float R_adjusted = wheel_base_ / std::tan(angle_r) - 0.5 * wheel_track_;
      vz = vx / (R_adjusted + 0.5 * wheel_track_); // Approximate center radius
  } else {
      vz = 0.0;
      angle_r = 0.0;
  }

  // update the steer angle with updated vz
  float delta = 0.0;
  if (std::abs(angle_r) > 1e-6) {
      delta = std::atan(vz * wheel_base_ / vx);
  }
  hw_interfaces_["steering"].command.position = delta;

  robot_base_->write(vx, vz);
  RCLCPP_DEBUG(get_logger(), "write: linear_x = %0.4f, angular_z = %0.4f, steer_angle = %0.4f", 
		  vx, vz, hw_interfaces_["steering"].command.position);

  return hardware_interface::return_type::OK;
}

}  // namespace max_robot_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  max_robot_base::MaxRobotSystemHardware, hardware_interface::SystemInterface)
