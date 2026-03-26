/**
 *****************************************************************************************
 *  Filename:       falcon_arm_hardware_interface.hpp
 *  Description:    ros2_control hardware interface plugin for the Falcon Robotic Arm.
 *                  Implements hardware_interface::SystemInterface to manage serial
 *                  communication between ROS2 controllers and the STM32 Nucleo
 *                  microcontroller over UART at 115200 baud.
 *
 *                  Interface summary:
 *                    State  interfaces : position + velocity for 5 arm joints,
 *                                        position for 1 gripper joint
 *                    Command interfaces: position for all 6 joints
 *                    Unit convention   : ROS2 side uses radians;
 *                                        firmware side uses degrees (auto-converted)
 *
 *  Author:         BARGAVAN R - MIT
 *  Co-Author:      RON NIRVIN J
 *
 *  Dependencies:
 *    hardware_interface/system_interface.hpp
 *    serial/serial.h    (serial library for UART communication)
 *    rclcpp/rclcpp.hpp
 *
 *  Notes:
 *    Plugin registration:  falcon_arm_hardware/FalconArmHardwareInterface
 *    URDF parameter keys:  serial_port, baud_rate, timeout_ms
 *    Default serial port:  /dev/ttyACM0  (STM32 Nucleo USB-CDC enumeration on Linux)
 *    Serial protocol:      See SERIAL_PROTOCOL.md in this package.
 *****************************************************************************************
 */

#ifndef FALCON_ARM_HARDWARE_INTERFACE_HPP
#define FALCON_ARM_HARDWARE_INTERFACE_HPP

// Include the base class for hardware interfaces
#include <hardware_interface/system_interface.hpp>

// Include standard C++ library for vectors and strings
#include <vector>
#include <string>

// Include ROS2 log ger for logging capability
#include <rclcpp/rclcpp.hpp>

// Include serial port library for STM32 communication
#include <serial/serial.h>

// Include the return code type for lifecycle transitions
#include <hardware_interface/types/hardware_interface_return_values.hpp>

// Define the namespace for this package
namespace falcon_arm_hardware
{
  // Define constant for the number of arm joints (5 joints)
  constexpr size_t NUM_ARM_JOINTS = 5;

  // Define constant for the number of gripper joints (1 joint)
  constexpr size_t NUM_GRIPPER_JOINTS = 1;

  // Define total number of joints (5 arm + 1 gripper = 6 total)
  constexpr size_t NUM_TOTAL_JOINTS = NUM_ARM_JOINTS + NUM_GRIPPER_JOINTS;

  // Define conversion factor from radians to degrees (180/π)
  constexpr double RADIANS_TO_DEGREES = 180.0 / 3.14159265358979323846;

  // Define conversion factor from degrees to radians (π/180)
  constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;

  /**
   * @class FalconArmHardwareInterface
   * @brief Hardware interface plugin for Falcon Robot Arm STM32 control
   * 
   * This class implements the ros2_control hardware interface for communicating
   * with the STM32 microcontroller via serial port. It converts joint position
   * commands from radians (used by MoveIt2) to degrees (used by the firmware)
   * and reads joint positions from the hardware.
   * 
   * The interface manages:
   * - 5 arm joints with position and velocity feedback
   * - 1 gripper joint with position control
   * - Serial communication at 115200 baud rate
   * - Conversion between radians and degrees
   */
  class FalconArmHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    /**
     * @brief Constructor for FalconArmHardwareInterface
     * 
     * Initializes the hardware interface with default values. This does not
     * actually open the serial port or allocate hardware resources; that happens
     * in the on_init() method during the initialization lifecycle.
     */
    FalconArmHardwareInterface();

    /**
     * @brief Destructor for FalconArmHardwareInterface
     * 
     * Cleans up any resources allocated by the constructor. The serial port
     * is closed in on_cleanup() during the shutdown lifecycle.
     */
    ~FalconArmHardwareInterface() override;

    /**
     * @brief Initialize the hardware interface
     * 
     * Called once at startup to initialize hardware parameters such as:
     * - Serial port name (default: /dev/ttyACM0)
     * - Baud rate (default: 115200)
     * - Timeout settings
     * - Joint names and limits
     *
     * @param info - Structure containing hardware configuration from robot description
     * @return CallbackReturn::SUCCESS if initialization was successful
     * @return CallbackReturn::ERROR if initialization failed
     */
    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

    /**
     * @brief Export state and command interfaces
     * 
     * Defines which joints and interfaces are available. This method creates:
     * - Position state interfaces for all 6 joints
     * - Velocity state interfaces for arm joints only
     * - Position command interfaces for all 6 joints
     *
     * @return Vector of StateInterface objects defining available interfaces
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * @brief Export command interfaces
     * 
     * Defines the command interfaces available to controllers.
     *
     * @return Vector of CommandInterface objects defining available interfaces
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
     * @brief Configure the hardware interface
     * 
     * Called before activation to prepare the hardware. This includes:
     * - Opening the serial port
     * - Setting serial parameters (baud rate, timeouts)
     * - Initializing variables
     *
     * @return CallbackReturn::SUCCESS if configuration was successful
     * @return CallbackReturn::ERROR if configuration failed
     */
    hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Activate the hardware interface
     * 
     * Called after configuration to start the hardware. This enables:
     * - Starting the communication with the microcontroller
     * - Initial state reading
     *
     * @return CallbackReturn::SUCCESS if activation was successful
     * @return CallbackReturn::ERROR if activation failed
     */
    hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Deactivate the hardware interface
     * 
     * Called before shutdown to stop active command transmission. This is
     * important to safely disable motors before closing the connection.
     *
     * @return CallbackReturn::SUCCESS if deactivation was successful
     * @return CallbackReturn::ERROR if deactivation failed
     */
    hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Clean up the hardware interface
     * 
     * Called at shutdown to release all hardware resources:
     * - Close serial port connection
     * - Free allocated memory
     *
     * @return CallbackReturn::SUCCESS if cleanup was successful
     * @return CallbackReturn::ERROR if cleanup failed
     */
    hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Read the current state from the hardware
     * 
     * This method is called periodically by the controller manager to:
     * - Read position and velocity from all joints via serial communication
     * - Update the state_interfaces with the received values
     * - Handle serial communication errors gracefully
     *
     * The serial protocol expects a message with joint positions in degrees
     * that are then converted back to radians for ros2_control.
     *
     * @return return_type::OK if read was successful
     * @return return_type::ERROR if read failed
     */
    hardware_interface::return_type read(
      const rclcpp::Time &time,
      const rclcpp::Duration &period) override;

    /**
     * @brief Write commands to the hardware
     * 
     * This method is called periodically by the controller manager to:
     * - Read position commands from command_interfaces in radians
     * - Convert position commands from radians to degrees
     * - Send the converted degree values to the STM32 via serial port
     * - Handle serial errors gracefully
     *
     * The conversion is necessary because:
     * - ROS2/MoveIt2 uses radians (standard SI unit)
     * - The STM32 firmware works with degrees for easier servo motor control
     *
     * @return return_type::OK if write was successful
     * @return return_type::ERROR if write failed
     */
    hardware_interface::return_type write(
      const rclcpp::Time &time,
      const rclcpp::Duration &period) override;

  private:
    /**
     * @brief Open connection to the STM32 via serial port
     * 
     * Establishes the serial communication link with the microcontroller.
     * This is called during configure phase.
     *
     * @return true if port was opened successfully, false otherwise
     */
    bool openSerialPort();

    /**
     * @brief Close connection to the STM32
     * 
     * Terminates the serial communication link safely.
     * This is called during cleanup phase.
     */
    void closeSerialPort();

    /**
     * @brief Send position commands to the microcontroller
     * 
     * Constructs and transmits a serial message containing joint position commands
     * in degrees. The message format is typically:
     * "J1:deg1,J2:deg2,J3:deg3,J4:deg4,J5:deg5,J6:deg6\n"
     *
     * @param command_degrees - Vector of 6 joint positions in degrees
     * @return true if message was sent successfully, false otherwise
     */
    bool sendCommands(const std::vector<double> &command_degrees);

    /**
     * @brief Read joint positions from the microcontroller
     * 
     * Receives a serial message containing current joint positions in degrees
     * and parses it into the feedback_positions vector.
     *
     * @param feedback_degrees - Output vector to store parsed joint positions (degrees)
     * @return true if message was received and parsed successfully, false otherwise
     */
    bool readFeedback(std::vector<double> &feedback_degrees);

    /**
     * @brief Parse a serial message containing joint positions
     * 
     * Extracts numerical position values from a serial message string.
     * Expected format: "J1:deg1,J2:deg2,J3:deg3,J4:deg4,J5:deg5,J6:deg6"
     *
     * @param message - The serial message string to parse
     * @param positions - Output vector to store parsed positions
     * @return true if parsing was successful, false if format was invalid
     */
    bool parseMessage(const std::string &message, std::vector<double> &positions);

    // ========== Member Variables ==========

    /// Serial port object for communicating with the STM32 microcontroller
    serial::Serial serial_port_;

    /// Name of the serial port device (e.g., "/dev/ttyACM0" on Linux)
    std::string serial_port_name_;

    /// Baud rate for serial communication (bits per second)
    uint32_t baud_rate_;

    /// Timeout for reading from serial port (milliseconds)
    serial::Timeout timeout_;

    /// Vector storing the current position of each joint in radians
    /// Index 0-4: arm joints, Index 5: gripper joint
    std::vector<double> joint_positions_;

    /// Vector storing the current velocity of each joint in radians/second
    /// Note: Only arm joints provide velocity feedback
    std::vector<double> joint_velocities_;

    /// Vector storing the commanded position for each joint in radians
    /// These values are received from MoveIt2 via the controller
    std::vector<double> joint_commands_;

    /// Vector storing joint names (used for logging and debugging)
    std::vector<std::string> joint_names_;

    /// Flag indicating whether the serial port is currently open
    bool is_port_open_;

    /// Flag indicating whether the hardware interface is in active state
    /// Used to prevent commands from being sent when hardware is inactive
    bool is_active_;

    /// Counter for communication errors (can be used for diagnostics)
    uint32_t communication_error_count_;

    /// Maximum allowed communication errors before reporting failure
    uint32_t max_allowed_errors_;
  };

} // namespace falcon_arm_hardware

#endif // FALCON_ARM_HARDWARE_INTERFACE_HPP
