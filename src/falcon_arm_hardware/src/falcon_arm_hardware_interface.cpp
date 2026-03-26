/**
 *****************************************************************************************
 *  Filename:       falcon_arm_hardware_interface.cpp
 *  Description:    Implementation of the Falcon Arm ros2_control hardware interface.
 *                  Handles the full lifecycle (init → configure → activate → read/write
 *                  → deactivate → cleanup) for serial communication with the STM32
 *                  Nucleo microcontroller.
 *
 *                  Key responsibilities:
 *                    • open/close UART serial port at 115 200 baud
 *                    • read() — receive joint feedback from firmware, convert deg → rad
 *                    • write() — send position commands to firmware, convert rad → deg
 *                    • graceful error handling (counts errors, logs warnings, continues)
 *
 *  Author:         BARGAVAN R - MIT
 *  Co-Author:      RON NIRVIN J
 *
 *  Serial message format (see SERIAL_PROTOCOL.md for full spec):
 *    TX to STM32 :  "J1:deg1,J2:deg2,J3:deg3,J4:deg4,J5:deg5,J6:deg6\n"
 *    RX from STM32:  same format with current encoder-read positions
 *
 *  Unit convention:
 *    ROS2 / MoveIt2 side  →  radians  (SI standard)
 *    STM32 firmware side  →  degrees  (servo library convention)
 *    Conversion applied in read() and write() using RADIANS_TO_DEGREES constant.
 *****************************************************************************************
 */

// Include the header file for this class
#include "falcon_arm_hardware/falcon_arm_hardware_interface.hpp"

// Include standard library for string operations
#include <sstream>

// Include for std::find function
#include <algorithm>

// Use the namespace defined in the package
namespace falcon_arm_hardware
{
  /**
   * CONSTRUCTOR: Initialize the FalconArmHardwareInterface
   * 
   * This constructor is called when the hardware interface object is first created.
   * It initializes all member variables to safe default values.
   */
  FalconArmHardwareInterface::FalconArmHardwareInterface()
    : // Initialize serial port name to the default USB port for STM32
      serial_port_name_("/dev/ttyACM0"),
      // Initialize baud rate to 115200 bits per second (standard for STM32)
      baud_rate_(115200),
      // Initialize timeout structure for serial communication
      timeout_(serial::Timeout::simpleTimeout(1000)), // 1 second timeout
      // Initialize serial port open flag to false (port not yet opened)
      is_port_open_(false),
      // Initialize active flag to false (hardware not yet activated)
      is_active_(false),
      // Initialize communication error count to 0
      communication_error_count_(0),
      // Initialize maximum allowed communication errors to 10
      max_allowed_errors_(10)
  {
    // Resize joint positions vector to hold 6 values (5 arm + 1 gripper)
    // Initialize all values to 0.0 radians
    joint_positions_.resize(NUM_TOTAL_JOINTS, 0.0);

    // Resize joint velocities vector to hold 6 values
    // Initialize all values to 0.0 radians/second
    joint_velocities_.resize(NUM_TOTAL_JOINTS, 0.0);

    // Resize joint commands vector to hold 6 values
    // Initialize all values to 0.0 radians
    joint_commands_.resize(NUM_TOTAL_JOINTS, 0.0);

    // Resize joint names vector to hold 6 names
    joint_names_.resize(NUM_TOTAL_JOINTS);
  }

  /**
   * DESTRUCTOR: Clean up the FalconArmHardwareInterface
   * 
   * This destructor is called when the hardware interface object is destroyed.
   * It ensures the serial port is closed before the object is deleted.
   */
  FalconArmHardwareInterface::~FalconArmHardwareInterface()
  {
    // Close the serial port if it's open to prevent resource leaks
    if (is_port_open_)
    {
      closeSerialPort();
    }
  }

  /**
   * LIFECYCLE METHOD: on_init
   * 
   * This method is called during system initialization to read configuration
   * parameters from the robot description and set up the hardware interface.
   * It's part of the hardware lifecycle: on_init -> on_configure -> on_activate
   */
  hardware_interface::CallbackReturn FalconArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info)
  {
    // Call the parent class initialization method
    // This validates the HardwareInfo structure against the URDF
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      // If parent initialization fails, log error and return failure
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Failed to initialize parent SystemInterface");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Extract the number of joints from the hardware info
    // This should be 6 (5 arm joints + 1 gripper joint)
    const size_t num_joints = info.joints.size();
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Initializing with %zu joints", num_joints);

    // Validate that we have exactly 6 joints
    if (num_joints != NUM_TOTAL_JOINTS)
    {
      // Log error if joint count doesn't match expected value
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Expected %zu joints but got %zu", NUM_TOTAL_JOINTS, num_joints);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Iterate through all joints to extract their names and validate them
    for (size_t i = 0; i < num_joints; ++i)
    {
      // Extract the joint name from the hardware info
      joint_names_[i] = info.joints[i].name;

      // Log the joint name for debugging purposes
      RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                  "Joint %zu: %s", i, joint_names_[i].c_str());
    }

    // Try to extract serial port configuration from hardware parameters
    // Loop through all parameters in the hardware definition
    for (const auto &param : info.hardware_parameters)
    {
      // Check if the parameter is the serial port name
      if (param.first == "serial_port")
      {
        // Set the serial port name from configuration (e.g., "/dev/ttyACM0")
        serial_port_name_ = param.second;
        // Log the configured serial port
        RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                    "Serial port set to: %s", serial_port_name_.c_str());
      }
      // Check if the parameter is the baud rate
      else if (param.first == "baud_rate")
      {
        // Convert the baud rate string to an unsigned integer (e.g., "115200" -> 115200)
        baud_rate_ = std::stoul(param.second);
        // Log the configured baud rate
        RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                    "Baud rate set to: %u", baud_rate_);
      }
    }

    // Log successful initialization
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Hardware initialization successful");

    // Return success to indicate initialization completed without errors
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * INTERFACE METHOD: export_state_interfaces
   * 
   * This method defines which state interfaces (sensors/feedback) are available.
   * State interfaces are read-only and provide feedback from the hardware.
   * For our arm: position and velocity feedback from all joints.
   */
  std::vector<hardware_interface::StateInterface> FalconArmHardwareInterface::export_state_interfaces()
  {
    // Create a vector to store all state interfaces
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Loop through all 6 joints (5 arm + 1 gripper)
    for (size_t i = 0; i < NUM_TOTAL_JOINTS; ++i)
    {
      // Add a position state interface for this joint
      // This allows controllers to read the current joint position in radians
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          // Joint name (e.g., "joint_1")
          joint_names_[i],
          // Interface name: "position" (standard ROS2 interface name)
          "position",
          // Reference to the internal storage where position is stored
          &joint_positions_[i]));

      // Add a velocity state interface for this joint
      // This allows controllers to read the current joint velocity in radians/second
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          // Joint name (e.g., "joint_1")
          joint_names_[i],
          // Interface name: "velocity" (standard ROS2 interface name)
          "velocity",
          // Reference to the internal storage where velocity is stored
          &joint_velocities_[i]));
    }

    // Return the vector containing all state interfaces
    // Controllers will use these to read joint feedback
    return state_interfaces;
  }

  /**
   * INTERFACE METHOD: export_command_interfaces
   * 
   * This method defines which command interfaces (actuators/motors) are available.
   * Command interfaces are write-only and send commands to the hardware.
   * For our arm: position commands for all 6 joints.
   */
  std::vector<hardware_interface::CommandInterface> FalconArmHardwareInterface::export_command_interfaces()
  {
    // Create a vector to store all command interfaces
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Loop through all 6 joints (5 arm + 1 gripper)
    for (size_t i = 0; i < NUM_TOTAL_JOINTS; ++i)
    {
      // Add a position command interface for this joint
      // This allows controllers to send position commands in radians
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          // Joint name (e.g., "joint_1")
          joint_names_[i],
          // Interface name: "position" (standard ROS2 interface name)
          "position",
          // Reference to the internal storage where commands are stored
          &joint_commands_[i]));
    }

    // Return the vector containing all command interfaces
    // Controllers will use these to send joint position commands
    return command_interfaces;
  }

  /**
   * LIFECYCLE METHOD: on_configure
   * 
   * This method is part of the hardware lifecycle and is called after on_init
   * but before on_activate. It prepares the hardware for operation by opening
   * the serial connection to the STM32 microcontroller.
   */
  hardware_interface::CallbackReturn FalconArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &previous_state)
  {
    // Log that configuration is starting
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Configuring Falcon Arm Hardware Interface...");

    // Attempt to open the serial port connection
    if (!openSerialPort())
    {
      // If opening fails, log error and return failure
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Failed to open serial port during configuration");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Log successful configuration
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Configuration successful");

    // Return success to proceed to activation
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * LIFECYCLE METHOD: on_activate
   * 
   * This method is called after on_configure to start active hardware operation.
   * It enables the sending of commands to the microcontroller.
   */
  hardware_interface::CallbackReturn FalconArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
  {
    // Log that activation is starting
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Activating Falcon Arm Hardware Interface...");

    // Set the active flag to true to enable command transmission
    is_active_ = true;

    // Reset communication error counter at activation
    communication_error_count_ = 0;

    // Log successful activation
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Hardware activated successfully");

    // Return success to indicate hardware is ready for control
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * LIFECYCLE METHOD: on_deactivate
   * 
   * This method is called when the hardware needs to be deactivated
   * (before on_cleanup). It safely stops the hardware from receiving new commands.
   */
  hardware_interface::CallbackReturn FalconArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &previous_state)
  {
    // Log that deactivation is starting
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Deactivating Falcon Arm Hardware Interface...");

    // Set the active flag to false to stop command transmission
    is_active_ = false;

    // Clear all joint commands to zero (safe state)
    for (auto &cmd : joint_commands_)
    {
      cmd = 0.0;
    }

    // Log successful deactivation
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Hardware deactivated successfully");

    // Return success
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * LIFECYCLE METHOD: on_cleanup
   * 
   * This method is called at system shutdown to release all hardware resources.
   * It's the last lifecycle method called, and it should close all connections.
   */
  hardware_interface::CallbackReturn FalconArmHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State &previous_state)
  {
    // Log that cleanup is starting
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Cleaning up Falcon Arm Hardware Interface...");

    // Close the serial port connection
    closeSerialPort();

    // Log successful cleanup
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Cleanup completed successfully");

    // Return success
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * OPERATION METHOD: read
   * 
   * This method is called repeatedly by the controller manager (typically at 100 Hz)
   * to read the current state of the hardware. It reads joint positions and velocities
   * from the STM32 microcontroller via serial communication.
   *
   * Flow:
   * 1. Read joint positions in degrees from serial port
   * 2. Convert positions from degrees to radians
   * 3. Update the state_interfaces (which controllers read from)
   * 4. Return OK or ERROR
   */
  hardware_interface::return_type FalconArmHardwareInterface::read(
    const rclcpp::Time &time,
    const rclcpp::Duration &period)
  {
    // Create a temporary vector to store joint positions in degrees (from hardware)
    std::vector<double> feedback_degrees(NUM_TOTAL_JOINTS, 0.0);

    // Try to read the feedback from the microcontroller
    if (!readFeedback(feedback_degrees))
    {
      // If read fails, increment the error counter
      communication_error_count_++;

      // Check if we've exceeded the maximum allowed errors
      if (communication_error_count_ > max_allowed_errors_)
      {
        // Log critical error if too many consecutive failures
        RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                     "Communication error threshold exceeded");
        return hardware_interface::return_type::ERROR;
      }

      // Return error but continue (transient communication failure)
      RCLCPP_WARN(rclcpp::get_logger("FalconArmHardwareInterface"),
                  "Failed to read feedback (error %u/%u)", 
                  communication_error_count_, max_allowed_errors_);
      return hardware_interface::return_type::ERROR;
    }

    // Reset error counter since this read was successful
    communication_error_count_ = 0;

    // Convert position values from degrees to radians
    // Loop through all 6 joints
    for (size_t i = 0; i < NUM_TOTAL_JOINTS; ++i)
    {
      // Convert this joint's position from degrees to radians
      // Formula: radians = degrees * (π / 180)
      joint_positions_[i] = feedback_degrees[i] * DEGREES_TO_RADIANS;

      // Calculate velocity as the rate of change of position
      // For now, we set velocity to 0 (assuming STM32 doesn't send velocity feedback)
      // In a real implementation, this could be (previous_position - current_position) / period
      joint_velocities_[i] = 0.0;
    }

    // Return OK to indicate successful read
    return hardware_interface::return_type::OK;
  }

  /**
   * OPERATION METHOD: write
   * 
   * This method is called repeatedly by the controller manager (typically at 100 Hz)
   * to send commands to the hardware. It reads position commands from the controller
   * (in radians), converts them to degrees, and sends them to the STM32 via serial.
   *
   * Flow:
   * 1. Check if hardware is active
   * 2. Read joint commands in radians from command_interfaces
   * 3. Convert commands from radians to degrees
   * 4. Send degree commands to serial port
   * 5. Return OK or ERROR
   */
  hardware_interface::return_type FalconArmHardwareInterface::write(
    const rclcpp::Time &time,
    const rclcpp::Duration &period)
  {
    // Check if the hardware interface is in the active state
    if (!is_active_)
    {
      // If inactive, don't send commands (safety feature)
      // Log this condition (but not too frequently to avoid log spam)
      RCLCPP_DEBUG(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Hardware not active, skipping write");
      return hardware_interface::return_type::OK;
    }

    // Create a temporary vector to store joint commands in degrees (for hardware)
    std::vector<double> command_degrees(NUM_TOTAL_JOINTS, 0.0);

    // Convert position commands from radians to degrees
    // Loop through all 6 joints
    for (size_t i = 0; i < NUM_TOTAL_JOINTS; ++i)
    {
      // Get the current command value in radians from the controller
      double command_rad = joint_commands_[i];

      // Convert from radians to degrees
      // Formula: degrees = radians * (180 / π)
      command_degrees[i] = command_rad * RADIANS_TO_DEGREES;

      // Log the command for debugging (verbose level, not shown by default)
      RCLCPP_DEBUG(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Joint %zu command: %.4f rad -> %.2f deg",
                   i, command_rad, command_degrees[i]);
    }

    // Try to send the commands to the microcontroller via serial port
    if (!sendCommands(command_degrees))
    {
      // If send fails, increment the error counter
      communication_error_count_++;

      // Check if we've exceeded the maximum allowed errors
      if (communication_error_count_ > max_allowed_errors_)
      {
        // Log critical error if too many consecutive failures
        RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                     "Command communication error threshold exceeded");
        return hardware_interface::return_type::ERROR;
      }

      // Return error but continue (transient communication failure)
      RCLCPP_WARN(rclcpp::get_logger("FalconArmHardwareInterface"),
                  "Failed to send commands (error %u/%u)",
                  communication_error_count_, max_allowed_errors_);
      return hardware_interface::return_type::ERROR;
    }

    // Reset error counter since this write was successful
    communication_error_count_ = 0;

    // Return OK to indicate successful write
    return hardware_interface::return_type::OK;
  }

  /**
   * PRIVATE METHOD: openSerialPort
   * 
   * Opens the serial port connection to the STM32 microcontroller.
   * Sets up communication parameters like baud rate and timeout.
   * 
   * @return true if port opened successfully, false otherwise
   */
  bool FalconArmHardwareInterface::openSerialPort()
  {
    // Log attempt to open serial port
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Opening serial port %s at %u baud",
                serial_port_name_.c_str(), baud_rate_);

    try
    {
      // Configure the serial port with the configured parameters
      serial_port_.setPort(serial_port_name_);
      serial_port_.setBaudrate(baud_rate_);
      serial_port_.setTimeout(timeout_);
      serial_port_.open();

      // Check if the port was successfully opened
      if (!serial_port_.isOpen())
      {
        // If port is not open after construction, log error
        RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                     "Serial port failed to open");
        return false;
      }

      // Set the flag indicating port is now open
      is_port_open_ = true;

      // Log successful port opening with port info
      RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                  "Serial port opened successfully");

      // Return true to indicate success
      return true;
    }
    catch (const serial::SerialException &e)
    {
      // Catch serial port exceptions and log the error message
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Serial port exception: %s", e.what());
      // Return false to indicate failure
      return false;
    }
  }

  /**
   * PRIVATE METHOD: closeSerialPort
   * 
   * Closes the serial port connection to the STM32 microcontroller.
   * Should be called during cleanup to release the serial device.
   */
  void FalconArmHardwareInterface::closeSerialPort()
  {
    // Log attempt to close serial port
    RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                "Closing serial port");

    try
    {
      // Check if the port is open before attempting to close
      if (serial_port_.isOpen())
      {
        // Close the serial port connection
        serial_port_.close();
      }

      // Set the flag indicating port is now closed
      is_port_open_ = false;

      // Log successful port closing
      RCLCPP_INFO(rclcpp::get_logger("FalconArmHardwareInterface"),
                  "Serial port closed successfully");
    }
    catch (const serial::SerialException &e)
    {
      // Catch serial port exceptions and log the error message
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Serial port exception during close: %s", e.what());
    }
  }

  /**
   * PRIVATE METHOD: sendCommands
   * 
   * Sends joint position commands to the STM32 microcontroller via serial port.
   * Constructs a message with all joint positions in degrees and transmits it.
   *
   * Example message format:
   * "J1:45.5,J2:30.0,J3:-15.2,J4:60.0,J5:45.0,J6:0.0\n"
   * 
   * @param command_degrees - Vector of 6 joint positions in degrees
   * @return true if message sent successfully, false otherwise
   */
  bool FalconArmHardwareInterface::sendCommands(const std::vector<double> &command_degrees)
  {
    // Check if serial port is open before trying to write
    if (!serial_port_.isOpen())
    {
      // Log error if port is not open
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Cannot send commands: serial port is not open");
      return false;
    }

    // Check if command vector has the correct size
    if (command_degrees.size() != NUM_TOTAL_JOINTS)
    {
      // Log error if vector size doesn't match expected joints
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Invalid command vector size: expected %zu, got %zu",
                   NUM_TOTAL_JOINTS, command_degrees.size());
      return false;
    }

    try
    {
      // Use stringstream to construct the message efficiently
      std::stringstream message_stream;

      // Build the message string with joint positions
      // Format: "J1:deg1,J2:deg2,...,J6:deg6\n"
      for (size_t i = 0; i < NUM_TOTAL_JOINTS; ++i)
      {
        // Add joint identifier and position to the message
        message_stream << "J" << (i + 1) << ":" << command_degrees[i];

        // Add comma separator between joints (but not after the last joint)
        if (i < NUM_TOTAL_JOINTS - 1)
        {
          message_stream << ",";
        }
      }

      // Add newline character to mark end of message
      message_stream << "\n";

      // Convert the stringstream to a string
      std::string message = message_stream.str();

      // Log the message being sent (for debugging)
      RCLCPP_DEBUG(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Sending: %s", message.c_str());

      // Write the message to the serial port
      // write() returns the number of bytes written
      size_t bytes_written = serial_port_.write(message);

      // Check if all bytes were successfully written
      if (bytes_written != message.length())
      {
        // Log warning if not all bytes were written (partial write)
        RCLCPP_WARN(rclcpp::get_logger("FalconArmHardwareInterface"),
                    "Partial write: sent %zu of %zu bytes",
                    bytes_written, message.length());
        return false;
      }

      // Return true to indicate successful send
      return true;
    }
    catch (const serial::SerialException &e)
    {
      // Catch serial port exceptions and log the error message
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Serial exception while sending commands: %s", e.what());
      // Return false to indicate failure
      return false;
    }
  }

  /**
   * PRIVATE METHOD: readFeedback
   * 
   * Reads joint position feedback from the STM32 microcontroller via serial port.
   * Waits for a complete message and parses it to extract joint positions in degrees.
   *
   * @param feedback_degrees - Output vector to store parsed joint positions (degrees)
   * @return true if message received and parsed successfully, false otherwise
   */
  bool FalconArmHardwareInterface::readFeedback(std::vector<double> &feedback_degrees)
  {
    // Check if serial port is open before trying to read
    if (!serial_port_.isOpen())
    {
      // Log error if port is not open
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Cannot read feedback: serial port is not open");
      return false;
    }

    try
    {
      // Check how many bytes are available to read from the serial buffer
      if (serial_port_.available() == 0)
      {
        // If no data available, return false without blocking
        RCLCPP_DEBUG(rclcpp::get_logger("FalconArmHardwareInterface"),
                     "No data available on serial port");
        return false;
      }

      // Read data from serial port until a newline character is found
      // This reads complete messages (assumed to end with '\n')
      std::string line = serial_port_.readline(
        // Maximum number of bytes to read (safety limit)
        256,
        // The delimiter character that marks end of message
        "\n");

      // Log the received message (for debugging)
      RCLCPP_DEBUG(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Received: %s", line.c_str());

      // Try to parse the received message into position values
      if (!parseMessage(line, feedback_degrees))
      {
        // If parsing fails, log error
        RCLCPP_WARN(rclcpp::get_logger("FalconArmHardwareInterface"),
                    "Failed to parse feedback message");
        return false;
      }

      // Return true to indicate successful read and parse
      return true;
    }
    catch (const serial::SerialException &e)
    {
      // Catch serial port exceptions and log the error message
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Serial exception while reading feedback: %s", e.what());
      // Return false to indicate failure
      return false;
    }
  }

  /**
   * PRIVATE METHOD: parseMessage
   * 
   * Parses a serial message string containing joint positions in degrees.
   * Expected format: "J1:deg1,J2:deg2,J3:deg3,J4:deg4,J5:deg5,J6:deg6"
   *
   * Example: "45.5,30.0,-15.2,60.0,45.0,0.0" or "J1:45.5,J2:30.0,..."
   *
   * @param message - The serial message string to parse
   * @param positions - Output vector to store parsed positions (must be sized correctly)
   * @return true if parsing was successful, false if format was invalid
   */
  bool FalconArmHardwareInterface::parseMessage(
    const std::string &message,
    std::vector<double> &positions)
  {
    // Check if the positions vector has the correct size
    if (positions.size() != NUM_TOTAL_JOINTS)
    {
      // Log error if vector is not the right size
      RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                   "Position vector size mismatch: expected %zu, got %zu",
                   NUM_TOTAL_JOINTS, positions.size());
      return false;
    }

    // Create a stringstream from the message for easy parsing
    std::stringstream ss(message);

    // String to temporarily store each parsed value
    std::string value_str;

    // Loop through all expected joint positions
    for (size_t i = 0; i < NUM_TOTAL_JOINTS; ++i)
    {
      // Try to extract the next value from the stringstream
      // getline with ',' reads until the next comma separator
      if (!std::getline(ss, value_str, ','))
      {
        // If extraction fails or no more data, log error
        RCLCPP_WARN(rclcpp::get_logger("FalconArmHardwareInterface"),
                    "Failed to parse joint %zu from message", i);
        return false;
      }

      try
      {
        // Find the colon separator if present (format: "J1:value")
        size_t colon_pos = value_str.find(':');

        // Extract the numeric portion of the string
        std::string numeric_part;
        if (colon_pos != std::string::npos)
        {
          // If colon found, take everything after it
          numeric_part = value_str.substr(colon_pos + 1);
        }
        else
        {
          // If no colon, use the entire string as numeric
          numeric_part = value_str;
        }

        // Convert the numeric string to a double value
        positions[i] = std::stod(numeric_part);

        // Log the parsed value (for debugging)
        RCLCPP_DEBUG(rclcpp::get_logger("FalconArmHardwareInterface"),
                     "Joint %zu parsed: %.2f degrees", i, positions[i]);
      }
      catch (const std::exception &e)
      {
        // Catch conversion exceptions (invalid number format)
        RCLCPP_ERROR(rclcpp::get_logger("FalconArmHardwareInterface"),
                     "Failed to convert value for joint %zu to double: %s",
                     i, e.what());
        return false;
      }
    }

    // Return true to indicate all positions were successfully parsed
    return true;
  }

} // namespace falcon_arm_hardware

// Register this class as a pluginlib plugin so ros2_control can find it
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  falcon_arm_hardware::FalconArmHardwareInterface,
  hardware_interface::SystemInterface)
