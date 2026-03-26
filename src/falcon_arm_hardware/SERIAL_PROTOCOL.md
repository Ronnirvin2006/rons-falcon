# Falcon Robot Arm - Serial Protocol Specification

## Overview

This document specifies the serial communication protocol between the hardware interface (ROS2 side) and the STM32 microcontroller (firmware side).

**Connection Parameters:**
- Baud Rate: **115200** bits per second
- Data Bits: **8**
- Stop Bits: **1**
- Parity: **None**
- Hardware Flow Control: **None**

---

## Message Direction 1: Hardware Interface → STM32 (Commands)

### Purpose
Hardware interface sends joint position commands to the STM32 microcontroller every 10 milliseconds (100 Hz update rate).

### Message Format

```
J1:degree1,J2:degree2,J3:degree3,J4:degree4,J5:degree5,J6:degree6\n
```

### Components

| Component | Description | Example |
|-----------|-------------|---------|
| `J1:` | Joint identifier prefix | Joints numbered 1-6 |
| `degree1` | Position in degrees | `45.5`, `-30.0`, `0.0` |
| `,` | Separator between joints | Single comma |
| `\n` | Line terminator | ASCII 0x0A (newline) |

### Example Messages

```
# All joints at zero degrees (ready position)
J1:0.0,J2:0.0,J3:0.0,J4:0.0,J5:0.0,J6:0.0\n

# Arm in a working position, gripper closed
J1:45.5,J2:30.0,J3:-15.2,J4:60.0,J5:45.0,J6:0.0\n

# Negative angles (reverse direction)
J1:-90.0,J2:-45.5,J3:15.0,J4:0.0,J5:30.0,J6:-10.0\n

# Decimal precision
J1:12.345,J2:-67.890,J3:0.025,J4:123.456,J5:89.012,J6:34.567\n
```

### Reception Requirements (STM32 Firmware)

1. **Receive Data**
   - Read serial data byte by byte
   - Buffer incoming characters until newline (`\n`) is received

2. **Parse Message**
   - Split message by comma (`,`)
   - Extract 6 joint values
   - Skip the `J#:` prefix, convert numeric part to float

3. **Validate Data**
   - Ensure exactly 6 joint values received
   - Validate degree values are within motor limits
   - Apply motor-specific joint limits

4. **Execute Commands**
   - Convert degrees to PWM duty cycle for servo motors
   - Send PWM signals to motor drivers
   - Log/store commands for feedback response

5. **Joint Degree Ranges** (example values, adjust for your motors)
   - Joints 1-5: -180° to +180° (typical servo range)
   - Joint 6 (gripper): 0° to 90° (open/close range)

### Parsing Pseudocode

```cpp
// Receives: "J1:45.5,J2:30.0,...,J6:0.0\n"
void parse_command(char* message) {
    float joint_degrees[6];
    char* token = strtok(message, ",");  // Split by comma
    
    for (int i = 0; i < 6; i++) {
        // Each token is like "J1:45.5"
        char* colon = strchr(token, ':');  // Find ':'
        float degree = atof(colon + 1);    // Parse number after ':'
        joint_degrees[i] = degree;
        
        // Validate and clamp to motor limits
        if (joint_degrees[i] < MIN_DEGREES[i])
            joint_degrees[i] = MIN_DEGREES[i];
        if (joint_degrees[i] > MAX_DEGREES[i])
            joint_degrees[i] = MAX_DEGREES[i];
        
        // Convert degrees to motor command
        uint16_t pwm_value = degrees_to_pwm(i, joint_degrees[i]);
        send_to_motor(i, pwm_value);
        
        token = strtok(NULL, ",");  // Next joint
    }
}
```

---

## Message Direction 2: STM32 → Hardware Interface (Feedback)

### Purpose
STM32 sends current joint positions back to the hardware interface for state feedback and controller monitoring.

### Message Format

```
J1:degree1,J2:degree2,J3:degree3,J4:degree4,J5:degree5,J6:degree6\n
```

### Same Format as Command Messages

The response format is **identical** to the command format for simplicity:
- Comma-separated values
- Degree units
- Newline terminated

### Example Feedback Messages

```
# Actual feedback from motors
J1:45.5,J2:29.8,J3:-15.3,J4:60.1,J5:44.9,J6:0.0\n

# Small deviations from commanded values
J1:45.5,J2:30.02,J3:-15.18,J4:60.05,J5:44.95,J6:0.02\n
```

### Transmission Requirements (STM32 Firmware)

1. **Timing**
   - Send feedback at **100 Hz minimum** (every 10ms or faster)
   - Or send feedback whenever position changes significantly (>0.5°)
   - Or use commanded frequency from host

2. **Data Source**
   - Read feedback from motor encoders/potentiometers
   - Get current actual motor position, not commanded position
   - Apply any necessary filtering to smooth values

3. **Format Requirements**
   - Send exactly 6 joint values in same order as input
   - Use same degree precision (1 decimal place minimum)
   - Always end with newline character

4. **Error Handling**
   - If motor encoder fails, send last known position + error flag
   - Or send special error value (e.g., 999.0)
   - Log error to firmware logs

### Transmission Pseudocode

```cpp
// Send feedback at 100 Hz
void send_feedback(void) {
    static uint32_t last_send_time = 0;
    uint32_t current_time = get_systick_ms();
    
    // Send every 10ms (100 Hz)
    if ((current_time - last_send_time) >= 10) {
        last_send_time = current_time;
        
        // Read current position from each motor
        float feedback_degrees[6];
        for (int i = 0; i < 6; i++) {
            feedback_degrees[i] = read_motor_position_degrees(i);
        }
        
        // Format and send message
        sprintf(uart_buffer, "J1:%.1f,J2:%.1f,J3:%.1f,J4:%.1f,J5:%.1f,J6:%.1f\n",
                feedback_degrees[0], feedback_degrees[1], 
                feedback_degrees[2], feedback_degrees[3],
                feedback_degrees[4], feedback_degrees[5]);
        
        uart_send_string(uart_buffer);
    }
}
```

---

## Motor Servo Mapping

### Joint to Motor Mapping

| Joint | Type | Motor | PWM Range | Degree Range |
|-------|------|-------|-----------|-----------------|
| 1 | Revolute | MG996R | 500µs - 2500µs | -180° to +180° |
| 2 | Revolute | MG996R | 500µs - 2500µs | -180° to +180° |
| 3 | Revolute | MG996R | 500µs - 2500µs | -180° to +180° |
| 4 | Revolute | MG996R | 500µs - 2500µs | -180° to +180° |
| 5 | Revolute | Micro-9G | 500µs - 2500µs | -180° to +180° |
| 6 | Prismatic | Micro-9G | 500µs - 2500µs | 0° to 90° |

### PWM to Degree Conversion

For standard servo motors (MG996R, Micro-9G):

```cpp
// Servo neutral: 1500µs = 0 degrees
// Min: 500µs = -90 degrees (or -180°)
// Max: 2500µs = +90 degrees (or +180°)

// Linear conversion: PWM_value = 1500 + (degree * 1667 / 180)
// Assuming 5000µs period
uint16_t degrees_to_pwm(float degrees) {
    // Clamp to valid range
    if (degrees < -180) degrees = -180;
    if (degrees > 180) degrees = 180;
    
    // Convert: center at 1500µs, 1667µs per 180°
    // For 50Hz update: period = 20000µs, 1-2ms range
    uint16_t pwm_us = 1500 + (degrees * 1000 / 180.0);
    
    // Convert µs to timer count (system dependent)
    // Example: timer clock = 1MHz, count = µs value
    return pwm_us;
}

// Reverse conversion
float pwm_to_degrees(uint16_t pwm_us) {
    float degrees = (pwm_us - 1500) * 180.0 / 1000.0;
    return degrees;
}
```

---

## Handling Serial Errors

### Hardware Interface Error Cases

| Error | Hardware Response | Firmware Response |
|-------|------------------|-------------------|
| Timeout (no data) | Return ERROR, retry | Send heartbeat/status periodically |
| Malformed message | WARN log, skip frame | Send error indicator |
| Joint value invalid | Clamp and continue | Validate and reject |
| Serial port closed | Try to reconnect | Attempt re-connection |
| Communication errors | Increment counter | Re-send on request |

### Watchdog/Timeout Behavior

```cpp
// Hardware Interface Timeout (C++)
if (serial_available() == 0) {
    // No data available, timeout after 1 second
    if (timestamp_ms() - last_read_time > 1000) {
        RCLCPP_ERROR("Serial timeout - no feedback received");
        return ERROR;
    }
}

// STM32 Timeout (pseudocode)
if (missing_command_for_ms > 500) {
    // No command received for 500ms - emergency stop?
    disable_all_motors();
    last_safe_positions = current_positions;
    wait_for_new_command();
}
```

---

## Debugging Serial Communication

### ROS2 Side: Monitor Hardware Interface

```bash
# Enable debug logging
export ROS_LOG_LEVEL=DEBUG

# Run the interface
ros2 launch falcon_arm_bringup robot_hardware.launch.py

# Watch the output for:
# - "Sending: J1:45.5,..."
# - "Received: J1:45.5,..."
# - Serial port errors
```

### STM32 Side: Debug Output

Add UART output for debugging (separate from main protocol UART):

```cpp
// Separate UART for debug output
printf("Received command: %s\n", uart_buffer);
printf("Parsed: J1=%.1f J2=%.1f ... J6=%.1f\n", 
       j1_deg, j2_deg, ..., j6_deg);
printf("PWM values: %d %d %d %d %d %d\n",
       pwm1, pwm2, pwm3, pwm4, pwm5, pwm6);
```

### Monitor Serial Port (Linux)

```bash
# Install tools
sudo apt install minicom socat

# Monitor serial port with minicom
minicom -D /dev/ttyACM0 -b 115200

# Use socat to create a virtual port pair for testing
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# Send test messages
echo "J1:45.0,J2:30.0,J3:-15.0,J4:60.0,J5:45.0,J6:0.0" > /dev/ttyUSB0

# Read responses
cat < /dev/ttyUSB1
```

---

## Performance Specifications

### Latency
- **Command to Motor**: < 20ms (should execute within 2 cycles)
- **Position Feedback to Controller**: < 30ms (typically 10-15ms)
- **Round Trip**: < 50ms (command sent → feedback received)

### Jitter Tolerance
- System should tolerate ±5ms jitter in update rate
- Feedback rate variance should not exceed ±20%

### Message Rate
- **Nominal**: 100 Hz (10 ms interval)
- **Minimum**: 10 Hz (safe teleoperation)
- **Maximum**: 200 Hz (requires hardware support)

### Error Rate
- Target: < 0.1% corrupted frames
- Acceptable: < 5% with error recovery
- Critical: > 10% consecutive errors → stop hardware

---

## Notes for STM32 Implementation

1. **Use DMA for UART** - Prevent blocking during transmission/reception
2. **Use interrupts** - Don't poll serial port constantly
3. **Implement ring buffer** - Handle variable message arrival
4. **Add CRC/checksum** - Optional but recommended for reliability
5. **Log errors** - Store last N errors for debugging
6. **Rate limit feedback** - Avoid saturating USB serial link
7. **Graceful degradation** - Continue if one motor fails
8. **Emergency stop** - Hardware e-stop button with timeout

---

## References

- MG996R Datasheet: Standard servo, typically 500-2500µs PWM
- Arduino STM32 UART Documentation: Serial library reference
- ROS2 Serial Communication: Best practices guide
