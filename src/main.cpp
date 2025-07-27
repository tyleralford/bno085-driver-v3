#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// micro-ROS includes
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

#define BNO085_ADDR 0x4A
#define BNO085_SDA 6 // Use I2C1 bus
#define BNO085_SCL 7 // Use I2C1 bus
#define BNO085_RESET -1

// LED pin definitions
#define LED_BLUE 25   // GPIO25
#define LED_GREEN 16  // GPIO16  
#define LED_RED 17    // GPIO17

// State machine definition
enum SystemState {
  INITIALIZING,
  AWAITING_SYNC,
  OPERATIONAL,
  CRITICAL_FAULT
};

SystemState current_state = INITIALIZING;
unsigned long last_led_toggle = 0;
bool led_state = false;

// Configuration parameters struct
struct SystemConfig {
  // Publishing parameters
  double publish_rate_hz;
  int sync_timeout_ms;
  int sync_retry_attempts;
  
  // Baseline covariance values (diagonal elements)
  double orientation_covariance_base;
  double angular_velocity_covariance_base;
  double linear_acceleration_covariance_base;
  
  // Covariance scaling factors for calibration levels (0-3)
  double covariance_scale_factors[4];
} config;

// Create BNO085 object
Adafruit_BNO08x bno08x(BNO085_RESET);
sh2_SensorValue_t sensorValue;

// Create a new I2C object on custom pins
arduino::MbedI2C wire2(BNO085_SDA, BNO085_SCL);

// micro-ROS objects
rcl_publisher_t publisher;
rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
sensor_msgs__msg__Imu imu_msg;

// Sensor data storage for high-frequency publishing
struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float quat_real, quat_i, quat_j, quat_k;
  bool accel_ready;
  bool gyro_ready;
  bool quat_ready;
  uint8_t accel_calibration_status;  // Accelerometer calibration accuracy (0-3)
  uint8_t gyro_calibration_status;   // Gyroscope calibration accuracy (0-3)
  uint8_t quat_calibration_status;   // Quaternion/orientation calibration accuracy (0-3)
} sensor_data = {0};

// Function to initialize default configuration parameters
void initializeDefaultConfig() {
  // Publishing parameters
  config.publish_rate_hz = 60.0;  // Default 60Hz as per REQ-03
  config.sync_timeout_ms = 5000;  // 5 seconds as per REQ-06  
  config.sync_retry_attempts = 3; // 3 attempts as per REQ-07
  
  // Baseline covariance values (conservative defaults)
  config.orientation_covariance_base = 0.05;      // 0.05 rad^2
  config.angular_velocity_covariance_base = 0.01; // 0.01 (rad/s)^2
  config.linear_acceleration_covariance_base = 0.05; // 0.05 (m/s^2)^2
  
  // Covariance scaling factors for calibration levels 0-3
  // Level 0 (uncalibrated): 10x baseline
  // Level 1 (poor): 5x baseline  
  // Level 2 (good): 2x baseline
  // Level 3 (excellent): 1x baseline
  config.covariance_scale_factors[0] = 10.0;
  config.covariance_scale_factors[1] = 5.0;
  config.covariance_scale_factors[2] = 2.0;
  config.covariance_scale_factors[3] = 1.0;
}

// Function to calculate scaled covariance based on calibration level  
void calculateScaledCovariance(double baseline_covariance, uint8_t calibration_level, double covariance_matrix[9]) {
  // Ensure calibration level is in valid range (0-3)
  if (calibration_level > 3) {
    calibration_level = 0; // Default to worst case if invalid
  }
  
  // Calculate scaled covariance using configured scaling factors
  double scaled_covariance = baseline_covariance * config.covariance_scale_factors[calibration_level];
  
  // Set diagonal elements to scaled value (matrix is row-major 3x3)
  covariance_matrix[0] = scaled_covariance; // [0,0]
  covariance_matrix[1] = 0.0;               // [0,1]
  covariance_matrix[2] = 0.0;               // [0,2]
  covariance_matrix[3] = 0.0;               // [1,0]
  covariance_matrix[4] = scaled_covariance; // [1,1]
  covariance_matrix[5] = 0.0;               // [1,2]
  covariance_matrix[6] = 0.0;               // [2,0]
  covariance_matrix[7] = 0.0;               // [2,1]
  covariance_matrix[8] = scaled_covariance; // [2,2]
}

// Function to set LED status based on current state
void setLedStatus(SystemState state) {
  unsigned long current_time = millis();
  
  // Turn off all LEDs first (HIGH = OFF with inverted logic)
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  
  switch (state) {
    case INITIALIZING:
      // Blue LED rapid blink (5Hz = 200ms period)
      if (current_time - last_led_toggle > 100) {
        led_state = !led_state;
        digitalWrite(LED_BLUE, led_state ? LOW : HIGH);  // LOW = ON
        last_led_toggle = current_time;
      }
      break;
      
    case AWAITING_SYNC:
      // Blue LED slow pulse (1Hz = 1000ms period)
      if (current_time - last_led_toggle > 500) {
        led_state = !led_state;
        digitalWrite(LED_BLUE, led_state ? LOW : HIGH);  // LOW = ON
        last_led_toggle = current_time;
      }
      break;
      
    case OPERATIONAL:
      // Solid Green LED
      digitalWrite(LED_GREEN, LOW);  // LOW = ON
      break;
      
    case CRITICAL_FAULT:
      // Solid Red LED
      digitalWrite(LED_RED, LOW);  // LOW = ON
      break;
  }
}

// Timer callback for publishing IMU data at 60Hz
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  // Only publish if time is synchronized AND in OPERATIONAL state
  if (timer != NULL && rmw_uros_epoch_synchronized() && current_state == OPERATIONAL) {
    
    // Get current synchronized time in nanoseconds
    int64_t now_nanos = rmw_uros_epoch_nanos();
    
    // Populate IMU message header with synchronized time
    imu_msg.header.stamp.sec = now_nanos / 1000000000LL;
    imu_msg.header.stamp.nanosec = now_nanos % 1000000000LL;
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = strlen("imu_link") + 1;
    
    // Populate orientation (quaternion)
    imu_msg.orientation.x = sensor_data.quat_i;
    imu_msg.orientation.y = sensor_data.quat_j; 
    imu_msg.orientation.z = sensor_data.quat_k;
    imu_msg.orientation.w = sensor_data.quat_real;
    
    // Populate angular velocity (rad/s)
    imu_msg.angular_velocity.x = sensor_data.gyro_x;
    imu_msg.angular_velocity.y = sensor_data.gyro_y;
    imu_msg.angular_velocity.z = sensor_data.gyro_z;
    
    // Populate linear acceleration (m/s^2) 
    imu_msg.linear_acceleration.x = sensor_data.accel_x;
    imu_msg.linear_acceleration.y = sensor_data.accel_y;
    imu_msg.linear_acceleration.z = sensor_data.accel_z;
    
    // Dynamic covariance calculation based on sensor calibration status
    // Calculate dynamic covariance matrices using sensor-specific calibration status
    calculateScaledCovariance(config.orientation_covariance_base, sensor_data.quat_calibration_status, imu_msg.orientation_covariance);
    calculateScaledCovariance(config.angular_velocity_covariance_base, sensor_data.gyro_calibration_status, imu_msg.angular_velocity_covariance);
    calculateScaledCovariance(config.linear_acceleration_covariance_base, sensor_data.accel_calibration_status, imu_msg.linear_acceleration_covariance);
    
    // Publish the message
    rcl_publish(&publisher, &imu_msg, NULL);
  }
}

// Function to disable all sensor reports
void disableAllReports() {
  delay(200);

  // A comprehensive list of all sensor report types from sh2.h
  sh2_SensorId_t allSensors[] = {
      SH2_RAW_ACCELEROMETER,
      SH2_RAW_GYROSCOPE,
      SH2_RAW_MAGNETOMETER,
      SH2_ACCELEROMETER,
      SH2_LINEAR_ACCELERATION,
      SH2_GRAVITY,
      SH2_GYROSCOPE_CALIBRATED,
      SH2_GYROSCOPE_UNCALIBRATED,
      SH2_MAGNETIC_FIELD_CALIBRATED,
      SH2_MAGNETIC_FIELD_UNCALIBRATED,
      SH2_ROTATION_VECTOR,
      SH2_GAME_ROTATION_VECTOR,
      SH2_GEOMAGNETIC_ROTATION_VECTOR,
      SH2_ARVR_STABILIZED_RV,
      SH2_ARVR_STABILIZED_GRV,
      SH2_TAP_DETECTOR,
      SH2_STEP_COUNTER,
      SH2_STEP_DETECTOR,
      SH2_SHAKE_DETECTOR,
      SH2_SIGNIFICANT_MOTION,
      SH2_STABILITY_CLASSIFIER,
      SH2_PERSONAL_ACTIVITY_CLASSIFIER,
      SH2_TEMPERATURE,
      SH2_AMBIENT_LIGHT,
      SH2_PROXIMITY
  };

  for (const auto& sensorId : allSensors) {
    if (!bno08x.enableReport(sensorId, 0)) {
      Serial.print("Could not disable report for sensor ID: 0x");
      Serial.println(sensorId, HEX);
    }
  }
  delay(200);
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // Wait for serial port to be available
  }
  delay(500); // Allow time for serial to stabilize
  Serial.println("BNO085 micro-ROS IMU Publisher");
  Serial.println("Initializing I2C and sensor...");
  
  // Configure LED pins as outputs
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // Set initial state and LED status
  current_state = INITIALIZING;
  setLedStatus(current_state);
  
  // Initialize I2C with default pins
  wire2.begin(); // Use default I2C pins
  delay(200); // Allow time for I2C to stabilize

  // Try to initialize the BNO085 sensor
  if (!bno08x.begin_I2C(BNO085_ADDR, &wire2)) {
    Serial.println("BNO085 not detected at I2C address 0x4A. Check wiring!");
    current_state = CRITICAL_FAULT;
    setLedStatus(current_state);
    while (1) {
      setLedStatus(current_state);  // Keep updating LED status
      delay(1000);
      if (!bno08x.begin_I2C(BNO085_ADDR, &wire2)) {
        Serial.println("Retrying sensor initialization...");
      } else {
        Serial.println("BNO085 re-initialized successfully!");
        current_state = INITIALIZING;
        setLedStatus(current_state);
        break; // Exit loop if initialization succeeds
      }
    }
  }
  
  Serial.println("BNO085 found! Sensor initialized successfully.");
  
  // Print sensor information
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }
  
  // Disable all sensor reports initially
  disableAllReports();

  // Enable required sensor reports as specified in PRD
  if (bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000)) {  // 1ms intervals, optimized for 60Hz publishing
    Serial.println("Game Rotation Vector enabled at 60Hz");
  } else {
    Serial.println("Could not enable Game Rotation Vector");
  }
  
  if (bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000)) {  // 1ms intervals, optimized for 60Hz publishing
    Serial.println("Calibrated Gyroscope enabled at 60Hz");
  } else {
    Serial.println("Could not enable Calibrated Gyroscope");
  }
  
  if (bno08x.enableReport(SH2_ACCELEROMETER, 1000)) {  // 1ms intervals, optimized for 60Hz publishing
    Serial.println("Accelerometer enabled at 60Hz");
  } else {
    Serial.println("Could not enable Accelerometer");
  }
  
  Serial.println("Sensor setup complete.");
  
  delay(100);
  
  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  
  // Set up micro-ROS transport (serial)
  set_microros_serial_transports(Serial);
  delay(1000);
  
  // Initialize allocator
  allocator = rcl_get_default_allocator();
  
  // Initialize support structure
  rclc_support_init(&support, 0, NULL, &allocator);
  
  // Create node
  rclc_node_init_default(&node, "bno085_publisher", "", &support);
  
  // Initialize default configuration
  initializeDefaultConfig();
  
  Serial.println("Using hard-coded default parameter values:");
  Serial.print("  publish_rate_hz: ");
  Serial.println(config.publish_rate_hz);
  Serial.print("  sync_timeout_ms: ");
  Serial.println(config.sync_timeout_ms);
  Serial.print("  orientation_covariance_base: ");
  Serial.println(config.orientation_covariance_base);
  
  // Create publisher for IMU data
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data");
  
  // Calculate timer period from configurable rate
  int64_t timer_period_ns = (int64_t)(1000000000.0 / config.publish_rate_hz);
  Serial.print("Using publish rate: ");
  Serial.print(config.publish_rate_hz);
  Serial.print(" Hz (period: ");
  Serial.print(timer_period_ns / 1000000.0);
  Serial.println(" ms)");
  
  // Create timer with configurable period
  rclc_timer_init_default2(
    &timer,
    &support,
    timer_period_ns,
    timer_callback,
    true);  // autostart
  
  // Create executor with just the timer (no parameter server)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  
  // Initialize IMU message frame_id
  imu_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
  imu_msg.header.frame_id.capacity = 20;
  
  // Initialize sensor data flags
  sensor_data.accel_ready = true;  // Allow publishing from start
  sensor_data.gyro_ready = true;
  sensor_data.quat_ready = true;
  
  Serial.println("micro-ROS initialized successfully!");
  
  // Transition to AWAITING_SYNC state
  current_state = AWAITING_SYNC;
  setLedStatus(current_state);
  
  // Wait for time synchronization with agent
  Serial.println("Waiting for time synchronization with micro-ROS agent...");
  while (!rmw_uros_epoch_synchronized()) {
    // Update LED status for AWAITING_SYNC state
    setLedStatus(current_state);
    
    // Attempt to synchronize time with 1000ms timeout
    rmw_ret_t sync_ret = rmw_uros_sync_session(1000);
    if (sync_ret == RMW_RET_OK) {
      Serial.println("Time synchronization successful!");
      break;
    } else {
      Serial.println("Time sync failed, retrying in 1 second...");
      delay(1000);
    }
  }
  
  // Transition to OPERATIONAL state
  current_state = OPERATIONAL;
  setLedStatus(current_state);
  Serial.println("Setup complete - time synchronized and ready to publish data!");
}

void loop() {
  // Update LED status based on current state
  setLedStatus(current_state);
  
  // State machine logic
  switch (current_state) {
    case OPERATIONAL:
      // Check if time sync is still active
      if (!rmw_uros_epoch_synchronized()) {
        Serial.println("Time synchronization lost! Transitioning to AWAITING_SYNC");
        current_state = AWAITING_SYNC;
        setLedStatus(current_state);
        break;
      }
      
      // Check if sensor was reset or lost connection
      if (bno08x.wasReset()) {
        Serial.println("Sensor was reset, re-enabling reports");
        // Re-enable reports after reset
        disableAllReports();
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000) ||
            !bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000) ||
            !bno08x.enableReport(SH2_ACCELEROMETER, 1000)) {
          Serial.println("Failed to re-enable sensor reports after reset!");
          current_state = CRITICAL_FAULT;
          setLedStatus(current_state);
          break;
        }
      }
      
      // Spin the executor to handle timer callbacks
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      
      // Check if new sensor data is available
      if (bno08x.getSensorEvent(&sensorValue)) {
        // Check what type of data we received
        switch (sensorValue.sensorId) {
          case SH2_ACCELEROMETER:
            // Store accelerometer data
            sensor_data.accel_x = sensorValue.un.accelerometer.x;
            sensor_data.accel_y = sensorValue.un.accelerometer.y;
            sensor_data.accel_z = sensorValue.un.accelerometer.z;
            sensor_data.accel_ready = true;
            // Store calibration status from accelerometer
            sensor_data.accel_calibration_status = sensorValue.status;
            break;
            
          case SH2_GYROSCOPE_CALIBRATED:
            // Store gyroscope data
            sensor_data.gyro_x = sensorValue.un.gyroscope.x;
            sensor_data.gyro_y = sensorValue.un.gyroscope.y;
            sensor_data.gyro_z = sensorValue.un.gyroscope.z;
            sensor_data.gyro_ready = true;
            // Store calibration status from gyroscope
            sensor_data.gyro_calibration_status = sensorValue.status;
            break;
            
          case SH2_GAME_ROTATION_VECTOR:
            // Store quaternion data
            sensor_data.quat_real = sensorValue.un.gameRotationVector.real;
            sensor_data.quat_i = sensorValue.un.gameRotationVector.i;
            sensor_data.quat_j = sensorValue.un.gameRotationVector.j;
            sensor_data.quat_k = sensorValue.un.gameRotationVector.k;
            sensor_data.quat_ready = true;
            // Store calibration status from rotation vector
            sensor_data.quat_calibration_status = sensorValue.status;
            break;
        }
      }
      break;
      
    case AWAITING_SYNC:
      // Try to synchronize time
      if (rmw_uros_epoch_synchronized()) {
        Serial.println("Time synchronization restored! Transitioning to OPERATIONAL");
        current_state = OPERATIONAL;
        setLedStatus(current_state);
      } else {
        // Attempt sync every 5 seconds as per requirements
        static unsigned long last_sync_attempt = 0;
        if (millis() - last_sync_attempt > 5000) {
          rmw_ret_t sync_ret = rmw_uros_sync_session(1000);
          if (sync_ret != RMW_RET_OK) {
            Serial.println("Time sync attempt failed, will retry in 5 seconds...");
          }
          last_sync_attempt = millis();
        }
      }
      break;
      
    case CRITICAL_FAULT:
      // Try to recover from critical fault by reinitializing sensor
      static unsigned long last_recovery_attempt = 0;
      if (millis() - last_recovery_attempt > 5000) {
        Serial.println("Attempting to recover from critical fault...");
        if (bno08x.begin_I2C(BNO085_ADDR, &wire2)) {
          Serial.println("Sensor recovery successful! Returning to INITIALIZING state");
          // Re-enable sensor reports
          disableAllReports();
          if (bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000) &&
              bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000) &&
              bno08x.enableReport(SH2_ACCELEROMETER, 1000)) {
            current_state = AWAITING_SYNC;  // Go to sync state since micro-ROS is already initialized
            setLedStatus(current_state);
          }
        }
        last_recovery_attempt = millis();
      }
      break;
      
    case INITIALIZING:
      // This state is only used during setup, should not reach here in loop
      break;
  }
}
