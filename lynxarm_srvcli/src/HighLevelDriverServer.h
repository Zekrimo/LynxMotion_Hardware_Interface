#include "rclcpp/rclcpp.hpp"  // Include ROS 2 C++ client library
#include "rclcpp_action/rclcpp_action.hpp"
#include "lynxarm_interface/srv/set_servo_position.hpp"  // Include the service message header file
#include "lynxarm_interface/action/predefined_command_action.hpp"
#include "lynxarm_interface/action/single_servo_command.hpp"
#include "LowLevelDriver.cpp"  // Include the LowLevelDriver class header file
#include "Configuration.hpp"

#include <memory>  // Include the <memory> header for smart pointers


/**
 * @brief Enumeration of predefined positions.
 */
enum predefined_positions
{
  PARK = 0,      /**< Park position */
  READY = 1,      /**< Rest position */
  STRAIGHTUP = 2 /**< Straight up position */
};


/**
 * @class LynxarmHLD
 * @brief A class representing the high-level driver for the Lynxarm robot arm.
 * 
 * This class provides high-level control functions for the Lynxarm robot arm.
 * It handles sending commands to move the servos to specified positions, sending
 * predefined commands, and initializing the Lynxarm. It also handles goal requests
 * and cancellations for predefined command actions.
 */
class LynxarmHLD : public rclcpp::Node
{
public:
  /**
   * @brief Alias for the Lynx_Command action type.
   */
  using Lynx_Command = lynxarm_interface::action::PredefinedCommandAction;   
  using Servo_Command = lynxarm_interface::action::SingleServoCommand;

  /**
   * @brief Alias for the goal handle type used in the server.
   *
   * This type represents a handle to a goal in the server. It is used to track the status and progress of a goal.
   * The goal handle is templated on the action message type.
   */
  using GoalHandle = rclcpp_action::ServerGoalHandle<Lynx_Command>;

  using GoalHandleServo = rclcpp_action::ServerGoalHandle<Servo_Command>;

  /**
   * @brief Class representing the Lynxarm High-Level Driver.
   *
   * This class provides the functionality for controlling the Lynxarm robot using high-level commands.
   * It is used as a server for handling service requests from clients.
   */
  LynxarmHLD(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  ~LynxarmHLD();

  /**
   * @brief Sends a single servo to a specified position.
   * 
   * This function sends a command to move a single servo to a specified position.
   * The servo is identified by the channel parameter, and the position is specified
   * in degrees. The time parameter determines the duration of the movement, and the
   * speed parameter controls the speed of the movement.
   * 
   * @param channel The channel of the servo to be moved.
   * @param degrees The position to which the servo should be moved, in degrees.
   * @param time The duration of the movement, in milliseconds.
   * @param speed The speed of the movement. Default value is 1.
   * @return True if the command was successfully sent, false otherwise.
   */
  bool sendSingleServoToPosition(uint8_t channel, int64_t degrees, uint64_t time, int speed = 1); //in degrees

  /**
   * @brief Sends a command to move the robot arm to a predefined position.
   * 
   * @param command The command to be sent.
   * @return True if the command was successfully sent, false otherwise.
   */
  bool sendToPredefinedPosition(std::string command);

   /**
   * @brief Sends a predefined position to the low-level driver.
   *
   * @param position The predefined position to be sent.
   * @return True if the position was successfully sent, false otherwise.
   */
  bool sendPredefinedPosition(predefined_positions position);

  
  /**
   * @brief Initializes the Lynxarm.
   * 
   * This function initializes the Lynxarm and returns a boolean value indicating
   * whether the initialization was successful or not.
   * 
   * @return true if the Lynxarm is successfully initialized, false otherwise.
   */
  bool initLynxarm();

private:
  /**
   * @brief A shared pointer to an rclcpp_action::Server object for handling predefined command actions.
   */
  rclcpp_action::Server<lynxarm_interface::action::PredefinedCommandAction>::SharedPtr action_server_;

  rclcpp_action::Server<lynxarm_interface::action::SingleServoCommand>::SharedPtr action_server_servo_;

  /**
   * @brief A shared pointer to a service object for setting the servo position.
   */
  rclcpp::Service<lynxarm_interface::srv::SetServoPosition>::SharedPtr service_;

  int64_t max_time_ = 1000000;

  const std::string serialPort;

  const uint64_t baudRate;

  // Create an instance of the LowLevelDriver class
  LowLevelDriver lowLevelDriver; // Adjust port name and baud rate as needed

  /**
   * @brief Handles the goal request for the action server.
   *
   * This function is called when a new goal is received by the action server.
   *
   * @param uuid The unique identifier of the goal.
   * @param goal The shared pointer to the goal message.
   * @return The response to the goal request.
   */
  rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const lynxarm_interface::action::PredefinedCommandAction::Goal> goal);

   rclcpp_action::GoalResponse handle_servo_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const lynxarm_interface::action::SingleServoCommand::Goal> goal);

  /**
   * @brief Handles the cancellation of a goal.
   *
   * This function is responsible for handling the cancellation of a goal in the rclcpp_action::CancelResponse.
   *
   * @param goal_handle A shared pointer to the goal handle.
   * @return The response indicating the result of the cancellation.
   */
  rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::CancelResponse handle_servo_cancel( const std::shared_ptr<GoalHandleServo> goal_handle);


  /**
   * @brief Handles the accepted goal handle.
   *
   * This function is called when a goal handle is accepted by the server.
   *
   * @param goal_handle The shared pointer to the accepted goal handle.
   */
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

   void handle_servo_accepted(const std::shared_ptr<GoalHandleServo> goal_handle);

  /**
   * Executes the given goal handle.
   *
   * @param goal_handle The goal handle to execute.
   */
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  void execute_servo(const std::shared_ptr<GoalHandleServo> goal_handle);

  /**
   * @brief Converts the given PWM value to degrees.
   * 
   * @param pwm The PWM value to be converted.
   * @return The converted value in degrees.
   */
  int convertToDegrees(int pwm);

  /**
   * @brief Converts degrees to PWM value.
   * 
   * This function takes an input angle in degrees and converts it to the corresponding PWM value.
   * 
   * @param degrees The angle in degrees.
   * @return The PWM value corresponding to the input angle.
   */
  int convertToPWM(int degrees);

  /**
   * @brief Checks if the given value is in the given range
   * 
   * @param value 
   * @param minRange 
   * @param maxRange 
   * @return int 
   */
  int64_t convertToSafePWM(int64_t value, int64_t minRange, int64_t maxRange);
  
  /**
   * @brief Sets the position of the servo.
   *
   * This function is responsible for setting the position of the servo based on the provided request.
   *
   * @param request The request containing the servo position.
   * @param response The response indicating the success or failure of the operation.
   */
  void setServoPosition( const std::shared_ptr<lynxarm_interface::srv::SetServoPosition::Request> request, std::shared_ptr<lynxarm_interface::srv::SetServoPosition::Response> response);
};



