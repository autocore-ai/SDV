#include <turtlesim_source.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader
{
public:
  KeyboardReader()
      : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char *c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  TurtleSimSource source(argc, argv);
  return source.KeyLoop();
}

TurtleSimSource::TurtleSimSource(int argc = 0, char **argv = nullptr) : linear_(0),
                                                                        angular_(0),
                                                                        l_scale_(2.0),
                                                                        a_scale_(2.0)
{
  rclcpp::init(argc, argv);
  nh_ = rclcpp::Node::make_shared("turtle_sim_source");
  nh_->declare_parameter("scale_angular", rclcpp::ParameterValue(2.0));
  nh_->declare_parameter("scale_linear", rclcpp::ParameterValue(2.0));
  nh_->get_parameter("scale_angular", a_scale_);
  nh_->get_parameter("scale_linear", l_scale_);

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
  rotate_absolute_client_ = rclcpp_action::create_client<zf_turtlesim_msgs::action::RotateAbsolute>(nh_, "turtle1/rotate_absolute");
  clear_srv_ = nh_->create_service<std_srvs::srv::Empty>("clear", std::bind(&TurtleSimSource::clearCallback, this, std::placeholders::_1, std::placeholders::_2));

  std::thread{std::bind(&TurtleSimSource::spin, this)}.detach();

  signal(SIGINT, quit);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");
  puts("Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.");
  puts("'Q' to quit.");
}
bool TurtleSimSource::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "XXXXXXXXXXXXXXX::::Clearing turtlesim.");
  return true;
}

TurtleSimSource::~TurtleSimSource()
{
  rclcpp::shutdown();
}

void TurtleSimSource::sendGoal(float theta)
{
  auto goal = zf_turtlesim_msgs::action::RotateAbsolute::Goal();
  goal.theta = theta;
  auto send_goal_options = rclcpp_action::Client<zf_turtlesim_msgs::action::RotateAbsolute>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](std::shared_future<rclcpp_action::ClientGoalHandle<zf_turtlesim_msgs::action::RotateAbsolute>::SharedPtr> future)
  {
    RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
    this->goal_handle_ = future.get();
  };
  rotate_absolute_client_->async_send_goal(goal, send_goal_options);
}

void TurtleSimSource::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<zf_turtlesim_msgs::action::RotateAbsolute>::SharedPtr> future)
{
  RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
  this->goal_handle_ = future.get();
}

void TurtleSimSource::cancelGoal()
{
  if (goal_handle_)
  {
    RCLCPP_DEBUG(nh_->get_logger(), "Sending cancel request");
    try
    {
      rotate_absolute_client_->async_cancel_goal(goal_handle_);
    }
    catch (...)
    {
      // This can happen if the goal has already terminated and expired
    }
  }
}

void TurtleSimSource::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

void TurtleSimSource::updateTwist()
{
  twist.angular.z = a_scale_ * angular_;
  twist.linear.x = l_scale_ * linear_;
}

int TurtleSimSource::getKey()
{
  // get the next event from the keyboard
  try
  {
    input.readOne(&c);
  }
  catch (const std::runtime_error &)
  {
    perror("read():");
    return -1;
  }

  linear_ = angular_ = 0;
  RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

  switch (c)
  {
  case KEYCODE_LEFT:
    RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
    angular_ = 1.0;
    dirty = true;
    break;
  case KEYCODE_RIGHT:
    RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
    angular_ = -1.0;
    dirty = true;
    break;
  case KEYCODE_UP:
    RCLCPP_DEBUG(nh_->get_logger(), "UP");
    linear_ = 1.0;
    dirty = true;
    break;
  case KEYCODE_DOWN:
    RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
    linear_ = -1.0;
    dirty = true;
    break;
  case KEYCODE_G:
    RCLCPP_DEBUG(nh_->get_logger(), "G");
    sendGoal(0.0f);
    break;
  case KEYCODE_T:
    RCLCPP_DEBUG(nh_->get_logger(), "T");
    sendGoal(0.7854f);
    break;
  case KEYCODE_R:
    RCLCPP_DEBUG(nh_->get_logger(), "R");
    sendGoal(1.5708f);
    break;
  case KEYCODE_E:
    RCLCPP_DEBUG(nh_->get_logger(), "E");
    sendGoal(2.3562f);
    break;
  case KEYCODE_D:
    RCLCPP_DEBUG(nh_->get_logger(), "D");
    sendGoal(3.1416f);
    break;
  case KEYCODE_C:
    RCLCPP_DEBUG(nh_->get_logger(), "C");
    sendGoal(-2.3562f);
    break;
  case KEYCODE_V:
    RCLCPP_DEBUG(nh_->get_logger(), "V");
    sendGoal(-1.5708f);
    break;
  case KEYCODE_B:
    RCLCPP_DEBUG(nh_->get_logger(), "B");
    sendGoal(-0.7854f);
    break;
  case KEYCODE_F:
    RCLCPP_DEBUG(nh_->get_logger(), "F");
    cancelGoal();
    break;
  case KEYCODE_Q:
    RCLCPP_DEBUG(nh_->get_logger(), "quit");
    return 0;
  default:
    return getKey();
  }
  return c;
}

int TurtleSimSource::KeyLoop()
{
  int key = 0;
  for (;;)
  {
    key = Run();
    if (key > 0)
    {
      if (dirty == true)
      {
        twist_pub_->publish(twist);
        dirty = false;
      }
    }
    else
    {
      break;
    }
  }
  return key;
}

int TurtleSimSource::Run()
{
  auto key = getKey();
  if (key > 0)
  {
    updateTwist();
  }
  return key;
}

int TurtleSimSource::Quit()
{
  input.shutdown();
  exit(0);
}
