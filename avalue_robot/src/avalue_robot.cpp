#include <atomic>
#include <csignal>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "avalue_robot/Quaternion_Solution.h"
#include "avalue_robot/avalue_robot.h"
#include "avalue_robot_msg/msg/data.hpp"

static std::atomic<bool> g_should_exit{ false };

// sensor_msgs::Imu Mpu6050;
// Instantiate an IMU object 
sensor_msgs::msg::Imu Mpu6050;
using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;

static void handle_exit_signal(int signum)
{
    (void)signum;
    g_should_exit = true;
    // Set rclcpp::ok() to false, and the Control() loop will naturally exit.
    rclcpp::shutdown();
}

/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control
object through the Turn_on_robot class and automatically calls the constructor
initialization 
***************************************/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // auto node= std::make_shared<turn_on_robot>();

    // Register Signal
    signal(SIGHUP, handle_exit_signal);
    signal(SIGTERM, handle_exit_signal);
    signal(SIGINT, handle_exit_signal);

    turn_on_robot Robot_Control;
    Robot_Control.Control();

    return 0;
}

/**************************************
Date: January 28, 2021
Function: Data conversion function
***************************************/

short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;
    transition_16 |= Data_Low;
    return transition_16;
}

float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
	
	// Get the high 8 bits of data	
    transition_16 |= Data_High << 8; 
	// Get the lowest 8 bits of data
    transition_16 |= Data_Low; 
                               
	// The speed unit is changed from mm/s to m/s
    data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;
                                 // 
    return data_return;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the
subscribed instructions through the serial port command control of the lower
computer
***************************************/

void turn_on_robot::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
	// intermediate variable
    short transition;				
    
	/* 
	if(akm_cmd_vel=="ackermann_cmd") 
	{
		// Prompt message
		RCLCPP_INFO(this->get_logger(),"is akm");
	}
	*/	
	
	// Frame head 0x7B
    Send_Data.tx[0] = FRAME_HEADER;
	// Set aside
    Send_Data.tx[1] = 0;
	// Set aside
    Send_Data.tx[2] = 0;

    // The target velocity of the X-axis of the robot
    transition = 0;
    transition = akm_ctl->drive.speed * 1000;	// Magnify floating-point numbers by a thousand to simplify transmission
    Send_Data.tx[4] = transition;				// Take the lower 8 bits of the data
    Send_Data.tx[3] = transition >> 8;			// Take the upper 8 bits of the data

    // The target velocity of the Y-axis of the robot
    // transition=0;
    // transition = twist_aux->linear.y*1000;
    // Send_Data.tx[6] = transition;
    // Send_Data.tx[5] = transition>>8;

	// The target angular velocity of the robot's Z axis
    transition = 0;
    transition = akm_ctl->drive.steering_angle * 1000 / 2;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition >> 8;

	// For the BBC check bits, see the Check_Sum function
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); 
                      
	// Frame tail 0x7D
    Send_Data.tx[10] = FRAME_TAIL;

    try
    {
		// Sends data to the downloader via serial port
		Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); 
	}
    catch (serial::IOException &e)
	{
		// If sending data fails, an error message is printed
		RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port")); 
	}
}

// void turn_on_robot::Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
	// intermediate variable
    short transition; 
	/*
    if(akm_cmd_vel=="none") 
	{
		// 提示信息
		// Prompt message 
		RCLCPP_INFO(this->get_logger(),"not akm");
	}
    */
	
	// Frame head 0x7B
    Send_Data.tx[0] = FRAME_HEADER;
	// Set aside
    Send_Data.tx[1] = 0;
	// Set aside
    Send_Data.tx[2] = 0;

	// The target velocity of the X-axis of the robot
    transition = 0;
	transition = twist_aux->linear.x * 1000;	// Magnify floating-point numbers by a thousand to simplify transmission
	Send_Data.tx[4] = transition;				// Take the lower 8 bits of the data
	Send_Data.tx[3] = transition >> 8;			// Take the upper 8 bits of the data

    // The target velocity of the Y-axis of the robot
    transition = 0;
    transition = twist_aux->linear.y * 1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition >> 8;

    // The target angular velocity of the robot's Z axis
    transition = 0;
    transition = twist_aux->angular.z * 1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition >> 8;

	// For the BBC check bits, see the Check_Sum function
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
	
	// Frame tail 0x7D
    Send_Data.tx[10] = FRAME_TAIL;

    try
	{
		if (akm_cmd_vel == "none")
		{
			// Sends data to the downloader via serial port
			Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
		}         
	}
    catch (serial::IOException &e)
	{
		// If sending data fails, an error message is printed
		RCLCPP_ERROR (this->get_logger(), ("Unable to send data through serial port"));
	}
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
***************************************/

void turn_on_robot::Publish_ImuSensor()
{
	// Instantiate IMU topic data
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
	// IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack
    Imu_Data_Pub.header.frame_id = gyro_frame_id; 
                         
    // A quaternion represents a three-axis attitude
    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; 

    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
	
	// Three-axis attitude covariance matrix 
    Imu_Data_Pub.orientation_covariance[0] = 1e6; 
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;
	
	// Triaxial angular velocity 
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; 
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
	
	// Triaxial angular velocity covariance matrix
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; 
               
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
	
	// Triaxial acceleration 
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; 
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

    imu_publisher->publish(Imu_Data_Pub);
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial
velocity, angular velocity about triaxial, TF parent-child coordinates, and
covariance matrix
***************************************/

void turn_on_robot::Publish_Odom()
{
    // Convert the Z-axis rotation Angle into a quaternion for expression
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    avalue_robot_msg::msg::Data robotpose;
    avalue_robot_msg::msg::Data robotvel;
	
	// Instance the odometer topic data
    nav_msgs::msg::Odometry odom; 

    odom.header.stamp = rclcpp::Node::now();
	
	// Odometer TF parent coordinates
    odom.header.frame_id = odom_frame_id; 
	// Odometer TF subcoordinates
    odom.child_frame_id = robot_frame_id;

	// Position
    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
	
	// Posture, Quaternion converted by Z-axis rotation
    odom.pose.pose.orientation = odom_quat;

	// Speed in the X direction
    odom.twist.twist.linear.x = Robot_Vel.X;
	// Speed in the Y direction
    odom.twist.twist.linear.y = Robot_Vel.Y;
	// Angular velocity around the Z axis
    odom.twist.twist.angular.z = Robot_Vel.Z;

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

    /*   
	geometry_msgs::msg::TransformStamped odom_tf;

	odom_tf.header = odom.header;
	odom_tf.child_frame_id = odom.child_frame_id;
	odom_tf.header.stamp = rclcpp::Node::now();

	odom_tf.transform.translation.x = odom.pose.pose.position.x;
	odom_tf.transform.translation.y = odom.pose.pose.position.y;
	odom_tf.transform.translation.z = odom.pose.pose.position.z;
	odom_tf.transform.rotation = odom.pose.pose.orientation;

	tf_bro->sendTransform(odom_tf);
	*/
	
	// There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack    
    // tf_pub_->publish(odom_tf);
	// Publish odometer topic 
    odom_publisher->publish(odom); 
	// Publish odometer topic
    robotpose_publisher->publish(robotpose); 
	// Publish odometer topic
    robotvel_publisher->publish(robotvel);
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
***************************************/

void turn_on_robot::Publish_Voltage()
{
	// Define the data type of the power supply voltage publishing topic
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;
	
    if (Count_Voltage_Pub++ > 10)
	{
		Count_Voltage_Pub = 0;
		// The power supply voltage is obtained
		voltage_msgs.data = Power_voltage;
		
		// Post the power supply voltage topic unit: V, volt 
		voltage_publisher->publish(voltage_msgs);
	}
}


/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH
-1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1
to byte n-2 are compared with byte n-1, which is a BBC check Input parameter:
Count_Number: Check the first few bytes of the packet
***************************************/

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
    unsigned char check_sum = 0, k;

	// Receive data mode
    if (mode == 0) 
	{
		for (k = 0; k < Count_Number; k++)
		{
			// By bit or by bit 
			check_sum = check_sum ^ Receive_Data.rx[k];
		}
	}

	// Send data mode
	if (mode == 1) 
	{
		for (k = 0; k < Count_Number; k++)
		{
			// By bit or by bit
			check_sum = check_sum ^ Send_Data.tx[k];
		}
	}
	
	// Returns the bitwise XOR result
    return check_sum;
}

/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower
computer, and then the data is converted to international units
***************************************/

bool turn_on_robot::Get_Sensor_Data()
{
	// Intermediate variable
    short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0;
	// Temporary variable to save the data of the lower machine
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = { 0 };

	// Read the data sent by the lower computer through the serial port
    Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));
	
	
	// Record the position of the head and tail of the frame
    for (j = 0; j < 24; j++)
	{
		if (Receive_Data_Pr[j] == FRAME_HEADER)
		{
			Header_Pos = j;
		}
		else if (Receive_Data_Pr[j] == FRAME_TAIL)
		{
			Tail_Pos = j;
        }
	}

	if (Tail_Pos == (Header_Pos + 23))
	{
		// If the end of the frame is the last bit of the packet, copy the packet directly to receive_data.rx
		//  ROS_INFO("1----");
		memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
	}
	else if (Header_Pos == (1 + Tail_Pos))
	{
		//  If the header is behind the end of the frame, copy the packet to receive_data.rx after correcting the data location
		//  ROS_INFO("2----");
		for (j = 0; j < 24; j++)
		{
			Receive_Data.rx[j] = Receive_Data_Pr[(j + Header_Pos) % 24];
		}
	}
    else
	{
		//  In other cases, the packet is considered to be faulty
		//  ROS_INFO("3----");
		return false;
	}

	// The first part of the data is the frame header 0X7B
	Receive_Data.Frame_Header = Receive_Data.rx[0];
	
	// The last bit of data is frame tail 0X7D
    Receive_Data.Frame_Tail = Receive_Data.rx[23];
	
	// Judge the frame header
    if (Receive_Data.Frame_Header == FRAME_HEADER)
    {
		// Judge the end of the frame
        if (Receive_Data.Frame_Tail == FRAME_TAIL)
        {
            // BBC check passes or two packets are interlaced
            if (Receive_Data.rx[22] == Check_Sum (22, READ_DATA_CHECK)
			|| (Header_Pos == (1 + Tail_Pos)))
            {
				// Set aside
                Receive_Data.Flag_Stop = Receive_Data.rx[1];
				// Get the speed of the moving chassis in the X direction
                Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
                // Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);                
				// Get the speed of the moving chassis in the Z direction
                Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);
                
                // MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
				// Get the X-axis acceleration of the IMU
                Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]); 
				// Get the Y-axis acceleration of the IMU
                Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
				// Get the Z-axis acceleration of the IMU
                Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
				// Get the X-axis angular velocity of the IMU
                Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
				// Get the Y-axis angular velocity of the IMU
                Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
				// Get the Z-axis angular velocity of the IMU
                Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

				// Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2                
                Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
                Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
                Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
                
				// The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s 
				// Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
                Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

                // Get the battery voltage
                transition_16 = 0;
                transition_16 |= Receive_Data.rx[20] << 8;
                transition_16 |= Receive_Data.rx[21];
				// Unit conversion millivolt(mv)->volt(v)
                Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;

                return true;
            }
        }
    }

    return false;
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
***************************************/

void turn_on_robot::Control()
{
    rclcpp::Time current_time, last_time;
    current_time = rclcpp::Node::now();
    last_time = rclcpp::Node::now();
	
	while (rclcpp::ok())
	{
		try
		{
			current_time = rclcpp::Node::now();
			// Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage)             
			Sampling_Time = (current_time - last_time).seconds();

			// The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
			if (true == Get_Sensor_Data())
			{
				// Calculate the displacement in the X direction, unit: m 
				Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
				// Calculate the displacement in the Y direction, unit: m
				Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
				// The angular displacement about the Z axis, in rad
				Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

				// Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
				
				Quaternion_Solution(Mpu6050.angular_velocity.x,
									Mpu6050.angular_velocity.y,
									Mpu6050.angular_velocity.z,
									Mpu6050.linear_acceleration.x,
									Mpu6050.linear_acceleration.y,
									Mpu6050.linear_acceleration.z);
									
				// Publish the IMU topic
				Publish_ImuSensor();

				// Publish the topic of power supply voltage
				Publish_Voltage();

				Publish_Odom();
			}

             rclcpp::spin_some(this->get_node_base_interface());

             // Record the time and use it to calculate the time interval
             last_time = current_time;
             // Delay to avoid busy loop
             std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_ERROR (this->get_logger(), "unexpectedly failed with %s", e.what());
        }
    }
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
***************************************/

turn_on_robot::turn_on_robot() : rclcpp::Node("avalue_robot")
{
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

    int serial_baud_rate = 115200;

    this->declare_parameter<int>("serial_baud_rate", 115200);
    // this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
    this->declare_parameter<std::string>("usart_port_name", "/dev/avalue_controller");
    this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
    this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_frame_id", "base_link");
    this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

    this->get_parameter("serial_baud_rate", serial_baud_rate);
    this->get_parameter("usart_port_name", usart_port_name);
    this->get_parameter("cmd_vel", cmd_vel);
    this->get_parameter("akm_cmd_vel", akm_cmd_vel);
    this->get_parameter("odom_frame_id", odom_frame_id);
    this->get_parameter("robot_frame_id", robot_frame_id);
    this->get_parameter("gyro_frame_id", gyro_frame_id);

    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);
    // odom_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });

    imu_publisher = create_publisher<sensor_msgs::msg::Imu>("mobile_base/sensors/imu_data", 2);
    // imu_timer = create_wall_timer(1s/100, [=]() { Publish_ImuSensor(); });

    voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
    // voltage_timer = create_wall_timer(1s/100, [=]() { Publish_Voltage(); });
    // tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    robotpose_publisher = create_publisher<avalue_robot_msg::msg::Data>("robotpose", 10);
    // robotpose_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });

    robotvel_publisher = create_publisher<avalue_robot_msg::msg::Data>("robotvel", 10);
    // robotvel_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });
    tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(cmd_vel, 1, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

    Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(akm_cmd_vel, 1, std::bind(&turn_on_robot::Akm_Cmd_Vel_Callback, this, _1));

    try
	{
		// Attempts to initialize and open the serial port
		// Select the serial port number to enable
        Stm32_Serial.setPort(usart_port_name); 

		// Configure Baudrate
		Stm32_Serial.setBaudrate(serial_baud_rate);
		// Timeout
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
        Stm32_Serial.setTimeout(_time);
		// Open the serial port
        Stm32_Serial.open();
    }
    catch (serial::IOException &e)
	{
		// If opening the serial port fails, an error message is printed
		RCLCPP_ERROR(this->get_logger(), "avalue_robot can not open serial port,Please check the serial port cable! ");
	}
	
    if (Stm32_Serial.isOpen())
	{
		try 
		{
			// Raise DTR indicates the connection has been established
			Stm32_Serial.setDTR(true);   
			// Lower RTS as needed to prevent blocking
			Stm32_Serial.setRTS(false);
		} 
		catch (const serial::IOException& e) 
		{
			RCLCPP_WARN(this->get_logger(), "Failed to set DTR/RTS after opening port: %s", e.what());
		
		} 
		catch (const std::exception& e) 
		{
			RCLCPP_WARN(this->get_logger(), "Unexpected error while setting DTR/RTS: %s", e.what());
		}
		
		// Serial port opened successfully
		RCLCPP_INFO(this->get_logger(), "avalue_robot serial port opened");
	}
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object
ends its life cycle
***************************************/

turn_on_robot::~turn_on_robot()
{
	// Sends the stop motion command to the lower machine before the turn_on_robot object ends 
    Send_Data.tx[0] = FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;

	// The target velocity of the X-axis of the robot
    Send_Data.tx[4] = 0;
    Send_Data.tx[3] = 0;

	// The target velocity of the Y-axis of the robot
    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;

	// The target velocity of the Z-axis of the robot
    Send_Data.tx[8] = 0;
    Send_Data.tx[7] = 0;
	// Check the bits for the Check_Sum function
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); 
	
    Send_Data.tx[10] = FRAME_TAIL;

	if (Stm32_Serial.isOpen()) 
	{
		try
		{
			// Send data to the serial port
			Stm32_Serial.write(Send_Data.tx, sizeof (Send_Data.tx)); 
		}
		catch (serial::IOException &e)
		{
			// If data transmission fails, print an error message
			RCLCPP_ERROR(this->get_logger (), "Unable to send data through serial port");
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->get_logger (), "Unexpected error while writing to serial: %s", e.what ());
		}

		try
		{
			// Pull DTR high before shutdown to prevent the bridge chip from maintaining DTR=0 during shutdown
			Stm32_Serial.setDTR(true);
			// If necessary, you can also lower the RTS to ensure it does not block transmission
			Stm32_Serial.setRTS(false); 
		}
		catch (const serial::IOException& e)
		{
			RCLCPP_WARN(this->get_logger (), "Failed to set DTR/RTS before close");
		}
		catch (const std::exception& e) 
		{
		  RCLCPP_ERROR(this->get_logger(), "Unexpected error when closing serial: %s", e.what());
		}
		
		// Close the serial port
		Stm32_Serial.close(); 
	}
	
	// Serial port closed successfully
    RCLCPP_INFO(this->get_logger(), "Shutting down");
}