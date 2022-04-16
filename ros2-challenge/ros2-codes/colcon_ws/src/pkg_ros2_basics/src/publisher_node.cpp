// including important c++ header files.
#include <chrono>	// for date and time 
#include <functional>	// for arithmetic and logical operations
#include <memory>	// dynamic memory management
#include <string>	// string function

// ros client library for cpp
#include "rclcpp/rclcpp.hpp"

// built-in message type that will be used to publish string message
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// creating node class named SimplePublisher.
// Which inherits the arrtribites and methods of rclcpp::Node class
class SimplePublisher : public rclcpp::Node {
	public:
		// constructor that creates a node named publisher.
		// publisher message count is initialized to 0.
		SimplePublisher()
		: Node("publisher")
		{
			// publisher publishes the String message to topic named "topic"
			// with the queue message size of 10.
			publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
			
			// initialize the timer, the timer_callback function will be executed every
			// 500 milliseconds.
			timer_ = this->create_wall_timer(500ms,
			std::bind(&SimplePublisher::timer_callback, this));
		}
	
	private:
		void timer_callback() {
			// new message of type String
			auto msg = std_msgs::msg::String();
			
			// simple message data
			msg.data = "Hello! ROS2 is fun";

			// printing message data on terminal
			RCLCPP_INFO(this->get_logger(), "Publshing: '%s',", msg.data.c_str());

			// publishing data on topic name "topic"
			publisher_->publish(msg);	
		}

		// declaration of the time_ attribute
		rclcpp::TimerBase::SharedPtr timer_;
		
		// declaration of the publisher_ attribute
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]){
	// initialize the ros2 node
	rclcpp::init(argc, argv);

	// sarting the data processing from node as well as the callbacks and the timer
	rclcpp::spin(std::make_shared<SimplePublisher>());

	// shuting down the node when finished
	rclcpp::shutdown();

	return 0;
}

