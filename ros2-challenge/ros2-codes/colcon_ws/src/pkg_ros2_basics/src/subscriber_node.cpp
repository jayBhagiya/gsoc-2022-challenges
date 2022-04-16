// include the important cpp header files
#include <memory> // dynamic memory management

// Dependencies
#include "rclcpp/rclcpp.hpp" // ros client library for cpp
#include "std_msgs/msg/string.hpp" // simple string msg 

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node {
	public:
		// constructor
		// initializing the node with name subscriber
		SimpleSubscriber()
		: Node("subscriber")
		{
			// creating subscription
			subscription_ = this->create_subscription<std_msgs::msg::String>(
					"topic", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
		}
	
	private:
		// receive the string message that is published over the topic
		void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
			RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
		}
		
		// declaration of the subscription attribute
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
	// initializing the ros2 node
	rclcpp::init(argc, argv);

	// preparing to receive the messages that arrives on the topic
	rclcpp::spin(std::make_shared<SimpleSubscriber>());

	// shutdown routine for ros2
	rclcpp::shutdown();
	return 0;
}
