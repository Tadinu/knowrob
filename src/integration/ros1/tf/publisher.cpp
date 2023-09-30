#include <knowrob/ros/tf/publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>

// TODO: handle static object transforms (e.g. for features, srdl components also have static transforms relative to base link)

TFPublisher::TFPublisher(rclcpp::Node* node, TFMemory &memory, double frequency, bool clear_after_publish) :
		node_(node),
		memory_(memory),
		is_running_(true),
		frequency_(frequency),
		clear_after_publish_(clear_after_publish),
	    thread_(&TFPublisher::loop, this)
{
}

TFPublisher::~TFPublisher()
{
	is_running_ = false;
	thread_.join();
}

void TFPublisher::loop()
{
	auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
	rclcpp::Rate r(frequency_);
	while(rclcpp::ok()) {
		publishTransforms(*tf_broadcaster.get());
		r.sleep();
		if(!is_running_) break;
	}
}

void TFPublisher::publishTransforms(tf2_ros::TransformBroadcaster &tf_broadcaster)
{
	tf2_msgs::msg::TFMessage tf_msg;
	memory_.loadTF(tf_msg, clear_after_publish_);
	tf_broadcaster.sendTransform(tf_msg.transforms);
}
