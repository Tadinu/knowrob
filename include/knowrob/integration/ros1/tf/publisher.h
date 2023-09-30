#ifndef __KNOWROB_TF_PUBLISHER__
#define __KNOWROB_TF_PUBLISHER__

#include <thread>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <knowrob/ros/tf/memory.h>

/**
 * TF publisher for frames that are managed by the KB.
 */
class TFPublisher
{
public:
	TFPublisher(rclcpp::Node* node, TFMemory &memory,
			    double frequency=10.0,
			    bool clear_after_publish=false);
	~TFPublisher();

protected:
	rclcpp::Node* node_ = nullptr;
	TFMemory &memory_;
	bool is_running_ = false;
	double frequency_ = 0;
	bool clear_after_publish_ = false;
	std::thread thread_;

	void publishTransforms(tf2_ros::TransformBroadcaster &tf_broadcaster);
	void loop();
};

#endif //__KNOWROB_TF_PUBLISHER__
