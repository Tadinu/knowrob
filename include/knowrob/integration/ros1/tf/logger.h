#ifndef __KNOWROB_TF_LOGGER__
#define __KNOWROB_TF_LOGGER__

#include <string>

// MONGO
#include <mongoc.h>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <knowrob/ros/tf/memory.h>

/**
 * A TF listener that stores messages in MongoDB.
 */
class TFLogger
{
public:
	TFLogger(rclcpp::Node* node,
			TFMemory &memory,
			const std::string &topic="tf");
	~TFLogger();

	void set_db_name(const std::string &db_name)
	{ db_name_ = db_name; }

    void set_db_uri(const std::string &db_uri)
    { db_uri_ = db_uri; }

	const std::string& get_db_name()
	{ return db_name_; }

	void set_time_threshold(double v)
	{ timeThreshold_ = v; }

	double get_time_threshold() const
	{ return timeThreshold_; }

	void set_vectorial_threshold(double v)
	{ vectorialThreshold_ = v; }

	double get_vectorial_threshold() const
	{ return vectorialThreshold_; }

	void set_angular_threshold(double v)
	{ angularThreshold_ = v; }

	double get_angular_threshold() const
	{ return angularThreshold_; }

	void store(const geometry_msgs::msg::TransformStamped &ts);

protected:
	TFMemory &memory_;
	double vectorialThreshold_ = 0.001;
	double angularThreshold_ = 0.001;
	double timeThreshold_ = -1.0;
	std::string db_name_ = "roslog";
    std::string db_uri_;
	std::string topic_;

	char buf_[16];
	size_t keylen_ = 0;

	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_static_;

	void store_document(bson_t *doc);

	bool ignoreTransform(const geometry_msgs::msg::TransformStamped &ts);

	void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

	void appendTransform(bson_t *ts_doc,
			const geometry_msgs::msg::TransformStamped &ts);
};

#endif //__KNOWROB_TF_LOGGER__
