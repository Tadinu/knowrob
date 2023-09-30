#ifndef __KNOWROB_TF_REPUBLISHER__
#define __KNOWROB_TF_REPUBLISHER__

#include <string>

// MONGO
#include <mongoc.h>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <knowrob/ros/tf/memory.h>
#include <knowrob/ros/tf/publisher.h>
#include <knowrob/mongodb/MongoInterface.h>

/**
 * A TF publisher that publishes data stored in mongo DB.
 */
class TFRepublisher
{
public:
	TFRepublisher(rclcpp::Node* node, double frequency=10.0);

	~TFRepublisher();

	void set_realtime_factor(double realtime_factor)
	{ realtime_factor_ = realtime_factor; }

	void set_loop(bool loop)
	{ loop_ = loop; }

	void set_db_name(std::string db_name)
	{ db_name_ = db_name; }

    void set_db_uri(const std::string &db_uri)
    { db_uri_ = db_uri; }

	void set_db_collection(std::string db_collection)
	{ db_collection_ = db_collection; }

	TFMemory& memory()
	{ return memory_; }

	void set_goal(double time_min, double time_max);

	void set_now(double time);

	void set_progress(double percent);

	void clear();

protected:
	double realtime_factor_ = 1.0;
	double frequency_ = 0;
	bool loop_ = true;
	bool is_running_ = true;
	bool reset_ = false;
	bool skip_reset_ = false;
	bool has_been_skipped_ = false;

	double time_min_ = 0.0;
	double time_max_ = 0.0;
	double time_ = 0.0;

	std::string db_name_;
    std::string db_uri_;
	std::string db_collection_;
    std::shared_ptr<MongoCollection> collection_;

	mongoc_cursor_t *cursor_ = nullptr;
	geometry_msgs::msg::TransformStamped ts_;
	bool has_next_ = false;
	bool has_new_goal_ = false;

	TFMemory memory_;
	TFPublisher publisher_;

	std::thread thread_;
	std::thread tick_thread_;

	void loop();
	void tick_loop();
	void create_cursor(double start_time);
	void reset_cursor();
	void advance_cursor();
	void read_transform(const bson_t *doc);
	void set_initial_poses(double unix_time);
};

#endif //__KNOWROB_TF_REPUBLISHER__
