#ifndef __KNOWROB_TF_MEMORY__
#define __KNOWROB_TF_MEMORY__

#include <string>
#include <set>
#include <map>
#include <mutex>

// MONGO
#include <mongoc.h>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**
 * A cache of most recent poses.
 */
class TFMemory
{
public:
	TFMemory();

	/**
	 * True if a transform with given frame was added before.
	 */
	bool has_transform(const std::string &frame) const;

	/**
	 * Clears cached transforms and the list of managed frames.
	 */
	bool clear();

	/**
	 * Clears cached transforms.
	 */
	bool clear_transforms_only();

	/**
	 * Get the transform associated to a frame.
	 */
	const geometry_msgs::msg::TransformStamped& get_transform(const std::string &frame, int buffer_index=0) const;

	/**
	 * Read a Prolog pose term into TransformStamped.
	 */
	void create_transform(geometry_msgs::msg::TransformStamped *ts, const std::string &frame, const PlTerm &term, double stamp);

	/**
	 * Add a transform, overwriting any previous transform with same frame.
	 */
	void set_transform(const geometry_msgs::msg::TransformStamped &ts);

	/**
	 * Add a transform, overwriting any previous transform with same frame.
	 */
	void set_managed_transform(const geometry_msgs::msg::TransformStamped &ts);

	/**
	 * Read Prolog pose term for frame.
	 */
	bool get_pose_term(const std::string &frame, PlTerm *term, double *stamp);

	/**
	 * Add a transform, overwriting any previous transform with same frame.
	 */
	bool set_pose_term(const std::string &frame, const PlTerm &term, double stamp);

	/**
	 * True for frames which are managed by the KB.
	 */
	bool is_managed_frame(const std::string &frame) const;

	bool loadTF(tf2_msgs::msg::TFMessage &tf_msg, bool clear_memory);

protected:
	std::set<std::string> managed_frames_[2];
	std::map<std::string, geometry_msgs::msg::TransformStamped> transforms_[2];
	std::mutex transforms_lock_;
	std::mutex names_lock_;
	int buffer_index_;

	void loadTF_internal(tf2_msgs::msg::TFMessage &tf_msg, int buffer_index);
};

#endif //__KNOWROB_TF_MEMORY__
