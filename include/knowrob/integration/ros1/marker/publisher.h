#ifndef __KNOWROB_MARKER_PUBLISHER__
#define __KNOWROB_MARKER_PUBLISHER__

#include <thread>
#include <map>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/**
 * A marker publisher that maps Prolog terms to marker messages.
 */
class MarkerPublisher : public rclcpp::Node
{
public:
	MarkerPublisher(const char* ns);

	/**
	 * Publish an array of marker messages.
	 */
	void publish(visualization_msgs::msg::MarkerArray &array_msg);

	/**
	 * Sets the current marker from a Prolog term and returns a reference
	 * to the marker.
	 */
	const visualization_msgs::msg::Marker& setMarker(const PlTerm &term);

protected:
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
	visualization_msgs::msg::Marker msg_;
	std::map<std::string,int> idMap_;
	int idCounter_ = 0;

	int getID(const std::string &name);
};

#endif //__KNOWROB_MARKER_PUBLISHER__
