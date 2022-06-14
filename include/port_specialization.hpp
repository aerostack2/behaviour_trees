#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline geometry_msgs::msg::Pose convertFromString(BT::StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::msg::Pose output;
            output.position.x = convertFromString<double>(parts[0]);
            output.position.y = convertFromString<double>(parts[1]);
            output.position.z = convertFromString<double>(parts[2]);
            return output;
        }
    }
} // end namespace BT
