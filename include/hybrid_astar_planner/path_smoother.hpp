#ifndef __PATH_SMOOTHER__
#define __PATH_SMOOTHER__

#include <nav_msgs/msg/path.hpp>

namespace planning {

class PathSmoother {
   public:
    PathSmoother();
    void smoothPath(const nav_msgs::msg::Path::SharedPtr);
    void obstacleTerm();
    void smoothingTerm();
};

}  // end of namespace planning

#endif