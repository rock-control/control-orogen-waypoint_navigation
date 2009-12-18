#ifndef DUMBTRAJECTORYFOLLOWER_TYPES_HPP
#define DUMBTRAJECTORYFOLLOWER_TYPES_HPP

#ifndef __orogen
#include <vector>
#endif
#include <dfki/base_types.h>

namespace dumbtrajectoryfollower {

struct Waypoint {
    DFKI::Pose3D point;
    //defines how near robot mus be to a point
    //so that it is marked as reached
    DFKI::Matrix3 covarince;
};

struct Trajectory {
    std::vector<DFKI::Pose3D> points;
};


}

#endif