#ifndef DUMBTRAJECTORYFOLLOWER_TYPES_HPP
#define DUMBTRAJECTORYFOLLOWER_TYPES_HPP

#ifndef __orogen
#include <vector>
#endif
#include <dfki/base_types.h>

namespace dumbtrajectoryfollower {

struct Trajectory {
    std::vector<DFKI::Pose3D> points;
};

}

#endif