#include "Task.hpp"
#include <WaypointNavigation.hpp>

using namespace waypoint_navigation;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    follower = NULL;   
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    //defaults
    _maxTv.set(0.6);
    _maxRv.set(1.5);

    follower = new WaypointNavigation();
    trajectory.clear();
    return true;
}

// bool Task::startHook()
// {
//     return true;
// }

void Task::updateHook()
{
    if(_trajectory.readNewest(trajectory) != RTT::NoData) {
        //convert to driver format
        std::cerr << "DTF: got " << trajectory.size() << " points in trajectory" << std::endl;

        std::vector<base::Waypoint*> waypoints;
        for (std::vector<base::Waypoint>::const_iterator it = trajectory.begin();
                it != trajectory.end(); ++it)
        {
            waypoints.push_back(new base::Waypoint(*it));
        }
        follower->setTrajectory(waypoints);
    }
    
    base::samples::RigidBodyState pose;
    if (!trajectory.empty() && _pose.readNewest(pose) != RTT::NoData)
    {
	follower->setPose(pose);
	
	if(follower->testSetNextWaypoint()) 
	{
	    std::vector<base::Waypoint *>::const_iterator wpi = follower->getCurrentWaypoint();
	    _currentWaypoint.write(**wpi);
	}
	
	base::MotionCommand2D mc;
	follower->getMovementCommand(mc.translation, mc.rotation);
	
	if(mc.translation > _maxTv.get())
	    mc.translation = _maxTv.get();
	
	if(mc.translation < -_maxTv.get())
	    mc.translation = -_maxTv.get();

	if(mc.rotation > _maxRv.get())
	    mc.rotation = _maxRv.get();

	if(mc.rotation < -_maxRv.get())
	    mc.rotation = -_maxRv.get();

	_motion_command.write(mc);
    }
}

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }
void Task::cleanupHook()
{
    delete follower;
}

