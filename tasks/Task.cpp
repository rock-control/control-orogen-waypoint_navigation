#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>
#include <WaypointNavigation.hpp>
#include <base/wrappers/waypoint.h>

using namespace waypoint_navigation;


RTT::NonPeriodicActivity* Task::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{
    dtf = NULL;   
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    //defaults
    _pointReachedDistanceX.set(1.2);
    _pointReachedDistanceY.set(1.2);
    _pointReachedDistanceZ.set(5.0);

    _maxTv.set(0.6);
    _maxRv.set(1.5);
    
    return true;
}

bool Task::startHook()
{
    if(dtf) {
	delete dtf;
    }
    dtf = new WaypointNavigation();
    dtf->setTrajectory(trajcetoryDriver);
    return true;
}

void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    wrappers::samples::RigidBodyState pose;
    std::vector<wrappers::Waypoint> trajectory;
    
    if(isPortUpdated(_trajectory)) {
	if(_trajectory.read(trajectory)) {
	    //note, dtf deletes the Pose* for us
	    trajcetoryDriver.clear();
	    
	    //convert to driver format
	    std::cerr << "DTF: got " << trajectory.size() << " points in trajectory" << std::endl;
	    for(std::vector<wrappers::Waypoint>::iterator it = trajectory.begin(); it != trajectory.end(); it++) {
		base::Waypoint *wp_intern = new base::Waypoint();
		*wp_intern = *it;		
		trajcetoryDriver.push_back(wp_intern);
	    }		
	    dtf->setTrajectory(trajcetoryDriver);
	    _usedTrajectory.write(trajectory);
	}
    }
    
    if(_pose.read(pose)) 
    {
	base::samples::RigidBodyState rbs = pose;	
	
	dtf->setPose(rbs);
	
	if(dtf->testSetNextWaypoint()) 
	{
	    std::vector<base::Waypoint *>::const_iterator wpi = dtf->getCurrentWaypoint();
	    wrappers::Waypoint wp = **wpi;;	    
	    _currentWaypoint.write(wp);
	}
	
	controldev::MotionCommand mc;
	dtf->getMovementCommand(mc.translation, mc.rotation);
	std::cout << "DTF: New Movement command tv " << mc.translation << " rv " << mc.rotation << std::endl;
	
	if(mc.translation > _maxTv.get())
	    mc.translation = _maxTv.get();
	
	if(mc.translation < -_maxTv.get())
	    mc.translation = -_maxTv.get();

	if(mc.rotation > _maxRv.get())
	    mc.rotation = _maxRv.get();

	if(mc.rotation < -_maxRv.get())
	    mc.rotation = -_maxRv.get();

	_motionCommand.write(mc);
    }
}

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }
// void Task::cleanupHook()
// {
// }

