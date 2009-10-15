#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace dumbtrajectoryfollower;


RTT::NonPeriodicActivity* Task::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    return true;
}
// bool Task::startHook()
// {
//     return true;
// }

void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    DFKI::Pose3D pose;
    DFKI::Pose3D targetPose;
    bool gotTarget = false;
    
    if(gotTarget = _targetPose.read(targetPose)) {
	if(isPortUpdated(_targetPose)) {
	    dtf.setTargetPose(targetPose.position.getEigenType(), targetPose.orientation.getEigenType());
	}
    }
    
    if(_pose.read(pose) && gotTarget) {
	dtf.setPose(pose.position.getEigenType(), pose.orientation.getEigenType());
	
	controldev::MotionCommand mc;
	dtf.getMovementCommand(mc.translation, mc.rotation);
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

