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
    Trajectory trajectory;
    bool gotTrajectory = false;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond> > trajcetoryDriver;
    
    if(gotTrajectory = _trajectory.read(trajectory)) {
	if(isPortUpdated(_trajectory)) {
	    //convert to driver format
	    for(std::vector<DFKI::Pose3D>::iterator it = trajectory.trajectory.begin(); it != trajectory.trajectory.end(); it++) {
		
		trajcetoryDriver.push_back(std::pair<Eigen::Vector3d, Eigen::Quaterniond> (it->position.getEigenType(), it->orientation.getEigenType()));
	    }
	    dtf.setTrajectory(trajcetoryDriver);
	}
    }
    
    if(_pose.read(pose) && gotTrajectory) {
	
	Eigen::Vector3d position = pose.position.getEigenType();
	Eigen::Quaterniond orientation = pose.orientation.getEigenType();
	
	dtf.setPose(position, orientation);
	
	dtf.testSetNextWaypoint();
	
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

