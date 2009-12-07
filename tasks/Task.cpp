#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>
#include <dumbtrajectoryfollower.hpp>

using namespace dumbtrajectoryfollower;


RTT::NonPeriodicActivity* Task::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{
    dtf = new DumbTrajectoryFollower();
    gotTrajectory = false;
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
    DFKI::SystemState pose;
    Trajectory trajectory;
    std::vector<DumbTrajectoryFollower::Pose *> trajcetoryDriver;
    
    if(_trajectory.read(trajectory)) {
	if(isPortUpdated(_trajectory)) {
	    //convert to driver format
	    std::cerr << "DTF: got " << trajectory.points.size() << " points in trajectory" << std::endl;
	    for(std::vector<DFKI::Pose3D>::iterator it = trajectory.points.begin(); it != trajectory.points.end(); it++) {
		DumbTrajectoryFollower::Pose *pose_intern = new DumbTrajectoryFollower::Pose();
		pose_intern->orientation = pose.orientation.getEigenType();
		pose_intern->position = pose.position.getEigenType();
		trajcetoryDriver.push_back(pose_intern);
	    }
	    dtf->setTrajectory(trajcetoryDriver);
	    gotTrajectory = true;
	}
    }
    
    if(_pose.read(pose) && gotTrajectory) 
    {
	Eigen::Vector3d position = pose.position.getEigenType();
	Eigen::Quaterniond orientation = pose.orientation.getEigenType();
	
	dtf->setPose(position, orientation);
	
	dtf->testSetNextWaypoint();
	
	controldev::MotionCommand mc;
	dtf->getMovementCommand(mc.translation, mc.rotation);
	std::cout << "DTF: New Movement command tv " << mc.translation << " rv " << mc.rotation << std::endl;
	
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

