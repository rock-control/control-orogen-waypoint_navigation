#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>
#include <dumbtrajectoryfollower.hpp>

using namespace dumbtrajectoryfollower;


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
    return true;
}

bool Task::startHook()
{
    if(dtf) {
	delete dtf;
    }
    dtf = new DumbTrajectoryFollower();
    dtf->setTrajectory(trajcetoryDriver);
    return true;
}

void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    DFKI::SystemState pose;
    Trajectory trajectory;
    
    if(_trajectory.read(trajectory)) {
	if(isPortUpdated(_trajectory)) {
	    //note, dtf deletes the Pose* for us
	    trajcetoryDriver.clear();
	    
	    //convert to driver format
	    std::cerr << "DTF: got " << trajectory.points.size() << " points in trajectory" << std::endl;
	    for(std::vector<DFKI::Pose3D>::iterator it = trajectory.points.begin(); it != trajectory.points.end(); it++) {
		DumbTrajectoryFollower::Pose *pose_intern = new DumbTrajectoryFollower::Pose();
		//HACK this specifys, that the position is "reached" if we are in a 0.2m radius to it
		//note, the robot will also stop if the covariance of the pose is bigger than (0.2)^2
		pose_intern->covariancePosition = Eigen::Matrix3d::Identity() * 0.04; 
		pose_intern->orientation = pose.orientation.getEigenType();
		pose_intern->position = pose.position.getEigenType();
		trajcetoryDriver.push_back(pose_intern);
	    }
	    dtf->setTrajectory(trajcetoryDriver);
	}
    }
    
    if(_pose.read(pose)) 
    {
	DumbTrajectoryFollower::Pose poseDTF;
	poseDTF.position = pose.position.getEigenType();
	poseDTF.orientation = pose.orientation.getEigenType();
	poseDTF.covariancePosition = pose.cov_position.getEigenType();
	poseDTF.covarianceOrientation = pose.cov_orientation.getEigenType();
	
	dtf->setPose(poseDTF);
	
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

