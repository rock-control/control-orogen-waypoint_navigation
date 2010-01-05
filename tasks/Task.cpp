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
    dtf = new DumbTrajectoryFollower();
    dtf->setTrajectory(trajcetoryDriver);
    return true;
}

void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    DFKI::SystemState pose;
    Trajectory trajectory;
    
    if(isPortUpdated(_trajectory)) {
	if(_trajectory.read(trajectory)) {
	    //note, dtf deletes the Pose* for us
	    trajcetoryDriver.clear();
	    
	    //convert to driver format
	    std::cerr << "DTF: got " << trajectory.points.size() << " points in trajectory" << std::endl;
	    for(std::vector<DFKI::Pose3D>::iterator it = trajectory.points.begin(); it != trajectory.points.end(); it++) {
		DumbTrajectoryFollower::Pose *pose_intern = new DumbTrajectoryFollower::Pose();
		pose_intern->covariancePosition = Eigen::Matrix3d::Identity();
		pose_intern->covariancePosition(0,0) = _pointReachedDistanceX.get() * _pointReachedDistanceX.get();
		pose_intern->covariancePosition(1,1) = _pointReachedDistanceY.get() * _pointReachedDistanceY.get();
		pose_intern->covariancePosition(2,2) = _pointReachedDistanceZ.get() * _pointReachedDistanceZ.get();
		pose_intern->orientation = it->orientation.getEigenType();
		pose_intern->position = it->position.getEigenType();
		
		trajcetoryDriver.push_back(pose_intern);
	    }		
	    dtf->setTrajectory(trajcetoryDriver);
	    _usedTrajectory.write(trajectory);
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
	
	if(dtf->testSetNextWaypoint()) 
	{
	    std::vector<DumbTrajectoryFollower::Pose *>::const_iterator wpi = dtf->getCurrentWaypoint();
	    Waypoint wp;
	    wp.point.position = (*wpi)->position;
	    wp.point.orientation = (*wpi)->orientation;
	    wp.covarince = (*wpi)->covariancePosition;
	    
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

