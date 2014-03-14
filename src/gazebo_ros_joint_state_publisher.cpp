#include "quadrotor/gazebo_ros_joint_state_publisher.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace gazebo;

GazeboRosJointStatePublisher::GazeboRosJointStatePublisher() {}

// Destructor
GazeboRosJointStatePublisher::~GazeboRosJointStatePublisher() {
    rosnode_->shutdown();
}

void GazeboRosJointStatePublisher::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    // Store the pointer to the model
    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();

    this->robot_namespace_ = parent_->GetName ();
    if ( !_sdf->HasElement ( "robotNamespace" ) ) {
        ROS_INFO ( "GazeboRosJointStatePublisher Plugin missing <robotNamespace>, defaults to \"%s\"",
                   this->robot_namespace_.c_str() );
    } else {
        this->robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
        if ( this->robot_namespace_.empty() ) this->robot_namespace_ = parent_->GetName ();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );

    if ( !_sdf->HasElement ( "jointName" ) ) {
        ROS_ASSERT ( "GazeboRosJointStatePublisher Plugin missing jointNames" );
    } else {
        sdf::ElementPtr element = _sdf->GetElement ( "jointName" ) ;
        std::string joint_names = element->Get<std::string>();
        boost::erase_all ( joint_names, " " );
        boost::split ( joint_names_, joint_names, boost::is_any_of ( "," ) );
    }

    this->update_rate_ = 100.0;
    if ( !_sdf->HasElement ( "updateRate" ) ) {
        ROS_WARN ( "GazeboRosJointStatePublisher Plugin (ns = %s) missing <updateRate>, defaults to %f",
                   this->robot_namespace_.c_str(), this->update_rate_ );
    } else {
        this->update_rate_ = _sdf->GetElement ( "updateRate" )->Get<double>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) {
        this->update_period_ = 1.0 / this->update_rate_;
    } else {
        this->update_period_ = 0.0;
    }
    last_update_time_ = this->world_->GetSimTime();

    for ( unsigned int i = 0; i< joint_names_.size(); i++ ) {
        joints_.push_back ( this->parent_->GetJoint ( joint_names_[i] ) );
        ROS_INFO ( "joint_name: %s", joint_names_[i].c_str() );
    }

    ROS_INFO ( "Starting GazeboRosJointStatePublisher Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName ().c_str() );

    tf_prefix_ = tf::getPrefixParam ( *rosnode_ );
    joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState> ( "joint_states",1000 );

    last_update_time_ = this->world_->GetSimTime();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &GazeboRosJointStatePublisher::OnUpdate, this, _1 ) );
}

void GazeboRosJointStatePublisher::OnUpdate ( const common::UpdateInfo & _info ) {
    // Apply a small linear velocity to the model.
    common::Time current_time = this->world_->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        publishTF();

        last_update_time_+= common::Time ( update_period_ );

    }

}

void GazeboRosJointStatePublisher::publishTF() {
    //ros::Time current_time = ros::Time::now();

		ros::Time current_time;
		current_time.sec = (this->world_->GetSimTime()).sec;
		current_time.nsec = (this->world_->GetSimTime()).nsec;

    joint_state.header.stamp = current_time;
    joint_state.name.resize ( joints_.size() );
    joint_state.position.resize ( joints_.size() );

    for ( int i = 0; i < joints_.size(); i++ ) {
        physics::JointPtr joint = joints_[i];
        math::Angle angle = joint->GetAngle ( 0 );
        joint_state.name[i] = joint->GetName();
        joint_state.position[i] = angle.Radian () ;
    }
    joint_state_publisher_.publish ( joint_state );
}
