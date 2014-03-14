#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// ROS
#include <ros/ros.h>

// Usage in URDF:
//   <gazebo>
//       <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
//      <robotNamespace>/pioneer2dx</robotNamespace>
//      <jointName>chassis_swivel_joint, swivel_wheel_joint, left_hub_joint, right_hub_joint</jointName>
//      <updateRate>100.0</updateRate>
//      <alwaysOn>true</alwaysOn>
//      <offset>0.0</offset>
//      <amplitude>0.5</amplitude>
//      <frequency>10.0</frequency>
//      <phase>0.0</phase>
//       </plugin>
//   </gazebo>

namespace gazebo {
class JointTilterController : public ModelPlugin {
public:
    JointTilterController();
    ~JointTilterController();
    void Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
    void OnUpdate ( const common::UpdateInfo & _info );
    double create_sine(double time);
    void joint_control(double sine);

    // Pointer to the model
private:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world_;
    physics::ModelPtr parent_;
    physics::JointPtr joints_;

    // ROS STUFF
    boost::shared_ptr<ros::NodeHandle> rosnode_;
    std::string robot_namespace_;
    std::string joint_names_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

    // Sine stuff
    double offset_;               /**< DC offset of the sine wave. */
    double amplitude_;            /**< Amplitude of the sine wave. */
    double frequency_;            /**< Frequency of the sine wave. */
    double phase_;                /**< Phase of the sine wave at t=0. */
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (JointTilterController);
}

using namespace gazebo;

// Constructor
JointTilterController::JointTilterController() {}

// Destructor
JointTilterController::~JointTilterController() {
    rosnode_->shutdown();
}

void JointTilterController::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    // Store the pointer to the model
    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();

    this->robot_namespace_ = parent_->GetName ();
    if ( !_sdf->HasElement ( "robotNamespace" ) ) {
        ROS_INFO ( "JointTilterController Plugin missing <robotNamespace>, defaults to \"%s\"",
                   this->robot_namespace_.c_str() );
    } else {
        this->robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
        if ( this->robot_namespace_.empty() ) this->robot_namespace_ = parent_->GetName ();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );

    if ( !_sdf->HasElement ( "jointName" ) ) {
        ROS_ASSERT ( "JointTilterController Plugin missing jointNames" );
    } else {
        sdf::ElementPtr element = _sdf->GetElement ( "jointName" ) ;
        std::string joint_names = element->Get<std::string>();
        boost::erase_all ( joint_names, " " );
        joint_names_ = joint_names;
    }

    this->update_rate_ = 100.0;
    if ( !_sdf->HasElement ( "updateRate" ) ) {
        ROS_WARN ( "JointTilterController Plugin (ns = %s) missing <updateRate>, defaults to %f",
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

    joints_ = this->parent_->GetJoint ( joint_names_ );
    ROS_INFO ( "joint_name: %s", joint_names_.c_str() );

    ROS_INFO ( "Starting JointTilterController Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName ().c_str() );


    last_update_time_ = this->world_->GetSimTime();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &JointTilterController::OnUpdate, this, _1 ) );

    // Valori di default per la funzione seno
    offset_ = 0.0;
    amplitude_ = 0.3;
    frequency_ = 1.0;
    phase_ = 0.0;

    if ( !_sdf->HasElement ( "offset" ) ) {
        ROS_WARN ( "JointTilterController Plugin (ns = %s) missing <offset>, defaults to %f",
                   this->robot_namespace_.c_str(), this->offset_ );
    } else {
        this->offset_ = _sdf->GetElement ( "offset" )->Get<double>();
    }

    if ( !_sdf->HasElement ( "amplitude" ) ) {
        ROS_WARN ( "JointTilterController Plugin (ns = %s) missing <amplitude>, defaults to %f",
                   this->robot_namespace_.c_str(), this->amplitude_ );
    } else {
        this->amplitude_ = _sdf->GetElement ( "amplitude" )->Get<double>();
    }

    if ( !_sdf->HasElement ( "frequency" ) ) {
        ROS_WARN ( "JointTilterController Plugin (ns = %s) missing <frequency>, defaults to %f",
                   this->robot_namespace_.c_str(), this->frequency_ );
    } else {
        this->frequency_ = _sdf->GetElement ( "frequency" )->Get<double>();
    }

    if ( !_sdf->HasElement ( "phase" ) ) {
        ROS_WARN ( "JointTilterController Plugin (ns = %s) missing <phase>, defaults to %f",
                   this->robot_namespace_.c_str(), this->phase_ );
    } else {
        this->phase_ = _sdf->GetElement ( "phase" )->Get<double>();
    }
}

void JointTilterController::OnUpdate ( const common::UpdateInfo & _info ) {
    // Apply a small linear velocity to the model.
    common::Time current_time = this->world_->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        double sine = create_sine( current_time.Double() );

        joint_control(sine);

        last_update_time_+= common::Time ( update_period_ );

    }

}

double JointTilterController::create_sine(double time) {
  double angular_frequency = 2.0*M_PI*frequency_;
  double p = phase_ + angular_frequency*time;
  double sin_p = sin(p);
  double q = offset_ + amplitude_*sin_p;

  return q;
}

void JointTilterController::joint_control(double sine) {
    math::Angle angle;
    angle.SetFromRadian(sine);
    joints_->SetAngle(0, angle);
}
