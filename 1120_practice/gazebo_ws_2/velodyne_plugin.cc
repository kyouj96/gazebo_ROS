#ifndef _VELODYNE_PLUGIN_HH_ 
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
	class VelodynePlugin : public ModelPlugin
	{
		public: VelodynePlugin() {}

		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
			if(_model -> GetJointCount() == 0)
			{
				std:: cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}
			
			this-> model = _model;
			this-> joint = _model->GetJoints()[0];
			
			this-> pid = common::PID(0.1,0,0);
			this-> model->GetJointController() -> SetVelocityPID( 
						this->joint->GetScopedName(), this-> pid);

			this-> model->GetJointController() ->SetVelocityTarget( 
						this->joint->GetScopedName(), 10.0);

			double velocity = 0;
		
			if(_sdf->HasElement("velocity") )
				velocity = _sdf->Get<double>("velocity");

			this->model->GetJointController()->SetVelocityTarget(
							this->joint->GetScopedName(), velocity);

			this->node = transport::NodePtr(new transport::Node());

			#if GAZEBO_MAJOR_VERSION < 8
			this->node->Init(this->model->GetWorld()->GetName());
			#else
			this->node->Init(this->model->GetWorld()->Name());
			#endif

			std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

			this->sub = this->node->Subscribe(topicName, 
										&VelodynePlugin::OnMsg, this);

		}
		
		private: void OnMsg(ConstVector3dPtr &_msg)
		{
			this->SetVelocity(_msg->x());
		}

		public: void SetVelocity(const double &_val)
		{
			//set the joint's target velocity.
			this-> model ->GetJointController() ->SetVelocityTarget(
									this->joint->GetScopedName(),_val);
		}

		private: physics::ModelPtr model;
		private: physics::JointPtr joint;
		private: common::PID pid;

		private: transport::NodePtr node;
		private: transport::SubscriberPtr sub;
	};

	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
			
					
