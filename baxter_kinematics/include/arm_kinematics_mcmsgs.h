#ifndef ARM_KINEMATICS_H_
#define ARM_KINEMATICS_H_
#include <cstring>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <motor_control_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/Bool.h>
//#include <PositionKinematicsNode.h>
#include <motor_control_msgs/JointPosition.h>
#include <motor_control_msgs/FKreply.h>
//#include <motor_control_msgs/GetPositionIK.h>
//#include <kinematics_msgs/GetKinematicSolverInfo.h>
//#include <kinematics_msgs/KinematicSolverInfo.h>

namespace arm_kinematics {
/*typedef struct INFO{
	std::vector<std::string> link_names;
	std::vector<std::string> joint_names;
	}KinematicSolverInfo;
typedef struct fkr{
std::vector<geometry_msgs::PoseStamped> pose;
std::vector<std_msgs::Bool> isValid;
}FKReply;*/
class Kinematics {
    public:
        Kinematics();
        bool init();
	typedef boost::shared_ptr<Kinematics> Ptr;
	static Ptr create()
  	{
    		Ptr parm_kinematics = Ptr(new Kinematics());
    		if(parm_kinematics->init())
    		{
      			return parm_kinematics;
    		}
    		return Ptr();
 	 }
    
        bool getPositionIK(const geometry_msgs::PoseStamped &pose_stamp,
                              const motor_control_msgs::JointPosition &seed,
                              motor_control_msgs::JointPosition *result);
   //kinematics::FKReply result;
        bool getPositionFK(std::string frame_id,const motor_control_msgs::JointPosition &joint_configuration, motor_control_msgs::FKreply &result);
	motor_control_msgs::KinematicSolverInfo info;

    private:
        ros::NodeHandle nh, nh_private;
        std::string root_name, tip_name;
        KDL::JntArray joint_min, joint_max;
        KDL::Chain chain;
        unsigned int num_joints;

        KDL::ChainFkSolverPos_recursive* fk_solver;
        KDL::ChainIkSolverPos_NR_JL *ik_solver_pos;
        KDL::ChainIkSolverVel_pinv* ik_solver_vel;

        ros::ServiceServer ik_service,ik_solver_info_service;
        ros::ServiceServer fk_service,fk_solver_info_service;

        tf::TransformListener tf_listener;

        //motor_control_msgs::KinematicSolverInfo info;
	
        bool loadModel(const std::string xml);
        bool readJoints(urdf::Model &robot_model);
        int getJointIndex(const std::string &name);
        int getKDLSegmentIndex(const std::string &name);

        /**
         * @brief This is the basic IK service method that will compute and return an IK solution.
         * @param PoseStamped message that includes the desired pose for which IK has to be calculated
         * @param JointPosition that includes desired Joint angles/names for the Pose.
         */
        
        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        //bool getIKSolverInfo();

        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        //bool getFKSolverInfo();

        /**
         * @brief This is the basic forward kinematics service that will return information about the kinematics node.
         * @param A request message. See service definition for GetPositionFK for more information on this message.
         * @param The response message. See service definition for GetPositionFK for more information on this message.
         */
        //bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
      //                     kinematics_msgs::GetPositionFK::Response &response);
	
};

}
#endif
