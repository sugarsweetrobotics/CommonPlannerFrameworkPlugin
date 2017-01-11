// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */


#include <iostream>
#include "TrajectoryPlannerSVC_impl.h"

#include "CommonPlannerFrameworkPlugin.h"
#include "PlannerRTC_Cnoid.h"

/*
 * Example implementational code for IDL interface Manipulation::ObjectDetectionService
 */
Manipulation_ObjectDetectionServiceSVC_impl::Manipulation_ObjectDetectionServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ObjectDetectionServiceSVC_impl::~Manipulation_ObjectDetectionServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_ObjectDetectionServiceSVC_impl::detectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo)
{
	Manipulation::ReturnValue* result;
  return result;
}

Manipulation::ReturnValue* Manipulation_ObjectDetectionServiceSVC_impl::setBaseFrame(const Manipulation::Matrix34& frame)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ObjectHandleStrategyService
 */
Manipulation_ObjectHandleStrategyServiceSVC_impl::Manipulation_ObjectHandleStrategyServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ObjectHandleStrategyServiceSVC_impl::~Manipulation_ObjectHandleStrategyServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_ObjectHandleStrategyServiceSVC_impl::getApproachOrientation(const Manipulation::ObjectInfo& objInfo, Manipulation::EndEffectorPose& eePos)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::KinematicSolverService

Manipulation_KinematicSolverServiceSVC_impl::Manipulation_KinematicSolverServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_KinematicSolverServiceSVC_impl::~Manipulation_KinematicSolverServiceSVC_impl()
{
  // Please add extra destructor code here.
}



 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_KinematicSolverServiceSVC_impl::solveKinematics(const Manipulation::EndEffectorPose& targetPose, const Manipulation::JointAngleSeq& startJointAngles,  Manipulation::JointAngleSeq_out targetJointAngles)
{
  Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
  return result;
}
*/


// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::CollisionDetectionService
 */
Manipulation_CollisionDetectionServiceSVC_impl::Manipulation_CollisionDetectionServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_CollisionDetectionServiceSVC_impl::~Manipulation_CollisionDetectionServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_CollisionDetectionServiceSVC_impl::isCollide(const Manipulation::RobotIdentifier& robotID, const Manipulation::JointAngleSeq& jointAngles, Manipulation::CollisionPairSeq_out collisions)
{
  Manipulation::ReturnValue_var result(new Manipulation::ReturnValue());
  result->id = Manipulation::OK;
  result->message = CORBA::string_dup("OK");
  
  // Please insert your code here and remove the following warning pragma
  std::cout << "isCollide called." << std::endl;
  
  std::string name = (const char*)robotID.name;
  
  std::vector<double> joints;
  for(size_t i = 0;i < jointAngles.length();++i) {
    joints.push_back(jointAngles[i].data);
  }

  bool flag = false;
  std::vector<std::string> collisionObjectNames;
  Return_t retval = m_pRTC->getPlugin()->isCollide(name, joints, flag, collisionObjectNames);

  Manipulation::CollisionPairSeq_var out(new Manipulation::CollisionPairSeq());
  if (retval.returnValue == RETVAL_OK) {
    if (collisionObjectNames.size() > 0) {
      out->length(collisionObjectNames.size());
      for(size_t i = 0;i < collisionObjectNames.size();++i) {
	out[i].name0 = CORBA::string_dup(name.c_str());
	out[i].name1 = CORBA::string_dup(collisionObjectNames[i].c_str());
      }
    }
  } else if (retval.returnValue == RETVAL_MODEL_NOT_FOUND) {
    result->id = Manipulation::MODEL_NOT_FOUND;
    std::stringstream ss;
    ss << "Model named '" << name << "' is not found in virtual space.";
    result->message = CORBA::string_dup(ss.str().c_str());
  } else if (retval.returnValue == RETVAL_INVALID_PRECONDITION) {
    result->id = Manipulation::INVALID_SETTING;
    std::stringstream ss;
    ss << "Invalid Setting of Choreonoid. For more information, Launch Choreonoid terminal and watch the output message.";
    result->message = CORBA::string_dup(ss.str().c_str());
  } else if (retval.returnValue == RETVAL_INVALID_JOINT_NUM) {
    result->id = Manipulation::INVALID_ARGUMENT;
    std::stringstream ss;
    ss << "Invalid Argument of isCollide method. For more information, Launch Choreonoid terminal and watch the output message.";
    result->message = CORBA::string_dup(ss.str().c_str());
  }
  collisions = out._retn();
  return result._retn();
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService
 */
Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan)>"
#endif
  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ModelServerService
 */
Manipulation_ModelServerServiceSVC_impl::Manipulation_ModelServerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ModelServerServiceSVC_impl::~Manipulation_ModelServerServiceSVC_impl()
{
  // Please add extra destructor code here.
}


static void operator<<=(Manipulation::JointParameter& jointInfo, const CnoidJointInfo& info) {
  jointInfo.name = CORBA::string_dup(info.name.c_str());
  switch(info.jointType) {
  JOINT_ROTATE:
    jointInfo.jointType = Manipulation::JOINT_ROTATE;
    break;
  JOINT_SLIDE:
    jointInfo.jointType = Manipulation::JOINT_SLIDE;
    break;
  JOINT_FIXED:
    jointInfo.jointType = Manipulation::JOINT_FIXED;
    break;
  JOINT_FREE:
    jointInfo.jointType = Manipulation::JOINT_FREE;
    break;
  default:
    std::cout << "Invalid JointType (" << info.jointType << ")" << std::endl;
    jointInfo.jointType = Manipulation::JOINT_UNKNOWN;
  }

  for(int i = 0;i < 3;i++) {
    jointInfo.axis[i] = info.axis[i];
    for(int j = 0;j < 3;j++) {
      jointInfo.offset[i][j] = info.rotation[i][j];
    }
    jointInfo.offset[i][3] = info.translation[i];
  }
  jointInfo.limit.upper = info.maxAngle;
  jointInfo.limit.lower = info.minAngle;
};

/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_ModelServerServiceSVC_impl::getModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out jointsInfo)
{
  Manipulation::ReturnValue_var result(new Manipulation::ReturnValue());
  std::string name = (const char*)robotID.name;
  CnoidModelInfo modelInfo;
  Return_t retval = m_pRTC->getPlugin()->getModelInfo(name, modelInfo);

  Manipulation::RobotJointInfo_var var(new Manipulation::RobotJointInfo());
  int n = modelInfo.joints.size();
  var->jointParameterSeq.length(n);
  for(int i = 0;i < n;++i) {
    var->jointParameterSeq[i] <<= modelInfo.joints[i];
  }
  jointsInfo = var._retn();

  return result._retn();
}

Manipulation::ReturnValue* Manipulation_ModelServerServiceSVC_impl::getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_ModelServerServiceSVC_impl::getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh)>"
#endif
  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::MotionGeneratorService
 */
Manipulation_MotionGeneratorServiceSVC_impl::Manipulation_MotionGeneratorServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_MotionGeneratorServiceSVC_impl::~Manipulation_MotionGeneratorServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::followManipPlan(const Manipulation::ManipulationPlan& manipPlan)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::followManipPlan(const Manipulation::ManipulationPlan& manipPlan)>"
#endif
  return result;
}

Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles)>"
#endif
  return result;
}



// End of example implementational code



