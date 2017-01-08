// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */

#include "PlannerRTC_Cnoid.h"
#include "TrajectoryPlannerSVC_impl.h"
#include "CommonPlannerFrameworkPlugin.h"

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
void Manipulation_ObjectDetectionServiceSVC_impl::detectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo)
{
}

void Manipulation_ObjectDetectionServiceSVC_impl::setGeometry(RTC::Geometry3D geometry)
{
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::CurrentStateService
 */
Manipulation_CurrentStateServiceSVC_impl::Manipulation_CurrentStateServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_CurrentStateServiceSVC_impl::~Manipulation_CurrentStateServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void Manipulation_CurrentStateServiceSVC_impl::getCurrentState(Manipulation::RobotJointInfo_out robotJoint)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void Manipulation_CurrentStateServiceSVC_impl::getCurrentState(Manipulation::RobotJointInfo_out robotJoint)>"
#endif
}



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
::CORBA::Boolean Manipulation_CollisionDetectionServiceSVC_impl::isCollide(const Manipulation::RobotIdentifier& manipInfo, const Manipulation::RobotJointInfo& jointSeq, Manipulation::CollisionInfo_out collision)
{
  std::string name = (const char*)manipInfo.name;
  std::vector<double> joints;
  for(size_t i = 0;i < jointSeq.jointInfoSeq.length();++i) {
    joints[i] = jointSeq.jointInfoSeq[i].jointAngle;
  }
  bool flag = false;
  std::vector<std::string> collisionObjectNames;
  
  Return_t retval = m_pRTC->getPlugin()->isCollide(name, joints, flag, collisionObjectNames);

  Manipulation::CollisionInfo_var out(new Manipulation::CollisionInfo());
  if (collisionObjectNames.size() > 0) {
    out->name = CORBA::string_dup(collisionObjectNames[0].c_str());
  } else {
    out->name = CORBA::string_dup("none");
  }
  collision = out._retn();
  return flag;
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
void Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan)>"
#endif
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

static void operator<<=(Manipulation::JointInfo& jointInfo, const CnoidJointInfo& info) {
  jointInfo.name = CORBA::string_dup(info.name.c_str());
  jointInfo.jointAngle = info.jointAngle;
  jointInfo.jointDistance = info.jointDistance;
  jointInfo.linkLength = info.linkLength;
  jointInfo.linkTwist = info.linkTwist;
  jointInfo.maxAngle = info.maxAngle;
  jointInfo.minAngle = info.minAngle;
};
/*
 * Methods corresponding to IDL attributes and operations
 */
void Manipulation_ModelServerServiceSVC_impl::getModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotInfo)
{
  std::string name = (const char*)robotID.name;
  CnoidModelInfo modelInfo;
  Return_t retval = m_pRTC->getPlugin()->getModelInfo(name, modelInfo);


  Manipulation::RobotJointInfo_var var(new Manipulation::RobotJointInfo());
  int n = modelInfo.joints.size();
  var->jointInfoSeq.length(n);
  for(int i = 0;i < n;++i) {
    var->jointInfoSeq[i] <<= modelInfo.joints[i];
  }
  robotInfo = var._retn();
  return;
}

void Manipulation_ModelServerServiceSVC_impl::getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void Manipulation_ModelServerServiceSVC_impl::getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh)>"
#endif
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
void Manipulation_MotionGeneratorServiceSVC_impl::followManipPlan(const Manipulation::ManipulationPlan& manipPlan)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void Manipulation_MotionGeneratorServiceSVC_impl::followManipPlan(const Manipulation::ManipulationPlan& manipPlan)>"
#endif
}

void Manipulation_MotionGeneratorServiceSVC_impl::getCurrentRobotJointInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotJoint)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void Manipulation_MotionGeneratorServiceSVC_impl::getCurrentRobotJointInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotJoint)>"
#endif
}



// End of example implementational code



