// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.h
 * @brief Service implementation header of TrajectoryPlanner.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "TrajectoryPlannerSkel.h"

#ifndef TRAJECTRYPLANNERSVC_IMPL_H
#define TRAJECTRYPLANNERSVC_IMPL_H


class PlannerRTC_Cnoid;

/*!
 * @class ObjectDetectionServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ObjectDetectionService
 */
class Manipulation_ObjectDetectionServiceSVC_impl
 : public virtual POA_Manipulation::ObjectDetectionService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ObjectDetectionServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ObjectDetectionServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ObjectDetectionServiceSVC_impl();

   // attributes and operations
   void detectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo);
   void setGeometry(RTC::Geometry3D geometry);

};


/*!
 * @class CurrentStateServiceSVC_impl
 * Example class implementing IDL interface Manipulation::CurrentStateService
 */
class Manipulation_CurrentStateServiceSVC_impl
 : public virtual POA_Manipulation::CurrentStateService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~CurrentStateServiceSVC_impl();


 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_CurrentStateServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_CurrentStateServiceSVC_impl();

   // attributes and operations
   void getCurrentState(Manipulation::RobotJointInfo_out robotJoint);

};

/*!
 * @class CollisionDetectionServiceSVC_impl
 * Example class implementing IDL interface Manipulation::CollisionDetectionService
 */
class Manipulation_CollisionDetectionServiceSVC_impl
 : public virtual POA_Manipulation::CollisionDetectionService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~CollisionDetectionServiceSVC_impl();

  PlannerRTC_Cnoid* m_pRTC;

public:
  void setRTC(PlannerRTC_Cnoid* pRTC) { m_pRTC = pRTC; }
 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_CollisionDetectionServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_CollisionDetectionServiceSVC_impl();

   // attributes and operations
   ::CORBA::Boolean isCollide(const Manipulation::RobotIdentifier& manipInfo, const Manipulation::RobotJointInfo& jointSeq, Manipulation::CollisionInfo_out collision);

};

/*!
 * @class ManipulationPlannerServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ManipulationPlannerService
 */
class Manipulation_ManipulationPlannerServiceSVC_impl
 : public virtual POA_Manipulation::ManipulationPlannerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ManipulationPlannerServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ManipulationPlannerServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ManipulationPlannerServiceSVC_impl();

   // attributes and operations
   void planManipulation(const Manipulation::RobotIdentifier& robotID, const Manipulation::RobotJointInfo& startRobotJointInfo, const Manipulation::RobotJointInfo& goalRobotJointInfo, Manipulation::ManipulationPlan_out manipPlan);

};

/*!
 * @class ModelServerServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ModelServerService
 */
class Manipulation_ModelServerServiceSVC_impl
 : public virtual POA_Manipulation::ModelServerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ModelServerServiceSVC_impl();
  PlannerRTC_Cnoid* m_pRTC;

public:
  void setRTC(PlannerRTC_Cnoid* pRTC) { m_pRTC = pRTC; }

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ModelServerServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ModelServerServiceSVC_impl();

   // attributes and operations
   void getModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotInfo);
   void getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh);

};

/*!
 * @class MotionGeneratorServiceSVC_impl
 * Example class implementing IDL interface Manipulation::MotionGeneratorService
 */
class Manipulation_MotionGeneratorServiceSVC_impl
 : public virtual POA_Manipulation::MotionGeneratorService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~MotionGeneratorServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_MotionGeneratorServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_MotionGeneratorServiceSVC_impl();

   // attributes and operations
   void followManipPlan(const Manipulation::ManipulationPlan& manipPlan);
   void getCurrentRobotJointInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out robotJoint);

};



#endif // TRAJECTRYPLANNERSVC_IMPL_H


