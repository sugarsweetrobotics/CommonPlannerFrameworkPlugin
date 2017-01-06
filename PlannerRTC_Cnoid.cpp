// -*- C++ -*-
/*!
 * @file  PlannerRTC_Cnoid.cpp
 * @brief PlannerRTC Service on Choreonoid
 * @date $Date$
 *
 * $Id$
 */

#include "PlannerRTC_Cnoid.h"

// Module specification
// <rtc-template block="module_spec">
static const char* plannerrtc_cnoid_spec[] =
  {
    "implementation_id", "PlannerRTC_Cnoid",
    "type_name",         "PlannerRTC_Cnoid",
    "description",       "PlannerRTC Service on Choreonoid",
    "version",           "1.0.0",
    "vendor",            "Ogata Laboratory",
    "category",          "MotionPlannin",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",

    // Widget
    "conf.__widget__.debug", "text",
    // Constraints

    "conf.__type__.debug", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PlannerRTC_Cnoid::PlannerRTC_Cnoid(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_collisionDetectorPort("collisionDetector")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PlannerRTC_Cnoid::~PlannerRTC_Cnoid()
{
}



RTC::ReturnCode_t PlannerRTC_Cnoid::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_collisionDetectorPort.registerProvider("Manipulation_CollisionDetectionService", "Manipulation::CollisionDetectionService", m_collisionDetectionService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_collisionDetectorPort);

  m_collisionDetectionService.setRTC(this);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PlannerRTC_Cnoid::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void PlannerRTC_CnoidInit(RTC::Manager* manager)
  {
    coil::Properties profile(plannerrtc_cnoid_spec);
    manager->registerFactory(profile,
                             RTC::Create<PlannerRTC_Cnoid>,
                             RTC::Delete<PlannerRTC_Cnoid>);
  }
  
};


