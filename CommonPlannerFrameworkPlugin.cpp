/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/OpenRTMUtil>

#include <boost/bind.hpp>

#include "CommonPlannerFrameworkPlugin.h"

#include "PlannerRTC_Cnoid.h"

/**
 * Constructor
 */
CommonPlannerFrameworkPlugin::CommonPlannerFrameworkPlugin() : cnoid::Plugin("CommonPlannerFramework") {
  require("Body");
  require("OpenRTM");
}


/**
 */
bool CommonPlannerFrameworkPlugin::initialize() {
  cnoid::ToolBar* bar = new cnoid::ToolBar("CommonPlanner");
  bar->addButton("Increment")->sigClicked().connect(boost::bind(&CommonPlannerFrameworkPlugin::onButtonClicked, this, +0.04));
  bar->addButton("Decrement")->sigClicked().connect(boost::bind(&CommonPlannerFrameworkPlugin::onButtonClicked, this, -0.04));
  addToolBar(bar);

  RTC::Manager& rtcManager = RTC::Manager::instance();
  PlannerRTC_CnoidInit(&rtcManager);
  const char* param = "PlannerRTC_Cnoid?instance_name=PlannerRTC_Cnoid&exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=30";
  RTObject_impl* rtc = rtcManager.createComponent(param);
  plannerRTC = dynamic_cast<PlannerRTC_Cnoid*>(rtc);
  plannerRTC->setPlugin(this);
  return true;
}

/**
 */
void CommonPlannerFrameworkPlugin::onButtonClicked(double dq) {
  cnoid::ItemList<cnoid::BodyItem> bodyItems = cnoid::ItemTreeView::mainInstance()->selectedItems<cnoid::BodyItem>();
  
  for(size_t i=0; i < bodyItems.size(); ++i){
    cnoid::BodyPtr body = bodyItems[i]->body();
    for(int j=0; j < body->numJoints(); ++j){
      body->joint(j)->q() += dq;
    }
    bodyItems[i]->notifyKinematicStateChange(true);
  }
}


//////
CNOID_IMPLEMENT_PLUGIN_ENTRY(CommonPlannerFrameworkPlugin)
