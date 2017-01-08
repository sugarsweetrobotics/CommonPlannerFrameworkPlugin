/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
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
  bar->addButton("Test")->sigClicked().connect(boost::bind(&CommonPlannerFrameworkPlugin::onTest, this));
  addToolBar(bar);

  RTC::Manager& rtcManager = RTC::Manager::instance();
  PlannerRTC_CnoidInit(&rtcManager);
  const char* param = "PlannerRTC_Cnoid?instance_name=PlannerRTC_Cnoid&exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=30";
  RTObject_impl* rtc = rtcManager.createComponent(param);
  plannerRTC = dynamic_cast<PlannerRTC_Cnoid*>(rtc);
  plannerRTC->setPlugin(this);

  /**
  int index = cnoid::CollisionDetector::factoryIndex("AISTCollisionDetector");
  std::cout << "Index = " << index << " / ";
  cnoid::ItemList<cnoid::WorldItem> worldItems = cnoid::ItemTreeView::mainInstance()->selectedItems<cnoid::WorldItem>();
  std::cout << "size = " << worldItems.size() << " / ";
  //  pDetector = cnoid::CollisionDetector::create(index);
  std::cout << "name = " << pDetector->name() << std::endl;
  */
  return true;
}


void CommonPlannerFrameworkPlugin::onTest() {
  std::cout << "Test" << std::endl;
  std::vector<double> joints;
  joints.push_back(0);
  joints.push_back(1);
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  std::vector<std::string> names;
  bool flag;
  Return_t retval = this->isCollide("orochi", joints, flag, names);
  std::cout << "flag: " << flag << std::endl;
  std::cout << "retval : " << retval.message << std::endl;
  for(size_t i = 0;i < names.size();++i) {
    std::cout << "Collide with " << names[i] << std::endl;
  }
  //std::cout << "isCollide() = " << this->isCollide("orochi") << std::endl;
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

void CommonPlannerFrameworkPlugin::collisionCallback(const cnoid::CollisionPair& pair) {}


void CommonPlannerFrameworkPlugin::onKinematicStateChanged(const std::string& name) {
  std::cout << "onKinematcStateChanged: " << name << std::endl;
  namedCounter[name] = namedCounter[name] + 1;
}

Return_t CommonPlannerFrameworkPlugin::isCollide(const std::string& name, const std::vector<double>& jointSeq, bool& out, std::vector<std::string>& collisionTargets) {

  Return_t retval;
  
  cnoid::BodyItemPtr targetBodyItem;
  cnoid::ItemList<cnoid::BodyItem> bodyItems = cnoid::ItemTreeView::instance()->checkedItems<cnoid::BodyItem>();

  
  for(size_t i = 0;i < bodyItems.size(); ++i) {
    cnoid::BodyPtr body = bodyItems[i]->body();
    if (body->name() == name) {
      targetBodyItem = bodyItems[i];
    }
  }

  bool flag = false;
  if (!targetBodyItem) {
    retval.returnValue = RETVAL_MODEL_NOT_FOUND;
    retval.message = "Model is not found.";
    out = false;
    return retval;
  }

  
  cnoid::WorldItemPtr world = targetBodyItem->findOwnerItem<cnoid::WorldItem>();
  if (!world) {
    retval.returnValue = RETVAL_INVALID_PRECONDITION;
    retval.message = "Choreonoid needs WorldItem to detect Collision";
    out = false;
    return retval;
  }

  std::map<std::string, int32_t>::iterator itr = namedCounter.find(name);
  if (itr == namedCounter.end()) {
    namedCounter[name] = 0;
    world->sigCollisionsUpdated()
    //targetBodyItem->sigKinematicStateChanged()
      .connect(boost::bind(&CommonPlannerFrameworkPlugin::onKinematicStateChanged, this, name));
  }

  if (targetBodyItem->body()->numJoints() != jointSeq.size()) {
    retval.returnValue = RETVAL_INVALID_JOINT_NUM;
    retval.message = "Invalid Number of Joints";
    out = false;
    return retval;
  }
  cnoid::BodyPtr body = targetBodyItem->body();
  for(int j=0; j < body->numJoints(); ++j){
    body->joint(j)->q() = jointSeq[j];
  }
  namedCounter[name] = 0;
  targetBodyItem->notifyKinematicStateChange(true);

  while(true) {
    if (namedCounter[name] > 0) {
      break;
    }
    ; // do nothing
  }
  
  // world->notifyUpdate();
  //world->collisionDetector()->detectCollisions(boost::bind(&CommonPlannerFrameworkPlugin::collisionCallback, this, _1));
  // world->updateCollisions();
  std::cout << "udpateCollisions()" << std::endl;
  std::vector<cnoid::CollisionLinkPairPtr> pairs = world->collisions();
  for(size_t i = 0;i < pairs.size(); ++i) {
    std::string targetName;
    if (pairs[i]->body[0]->name() == name) {
      std::string targetName = pairs[i]->body[1]->name();
      flag = true;
      collisionTargets.push_back(targetName);

    } else if (pairs[i]->body[1]->name() == name) {
      std::string targetName = pairs[i]->body[0]->name();
      flag = true;
      collisionTargets.push_back(targetName);
    }
    // std::cout << "Collision(" << pairs[i]->body[0]->name() << "/" << pairs[i]->body[1]->name() << ")" << std::endl;
  }
  out = flag;
  return retval;
}

Return_t CommonPlannerFrameworkPlugin::getModelInfo(const std::string& name, CnoidModelInfo& modelInfo) {
  Return_t retval;
  
  cnoid::BodyItemPtr targetBodyItem;
  
  cnoid::ItemList<cnoid::BodyItem> bodyItems = cnoid::ItemTreeView::instance()->checkedItems<cnoid::BodyItem>();
  for(size_t i = 0;i < bodyItems.size(); ++i) {
    cnoid::BodyPtr body = bodyItems[i]->body();
    if (body->name() == name) {
      targetBodyItem = bodyItems[i];
    }
  }

  if (!targetBodyItem) {
    retval.returnValue = RETVAL_MODEL_NOT_FOUND;
    retval.message = "Model is not found.";
    return retval;
  }

  cnoid::BodyPtr body = targetBodyItem->body();
  for(size_t i = 0;i < body->numAllJoints();++i) {
    cnoid::LinkPtr link = body->joint(i);
    CnoidJointInfo jointInfo;
    jointInfo.name = link->name();
    jointInfo.jointAngle = link->q();
    jointInfo.maxAngle = link->q_upper();
    jointInfo.minAngle = link->q_lower();
    modelInfo.joints.push_back(jointInfo);
  }
}

//////
CNOID_IMPLEMENT_PLUGIN_ENTRY(CommonPlannerFrameworkPlugin)
