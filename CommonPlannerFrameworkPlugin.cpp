/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/ItemTreeView>
#include <cnoid/LazyCaller>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ToolBar>
#include <cnoid/OpenRTMUtil>
#include <cnoid/MessageView>
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
  bar->addButton("Test")->sigClicked().connect(boost::bind(&CommonPlannerFrameworkPlugin::onTest, this));
  addToolBar(bar);

  RTC::Manager& rtcManager = RTC::Manager::instance();
  PlannerRTC_CnoidInit(&rtcManager);
  const char* param = "PlannerRTC_Cnoid?instance_name=PlannerRTC_Cnoid&exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=30";
  RTObject_impl* rtc = rtcManager.createComponent(param);
  plannerRTC = dynamic_cast<PlannerRTC_Cnoid*>(rtc);
  plannerRTC->setPlugin(this);

  return true;
}


void CommonPlannerFrameworkPlugin::onTest() {

  /*
  CnoidModelInfo info;
  this->getModelInfo("orochi", info);
  */

  std::vector<double> jointSeq(7, 0);
  jointSeq[1] = 1.0;
  bool flag;
  std::vector<std::string> collisionTargets;
  Return_t retval = this->isCollide("orochi", jointSeq, flag, collisionTargets);
  std::cout << "Return: " << retval.message << std::endl;
  if (flag) {
    
    std::cout << "Collide with " << collisionTargets[0] << std::endl;
  } else {
    std::cout << "No collision." << std::endl;
  }
    
}


void CommonPlannerFrameworkPlugin::onKinematicStateChanged(const std::string& name) {
  std::cout << "onKinematcStateChanged: " << name << std::endl;
  namedCounter[name] = namedCounter[name] + 1;
}

Return_t CommonPlannerFrameworkPlugin::isCollideSynchronously(const std::string& name, const std::vector<double>& jointSeq, bool& out, std::vector<std::string>& collisionTargets) {
  //  cnoid::callSynchronously(boost::bind(&CommonPlannerFrameworkPlugin::isCollide, this, name, jointSeq, out, collisionTargets));
  return this->isCollide(name, jointSeq, out, collisionTargets);
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

  if (targetBodyItem->body()->numJoints() != jointSeq.size()) {
    retval.returnValue = RETVAL_INVALID_JOINT_NUM;
    retval.message = "Invalid Number of Joints";
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

  
  cnoid::BodyPtr body = targetBodyItem->body();
  for(int j=0; j < body->numJoints(); ++j){
    body->joint(j)->q() = jointSeq[j];
  }
  namedCounter[name] = 0;
  targetBodyItem->notifyKinematicStateChange(true);
  std::cout << "udpateCollisions()" << std::endl;

  cnoid::MessageView::instance()->flush();
  /*
  while(true) {
    if (namedCounter[name] > 0) {
      break;
    }
    ; // do nothing
  }
  */

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

Return_t CommonPlannerFrameworkPlugin::getModelInfoSynchronously(const std::string& name, CnoidModelInfo& modelInfo) {
  //cnoid::callSynchronously(boost::bind(&CommonPlannerFrameworkPlugin::getModelInfo, this, name, modelInfo));
  return this->getModelInfo(name, modelInfo);
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
    //std::cout << "-J(" << i << ") = " << link->name() << std::endl;
    CnoidJointInfo jointInfo;
    jointInfo.name = link->name();
    
    if (link->isRotationalJoint()) {
      jointInfo.jointType = JOINT_ROTATE;
    } else if (link->isSlideJoint()) {
      jointInfo.jointType = JOINT_SLIDE;
    } else if (link->isFixedJoint()) {
      jointInfo.jointType = JOINT_FIXED;
    } else if (link->isFreeJoint()) {
      jointInfo.jointType = JOINT_FREE;
    } else {
      std::cout << "Invalid Joint Type (" << link->name() << ")" << std::endl;
      retval.returnValue = RETVAL_INVALID_PRECONDITION;
      retval.message = "Invalid Joint Type";
      return retval;
    }
    
    for(int i = 0;i < 3;i++) {
      jointInfo.axis[i] = link->a()[i];
      jointInfo.translation[i] = link->b()[i];
    }

    for(int i = 0;i < 3;i++) {
      for(int j = 0;j < 3;j++) {
	jointInfo.rotation[i][j] = link->Rb()(i, j);
      }
    }


    jointInfo.maxAngle = link->q_upper();
    jointInfo.minAngle = link->q_lower();
    
    modelInfo.joints.push_back(jointInfo);
  }
}

//////
CNOID_IMPLEMENT_PLUGIN_ENTRY(CommonPlannerFrameworkPlugin)
