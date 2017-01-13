#pragma once

/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/IdPair>
#include <cnoid/Plugin>

#include <cnoid/CollisionDetector>

#include <map>
/// Reference class 
class PlannerRTC_Cnoid;

enum RETVAL {
  RETVAL_OK,
  RETVAL_MODEL_NOT_FOUND,
  RETVAL_INVALID_PRECONDITION,
  RETVAL_INVALID_JOINT_NUM,
};

struct Return_t {
Return_t() : returnValue(RETVAL_OK), message("OK") {};
Return_t(RETVAL returnValue_, const std::string& message_): returnValue(returnValue_), message(message_) {};
  
  RETVAL returnValue;
  std::string message;
};

enum JOINT_TYPE {
  JOINT_ROTATE,
  JOINT_SLIDE,
  JOINT_FREE,
  JOINT_FIXED,
};

struct CnoidJointInfo {
public:
  CnoidJointInfo() {}
  CnoidJointInfo(const CnoidJointInfo& info) {copyFrom(info);}
  void operator=(const CnoidJointInfo& info) {copyFrom(info);}
private:
  void copyFrom(const CnoidJointInfo& info) {
    this->name = info.name;
    this->jointType = info.jointType;
    memcpy(this->translation, info.translation, 3 * sizeof(double));
    memcpy(this->axis, info.axis, 3 * sizeof(double));
    for(int i = 0;i < 3;i++) {
      memcpy(this->rotation[i], info.rotation[i], 3 * sizeof(double));
    }
    this->maxAngle = info.maxAngle;
    this->minAngle = info.minAngle;
  }

public:
  std::string name;
  JOINT_TYPE jointType;
  double axis[3];
  double translation[3];
  double rotation[3][3];
  double maxAngle;
  double minAngle;
};


struct CnoidModelInfo {
public:
  std::vector<CnoidJointInfo> joints;


public:
  void operator=(const CnoidModelInfo& mi) {
    this->joints.clear();
    this->joints.insert(this->joints.end(), mi.joints.begin(), mi.joints.end());
  }
};


/**
 * Plugin Class
 */
class CommonPlannerFrameworkPlugin : public cnoid::Plugin {
private:
  PlannerRTC_Cnoid* plannerRTC;

public:

  
  /**
   * Constructor
   */
  CommonPlannerFrameworkPlugin();

public:
  /**
   * Initialization Function called by System
   */
  virtual bool initialize();

  void onButtonClicked(double dq);

  void onTest();

  std::map<std::string, int32_t> namedCounter;
  void onKinematicStateChanged(const std::string& name);

  std::string __name;
  std::vector<double> __jointSeq;
  bool __out;
  std::vector<std::string> __collisionTargets;
  Return_t __retval;
  void __isCollide() {
    __retval = isCollide(__name, __jointSeq, __out, __collisionTargets);
  }
  Return_t isCollide(const std::string& name, const std::vector<double>& jointSeq, bool& out, std::vector<std::string>& collisionTargets);
  Return_t isCollideSynchronously(const std::string& name, const std::vector<double>& jointSeq, bool& out, std::vector<std::string>& collisionTargets);
  
  std::string __modelName;
  CnoidModelInfo __modelInfo;
  Return_t __modelRetval;
  void __getModelInfo() {
    __modelRetval = getModelInfo(__modelName, __modelInfo);
  }
  
  Return_t getModelInfo(const std::string& name, CnoidModelInfo& modelInfo);
  Return_t getModelInfoSynchronously(const std::string& name, CnoidModelInfo& modelInfo);
};


