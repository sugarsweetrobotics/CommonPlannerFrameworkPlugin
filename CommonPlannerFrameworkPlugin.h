#pragma once

/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/IdPair>
#include <cnoid/Plugin>

#include <cnoid/CollisionDetector>

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

struct CnoidJointInfo {
public:
  CnoidJointInfo() {}
  CnoidJointInfo(const CnoidJointInfo& info) {copyFrom(info);}
  void operator=(const CnoidJointInfo& info) {copyFrom(info);}
private:
  void copyFrom(const CnoidJointInfo& info) {}

public:
  std::string name;
  double jointAngle;
  double jointDistance;
  double linkLength;
  double linkTwist;
  double maxAngle;
  double minAngle;
};


struct CnoidModelInfo {
public:
  std::vector<CnoidJointInfo> joints;
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

  void collisionCallback(const cnoid::CollisionPair& pair);
  
  Return_t isCollide(const std::string& name, const std::vector<double>& jointSeq, bool& out, std::vector<std::string>& collisionTargets);

  Return_t getModelInfo(const std::string& name, CnoidModelInfo& modelInfo);
};


