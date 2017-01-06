#pragma once

/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/Plugin>

/// Reference class 
class PlannerRTC_Cnoid;

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


  bool isCollide();
};


