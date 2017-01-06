/**
   @author Yuki Suga at SSR
*/

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>

using namespace boost;
using namespace cnoid;

class CommonPlannerFrameworkPlugin : public Plugin {
public:
  /**
   * Constructor
   */
  CommonPlannerFrameworkPlugin() : Plugin("CommonPlannerFramework") {
    require("Body");
  }

public:
  /**
   * Initialization Function called by System
   */
  virtual bool initialize() {
    ToolBar* bar = new ToolBar("CommonPlanner");
    bar->addButton("Increment")->sigClicked().connect(bind(&CommonPlannerFrameworkPlugin::onButtonClicked, this, +0.04));
    bar->addButton("Decrement")->sigClicked().connect(bind(&CommonPlannerFrameworkPlugin::onButtonClicked, this, -0.04));
    addToolBar(bar);
    
    return true;
  }

  void onButtonClicked(double dq) {
    ItemList<BodyItem> bodyItems =
      ItemTreeView::mainInstance()->selectedItems<BodyItem>();
    
    for(size_t i=0; i < bodyItems.size(); ++i){
      BodyPtr body = bodyItems[i]->body();
      for(int j=0; j < body->numJoints(); ++j){
	body->joint(j)->q() += dq;
      }
      bodyItems[i]->notifyKinematicStateChange(true);
    }
  }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(CommonPlannerFrameworkPlugin)
