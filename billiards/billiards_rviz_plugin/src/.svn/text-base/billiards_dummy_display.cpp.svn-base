/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include "billiards_dummy_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"
#include "rviz/tools/tool.h"
#include "rviz/window_manager_interface.h"
#include "rviz/tools/move_tool.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"

#include <ogre_tools/shape.h>
#include <ogre_tools/arrow.h>
#include <ogre_tools/movable_text.h>

#include <wx/wx.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreManualObject.h>

#include <billiards_msgs/Constants.h>
#include <billiards_msgs/TableState.h>
#include <billiards_msgs/ShotPlan.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <billiards_msgs/GetTableStateAction.h>
#include <billiards_msgs/PlanShotAction.h>
#include <billiards_msgs/PlayAction.h>
#include <billiards_msgs/LocalizeTableAction.h>


namespace billiards_rviz_plugin
{

BilliardsTool* BilliardsDummyDisplay::s_tool_ = 0;

using billiards_msgs::Constants;

class Manip2D
{
public:
  Manip2D(rviz::VisualizationManager* manager, Ogre::SceneNode* parent, BilliardsTool* tool, float scale);

  bool collMatches(rviz::CollObjectHandle handle);
  void handle(rviz::CollObjectHandle handle, Ogre::Viewport* vp,  uint32_t x, uint32_t y);
  void align(const Ogre::Vector3& lookat, const Ogre::Vector3& pos);
  void set(const billiards_msgs::ShotPlan& plan);

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  Ogre::SceneNode* getSceneNode() { return bridge_node_; }

private:
  ogre_tools::Shape* rot_handle_;
  ogre_tools::Shape* bridge_;
  rviz::CollObjectHandle rot_coll_;
  rviz::CollObjectHandle bridge_coll_;

  rviz::VisualizationManager* manager_;

  Ogre::SceneNode* bridge_node_;
  BilliardsTool* tool_;
};

class BilliardsTool : public rviz::Tool, public wxEvtHandler
{
public:
  BilliardsTool(rviz::VisualizationManager* manager);
  ~BilliardsTool();

  virtual void activate();
  virtual void deactivate();

  virtual void update(float wall_dt, float ros_dt);

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  //virtual int processKeyEvent( wxKeyEvent& event ) { return 0; }

  //virtual bool hasProperties() { return false; }
  //virtual void enumerateProperties(PropertyManager* property_manager, const CategoryPropertyWPtr& parent) {}

private:
  struct Ball
  {
    uint32_t id;
    ogre_tools::Shape* shape;
    ogre_tools::MovableText* id_text;
    Ogre::SceneNode* text_node;
    rviz::CollObjectHandle coll;
  };

  rviz::SelectionHandlerPtr sel_handler_;

  Ball* pickBall(uint32_t x, uint32_t y, Ogre::Viewport* vp);
  Ogre::Vector3 getBallPosInTableFrame(Ball* b);

  void publishTable();
  billiards_msgs::TableState getTableState();
  bool isOnTable(Ball* b);

  void publishShotPlan();
  billiards_msgs::ShotPlan getShotPlan();

  void updateTableFrame();
  geometry_msgs::PoseStamped getLocalizedPose();

  void getTableStateCallback(actionlib::ActionServer<billiards_msgs::GetTableStateAction>::GoalHandle goal);
  void localizeTableCallback(actionlib::ActionServer<billiards_msgs::LocalizeTableAction>::GoalHandle goal);
  void planShotCallback(const billiards_msgs::PlanShotGoalConstPtr& goal);

  void onPlayDone(const actionlib::SimpleClientGoalState& state, const billiards_msgs::PlayResultConstPtr& result);
  void onVisionGetTableStateDone(const actionlib::SimpleClientGoalState& state, const billiards_msgs::GetTableStateResultConstPtr& result);
  void onPlanShotDone(const actionlib::SimpleClientGoalState& state, const billiards_msgs::PlanShotResultConstPtr& result);

  void onGetTableStateSucceed(wxCommandEvent& evt);
  void onGetTableStateAborted(wxCommandEvent& evt);

  void onLocalizeTableSucceed(wxCommandEvent& evt);
  void onLocalizeTableAborted(wxCommandEvent& evt);

  Ogre::SceneNode* scene_node_;
  rviz::MoveTool* move_tool_;

  std::vector<ogre_tools::Shape*> table_;
  typedef std::vector<Ball*> V_Ball;
  V_Ball balls_;

  bool left_is_down_;
  bool right_is_down_;

  Ball* selected_ball_;

  Ogre::Vector3 table_pos_;
  Ogre::Quaternion table_orient_;

  ros::Publisher table_state_pub_;
  ros::Publisher shot_plan_pub_;
  ros::Publisher localized_table_pub_;

  actionlib::ActionServer<billiards_msgs::GetTableStateAction> get_table_state_as_;
  actionlib::ActionServer<billiards_msgs::GetTableStateAction>::GoalHandle get_table_current_goal_;
  actionlib::ActionServer<billiards_msgs::LocalizeTableAction> localize_table_as_;
  actionlib::ActionServer<billiards_msgs::LocalizeTableAction>::GoalHandle localize_table_current_goal_;

  actionlib::SimpleActionServer<billiards_msgs::PlanShotAction> plan_shot_as_;

  actionlib::SimpleActionClient<billiards_msgs::PlayAction> play_fixed_ac_;
  actionlib::SimpleActionClient<billiards_msgs::GetTableStateAction> vision_get_table_state_ac_;
  actionlib::SimpleActionClient<billiards_msgs::PlanShotAction> plan_shot_ac_;
  actionlib::SimpleActionClient<billiards_msgs::LocalizeTableAction> localize_table_ac_;
  bool waiting_for_play_fixed_;

  Manip2D* bridge_manip_;
  Manip2D* base_manip_;
  Manip2D* selected_bridge_manip_;
  Ogre::ManualObject* radius_object_;

  Manip2D* localize_manip_;

  rviz::CollObjectHandle selected_coll_;

  void onPlay(wxCommandEvent& evt);
  void onVisionGetTableState(wxCommandEvent& evt);
  void onPlanShot(wxCommandEvent& evt);
  void onLocalizeTable(wxCommandEvent& evt);
  wxPanel* panel_;

  wxFrame* get_table_state_frame_;
  wxFrame* localize_table_frame_;

  friend class Manip2D;
};

#define HALF_BRIDGE_HEIGHT (Constants::BRIDGE_HEIGHT * 0.5)

Manip2D::Manip2D(rviz::VisualizationManager* manager, Ogre::SceneNode* parent, BilliardsTool* tool, float scale)
: manager_(manager)
, tool_(tool)
{
  float length = Constants::BRIDGE_TO_STRIKE_MIN * scale;
  float rad = 0.01 * scale;
  bridge_node_ = parent->createChildSceneNode();
  rot_handle_ = new ogre_tools::Shape(ogre_tools::Shape::Cylinder, manager->getSceneManager(), bridge_node_);
  bridge_ = new ogre_tools::Shape(ogre_tools::Shape::Cube, manager->getSceneManager(), bridge_node_);
  bridge_->setScale(Ogre::Vector3(Constants::BRIDGE_THICKNESS * scale, Constants::BRIDGE_WIDTH * scale, Constants::BRIDGE_HEIGHT * scale));
  rot_handle_->setScale(Ogre::Vector3(rad*2, length, rad*2));

  bridge_->setPosition(Ogre::Vector3(0.0f, 0.0f, HALF_BRIDGE_HEIGHT * scale));

  Ogre::Quaternion q(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  rviz::ogreToRobot(q);
  rot_handle_->setOrientation(q);

  Ogre::Vector3 p(0.0f, rad, -length/2.0f);
  rviz::ogreToRobot(p);
  rot_handle_->setPosition(p);
  //rot_handle_->setOffset(Ogre::Vector3(0.0f, rad, length/2.0f));
  //rot_handle_->setOrientation(Ogre::Quaternion(Ogre::Degree(45), Ogre::Vector3::UNIT_Z));

  rot_handle_->setColor(Ogre::ColourValue(0.3, 0.3, 0.6));
  bridge_->setColor(Ogre::ColourValue(0.8, 0.3, 0.3));


  rviz::SelectionManager* sel_man = manager->getSelectionManager();

  rviz::SelectionHandlerPtr sel_handler(new rviz::SelectionHandler);
  rot_coll_ = sel_man->createCollisionForObject(rot_handle_, sel_handler);
  bridge_coll_ = sel_man->createCollisionForObject(bridge_, sel_handler);

}

bool Manip2D::collMatches(rviz::CollObjectHandle handle)
{
  return rot_coll_ == handle || bridge_coll_ == handle;
}

Ogre::Vector3 Manip2D::getPosition()
{
  return bridge_node_->getPosition();
}

Ogre::Quaternion Manip2D::getOrientation()
{
  return bridge_node_->getOrientation();
}

void Manip2D::set(const billiards_msgs::ShotPlan& plan)
{
  bridge_node_->setPosition(Ogre::Vector3(plan.bridge_pose.pose.position.x, plan.bridge_pose.pose.position.y, plan.bridge_pose.pose.position.z));
  bridge_node_->setOrientation(Ogre::Quaternion(plan.bridge_pose.pose.orientation.w, plan.bridge_pose.pose.orientation.x,
                                                plan.bridge_pose.pose.orientation.y, plan.bridge_pose.pose.orientation.z));
}

void Manip2D::handle(rviz::CollObjectHandle handle, Ogre::Viewport* vp, uint32_t x, uint32_t y)
{
  Ogre::Ray ray = vp->getCamera()->getCameraToViewportRay(((float)x) / vp->getActualWidth(), ((float)y) / vp->getActualHeight());
  Ogre::Plane table_plane(Ogre::Vector3::UNIT_Y, tool_->table_pos_.y);
  std::pair<bool, Ogre::Real> intersection = ray.intersects( table_plane );
  if (intersection.first)
  {
    Ogre::Vector3 pos = ray.getPoint(intersection.second);
    rviz::ogreToRobot(pos);
    if (handle == bridge_coll_)
    {
      bridge_node_->setPosition(pos);
    }
    else if (handle == rot_coll_)
    {
      Ogre::Vector3 center = bridge_node_->getPosition();
      double angle = atan2(pos.y - center.y, pos.x - center.x);

      bridge_node_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ));
    }
  }
}

void Manip2D::align(const Ogre::Vector3& lookat, const Ogre::Vector3& pos)
{
  Ogre::Vector3 vec = lookat - pos;

  float dist = vec.length() + Constants::BALL_RADIUS;
  dist = std::max(dist, Constants::BRIDGE_TO_STRIKE_MIN + Constants::BALL_RADIUS);
  dist = std::min(dist, Constants::BRIDGE_TO_STRIKE_MAX + Constants::BALL_RADIUS);

  Ogre::Vector3 new_pos = lookat - (vec.normalisedCopy() * dist);
  bridge_node_->setPosition(Ogre::Vector3(new_pos.x, new_pos.y, 0.0));

  Ogre::Vector3 center = bridge_node_->getPosition();
  double angle = atan2(lookat.y - pos.y, lookat.x - pos.x);

  bridge_node_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ));
}

#define RAIL_WIDTH Constants::RAIL_DEPTH
#define HALF_RAIL_WIDTH (RAIL_WIDTH * 0.5)
#define HALF_RAIL_HEIGHT (Constants::RAIL_HEIGHT * 0.5)
#define OFFTABLE_BALL_START Ogre::Vector3(0.0, -0.2, 0.0)

BilliardsTool::BilliardsTool(rviz::VisualizationManager* manager)
: rviz::Tool("Billiards", 'b', manager)
, sel_handler_(new rviz::SelectionHandler)
, left_is_down_(false)
, right_is_down_(false)
, table_pos_(Ogre::Vector3::ZERO)
, table_orient_(Ogre::Quaternion::IDENTITY)
, get_table_state_as_(ros::NodeHandle(), "rviz_get_table_state", boost::bind(&BilliardsTool::getTableStateCallback, this, _1))
, localize_table_as_(ros::NodeHandle(), "rviz_localize_table", boost::bind(&BilliardsTool::localizeTableCallback, this, _1))
, plan_shot_as_(ros::NodeHandle(), "rviz_plan_shot", boost::bind(&BilliardsTool::planShotCallback, this, _1))
, play_fixed_ac_("play_fixed", true)
, vision_get_table_state_ac_("vision_get_table_state", true)
, plan_shot_ac_("plan_shot", true)
, localize_table_ac_("localize_table", true)
, selected_bridge_manip_(0)
, selected_coll_(0)
{
  move_tool_ = new rviz::MoveTool("blah", 'a', manager);
  scene_node_ = manager->getSceneManager()->getRootSceneNode()->createChildSceneNode();

  Ogre::Vector3 v(Ogre::Vector3::ZERO);
  Ogre::Quaternion q(Ogre::Quaternion::IDENTITY);
  rviz::robotToOgre(v);
  rviz::robotToOgre(q);
  scene_node_->setPosition(v);
  scene_node_->setOrientation(q);

  Ogre::ColourValue table_col(217 / 255.0, 142 / 255.0, 178 / 255.0, 0.7);

  // East rail
  ogre_tools::Shape* rail = new ogre_tools::Shape(ogre_tools::Shape::Cube, manager->getSceneManager(), scene_node_);
  rail->setScale(Ogre::Vector3(Constants::TABLE_LENGTH, RAIL_WIDTH, Constants::RAIL_HEIGHT));
  rail->setPosition(Ogre::Vector3(Constants::TABLE_LENGTH/2.0, -HALF_RAIL_WIDTH, HALF_RAIL_HEIGHT));
  rail->setColor(table_col);
  table_.push_back(rail);

  // West
  rail = new ogre_tools::Shape(ogre_tools::Shape::Cube, manager->getSceneManager(), scene_node_);
  rail->setScale(Ogre::Vector3(Constants::TABLE_LENGTH, RAIL_WIDTH, Constants::RAIL_HEIGHT));
  rail->setPosition(Ogre::Vector3(Constants::TABLE_LENGTH/2.0, Constants::TABLE_WIDTH + HALF_RAIL_WIDTH, HALF_RAIL_HEIGHT));
  rail->setColor(table_col);
  table_.push_back(rail);

  // South
  rail = new ogre_tools::Shape(ogre_tools::Shape::Cube, manager->getSceneManager(), scene_node_);
  rail->setScale(Ogre::Vector3(RAIL_WIDTH, Constants::TABLE_WIDTH, Constants::RAIL_HEIGHT));
  rail->setPosition(Ogre::Vector3(-HALF_RAIL_WIDTH, Constants::TABLE_WIDTH/2.0, HALF_RAIL_HEIGHT));
  rail->setColor(table_col);
  table_.push_back(rail);

  // South
  rail = new ogre_tools::Shape(ogre_tools::Shape::Cube, manager->getSceneManager(), scene_node_);
  rail->setScale(Ogre::Vector3(RAIL_WIDTH, Constants::TABLE_WIDTH, Constants::RAIL_HEIGHT));
  rail->setPosition(Ogre::Vector3(Constants::TABLE_LENGTH + HALF_RAIL_WIDTH, Constants::TABLE_WIDTH/2.0, HALF_RAIL_HEIGHT));
  rail->setColor(table_col);
  table_.push_back(rail);

  // Table surface
  ogre_tools::Shape* table = new ogre_tools::Shape(ogre_tools::Shape::Cube, manager->getSceneManager(), scene_node_);
  table->setScale(Ogre::Vector3(Constants::TABLE_LENGTH, Constants::TABLE_WIDTH, 0.05));
  table->setPosition(Ogre::Vector3(Constants::TABLE_LENGTH / 2.0, Constants::TABLE_WIDTH / 2.0, -0.025));
  table->setColor(table_col);
  table_.push_back(table);

  for (uint32_t i = 0;i < 16; ++i)
  {
    Ball* b = new Ball;
    b->id = i;
    ogre_tools::Shape* ball = new ogre_tools::Shape(ogre_tools::Shape::Sphere, manager->getSceneManager(), scene_node_);
    b->shape = ball;
    ball->setScale(Ogre::Vector3(Constants::BALL_RADIUS*2, Constants::BALL_RADIUS*2, Constants::BALL_RADIUS*2));
    ball->setPosition(OFFTABLE_BALL_START + Ogre::Vector3(0.0, -(i * 0.1), Constants::BALL_RADIUS));

    std::stringstream ss;
    ss << i;
    b->id_text = new ogre_tools::MovableText(ss.str());
    b->text_node = ball->getRootNode()->createChildSceneNode();
    b->text_node->attachObject(b->id_text);
    b->text_node->setPosition(Ogre::Vector3(0.0, 0.0, 1.0));
    b->id_text->setColor(Ogre::ColourValue(0.2, 0.2, 1.0));
    b->id_text->setCharacterHeight(1.0);
    b->id_text->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
    //b->id_text

    rviz::SelectionManager* sel_man = manager_->getSelectionManager();
    b->coll = sel_man->createCollisionForObject(ball, sel_handler_);

    if (i == 0)
    {
      ball->setColor(Ogre::ColourValue(1.0, 1.0, 1.0));
    }
    else if (i > 0 && i < 8)
    {
      ball->setColor(Ogre::ColourValue(1.0, 0.0, 0.0));
    }
    else if (i == 8)
    {
      ball->setColor(Ogre::ColourValue(0.0, 0.0, 0.0));
    }
    else
    {
      ball->setColor(Ogre::ColourValue(1.0, 1.0, 0.0));
    }

    balls_.push_back(b);
  }

  bridge_manip_ = new Manip2D(manager_, scene_node_, this, 1.0);

  Ogre::SceneNode* base_parent = bridge_manip_->getSceneNode()->createChildSceneNode();
  // off wiki
#if 01
  Ogre::Quaternion base_parent_orient(Constants::BRIDGE_IN_BASE_QW, Constants::BRIDGE_IN_BASE_QX, Constants::BRIDGE_IN_BASE_QY, Constants::BRIDGE_IN_BASE_QZ);
  //rviz::robotToOgre(base_parent_orient);
  base_parent_orient = base_parent_orient.Inverse();
  base_parent->setOrientation(base_parent_orient);

  Ogre::Vector3 base_parent_pos(Constants::BRIDGE_IN_BASE_X, Constants::BRIDGE_IN_BASE_Y, Constants::BRIDGE_IN_BASE_Z);
  base_parent_pos = -base_parent_pos;
  base_parent_pos = base_parent_orient * base_parent_pos;
  //rviz::robotToOgre(base_parent_pos);

  base_parent->setPosition(base_parent_pos);
#endif
  base_manip_ = new Manip2D(manager_, base_parent, this, 2.0);

  radius_object_ = manager->getSceneManager()->createManualObject("BilliardsRobotRadius");
  base_manip_->getSceneNode()->attachObject(radius_object_);
  radius_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
  for( float f = 0; f <= Ogre::Math::TWO_PI; f += 0.01f )
  {
    radius_object_->position( Constants::ROBOT_RADIUS * cos(f), Constants::ROBOT_RADIUS * sin(f), 0.0f );
    radius_object_->colour(Ogre::ColourValue(1.0, 0.0, 0.0, 1.0));
  }
  radius_object_->end();

  localize_manip_ = new Manip2D(manager_, scene_node_, this, 1.5);

  scene_node_->setVisible(false);

  ros::NodeHandle nh;
  table_state_pub_ = nh.advertise<billiards_msgs::TableState>("rviz_table_state", 1);
  shot_plan_pub_ = nh.advertise<billiards_msgs::ShotPlan>("rviz_shot_plan", 1);
  localized_table_pub_ = nh.advertise<geometry_msgs::PoseStamped>("rviz_localized_table", 1);

  rviz::WindowManagerInterface* wm = manager_->getWindowManager();
  panel_ = new wxPanel(wm->getParentWindow(), wxID_ANY);
  wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
  panel_->SetSizer(sizer);

  wxButton* button = new wxButton(panel_, wxID_ANY, wxT("localize_table"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onLocalizeTable), NULL, this);

  button = new wxButton(panel_, wxID_ANY, wxT("vision_get_table_state"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onVisionGetTableState), NULL, this);

  button = new wxButton(panel_, wxID_ANY, wxT("plan_shot"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onPlanShot), NULL, this);

  button = new wxButton(panel_, wxID_ANY, wxT("play_fixed"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onPlay), NULL, this);

  wm->addPane("Poolshark", panel_);
}

BilliardsTool::~BilliardsTool()
{
  delete bridge_manip_;
  delete move_tool_;
  manager_->getSceneManager()->destroySceneNode(scene_node_);
}

void BilliardsTool::activate()
{
  scene_node_->setVisible(true);
  move_tool_->activate();

  rviz::WindowManagerInterface* wm = manager_->getWindowManager();
  wm->showPane(panel_);
}

void BilliardsTool::deactivate()
{
  move_tool_->deactivate();
  scene_node_->setVisible(false);

  rviz::WindowManagerInterface* wm = manager_->getWindowManager();
  wm->closePane(panel_);
}

void BilliardsTool::onPlay(wxCommandEvent& evt)
{
  billiards_msgs::PlayGoal goal;
  play_fixed_ac_.cancelAllGoals();
  play_fixed_ac_.sendGoal(goal, boost::bind(&BilliardsTool::onPlayDone, this, _1, _2));
}

void BilliardsTool::onVisionGetTableState(wxCommandEvent& evt)
{
  billiards_msgs::GetTableStateGoal goal;
  vision_get_table_state_ac_.cancelAllGoals();
  vision_get_table_state_ac_.sendGoal(goal, boost::bind(&BilliardsTool::onVisionGetTableStateDone, this, _1, _2));
}

void BilliardsTool::onPlanShot(wxCommandEvent& evt)
{
  billiards_msgs::PlanShotGoal goal;
  goal.state = getTableState();
  plan_shot_ac_.cancelAllGoals();
  plan_shot_ac_.sendGoal(goal, boost::bind(&BilliardsTool::onPlanShotDone, this, _1, _2));
}

void BilliardsTool::onLocalizeTable(wxCommandEvent& evt)
{
  billiards_msgs::LocalizeTableGoal goal;
  localize_table_ac_.cancelAllGoals();
  localize_table_ac_.sendGoal(goal);
}

void BilliardsTool::onPlayDone(const actionlib::SimpleClientGoalState& state, const billiards_msgs::PlayResultConstPtr& result)
{
  ROS_INFO("Play Goal done, state %s", state.toString().c_str());
}

void BilliardsTool::onVisionGetTableStateDone(const actionlib::SimpleClientGoalState& state, const billiards_msgs::GetTableStateResultConstPtr& result)
{
  ROS_INFO("vision_get_table_state goal done");

  if (result)
  {
    ROS_INFO("vision_get_table_state got result");
    ROS_INFO_STREAM(*result);
    for (size_t i = 0; i < 16; ++i)
    {
      balls_[i]->shape->setPosition(OFFTABLE_BALL_START + Ogre::Vector3(0.0, -(i * 0.1), Constants::BALL_RADIUS));
    }

    for (size_t i = 0; i < result->state.balls.size(); ++i)
    {
      balls_[result->state.balls[i].id]->shape->setPosition(Ogre::Vector3(result->state.balls[i].point.point.x, result->state.balls[i].point.point.y,
                                                                         result->state.balls[i].point.point.z));
    }
  }
}

void BilliardsTool::onPlanShotDone(const actionlib::SimpleClientGoalState& state, const billiards_msgs::PlanShotResultConstPtr& result)
{
  if (result && state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    bridge_manip_->set(result->shot);
  }
}

billiards_msgs::TableState BilliardsTool::getTableState()
{
  billiards_msgs::TableState ts;
  V_Ball::iterator it = balls_.begin();
  V_Ball::iterator end = balls_.end();
  for (; it != end; ++it)
  {
    Ball* b = *it;
    if (isOnTable(b))
    {
      Ogre::Vector3 pos = getBallPosInTableFrame(b);
      billiards_msgs::BallState bs;
      bs.point.header.frame_id = "table";
      bs.point.header.stamp = ros::Time::now();
      bs.id = b->id;
      bs.pocketed = !isOnTable(b);
      bs.point.point.x = pos.x;
      bs.point.point.y = pos.y;
      ts.balls.push_back(bs);
    }
  }

  return ts;
}

void BilliardsTool::publishTable()
{
  table_state_pub_.publish(getTableState());
}

billiards_msgs::ShotPlan BilliardsTool::getShotPlan()
{
  billiards_msgs::ShotPlan plan;

  Ogre::Vector3 bridge_pos = bridge_manip_->getSceneNode()->_getDerivedPosition();
  Ogre::Quaternion bridge_orient = bridge_manip_->getSceneNode()->_getDerivedOrientation();
  rviz::ogreToRobot(bridge_pos);
  rviz::ogreToRobot(bridge_orient);

  plan.bridge_pose.pose.position.x = bridge_pos.x;
  plan.bridge_pose.pose.position.y = bridge_pos.y;
  plan.bridge_pose.pose.position.z = bridge_pos.z;
  plan.bridge_pose.pose.orientation.x = bridge_orient.x;
  plan.bridge_pose.pose.orientation.y = bridge_orient.y;
  plan.bridge_pose.pose.orientation.z = bridge_orient.z;
  plan.bridge_pose.pose.orientation.w = bridge_orient.w;
  plan.bridge_pose.header.frame_id = manager_->getFixedFrame();
  plan.bridge_pose.header.stamp = ros::Time::now();

  Ogre::Vector3 base_pos = base_manip_->getSceneNode()->_getDerivedPosition();
  Ogre::Quaternion base_orient = base_manip_->getSceneNode()->_getDerivedOrientation();
  rviz::ogreToRobot(base_pos);
  rviz::ogreToRobot(base_orient);

  plan.base_pose.pose.position.x = base_pos.x;
  plan.base_pose.pose.position.y = base_pos.y;
  plan.base_pose.pose.position.z = base_pos.z;
  plan.base_pose.pose.orientation.x = base_orient.x;
  plan.base_pose.pose.orientation.y = base_orient.y;
  plan.base_pose.pose.orientation.z = base_orient.z;
  plan.base_pose.pose.orientation.w = base_orient.w;
  plan.base_pose.header.frame_id = manager_->getFixedFrame();
  plan.base_pose.header.stamp = plan.bridge_pose.header.stamp;

  plan.velocity = 1.5;

  return plan;
}

void BilliardsTool::publishShotPlan()
{
  shot_plan_pub_.publish(getShotPlan());
}

void BilliardsTool::onGetTableStateSucceed(wxCommandEvent& evt)
{
  billiards_msgs::GetTableStateResult result;
  result.state = getTableState();
  get_table_current_goal_.setSucceeded(result);
  get_table_state_frame_->Close();
}

void BilliardsTool::onGetTableStateAborted(wxCommandEvent& evt)
{
  get_table_current_goal_.setAborted();
  get_table_state_frame_->Close();
}

void BilliardsTool::getTableStateCallback(actionlib::ActionServer<billiards_msgs::GetTableStateAction>::GoalHandle goal)
{
  wxFrame* frame = new wxFrame(0, wxID_ANY, wxT("Get Table State"));
  wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
  frame->SetSizer(sizer);

  wxButton* button = new wxButton(frame, wxID_ANY, wxT("SUCCEED"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onGetTableStateSucceed), NULL, this);

  button = new wxButton(frame, wxID_ANY, wxT("ABORT"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onGetTableStateAborted), NULL, this);
  get_table_state_frame_ = frame;
  get_table_current_goal_ = goal;
  goal.setAccepted();
  frame->Show();
}

void BilliardsTool::onLocalizeTableSucceed(wxCommandEvent& evt)
{
  billiards_msgs::LocalizeTableResult result;
  result.child_frame = "table";
  result.pose = getLocalizedPose();
  localize_table_current_goal_.setSucceeded(result);
  localize_table_frame_->Close();
}

void BilliardsTool::onLocalizeTableAborted(wxCommandEvent& evt)
{
  localize_table_current_goal_.setAborted();
  localize_table_frame_->Close();
}

void BilliardsTool::localizeTableCallback(actionlib::ActionServer<billiards_msgs::LocalizeTableAction>::GoalHandle goal)
{
  wxFrame* frame = new wxFrame(0, wxID_ANY, wxT("Localize Table"));
  wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
  frame->SetSizer(sizer);

  wxButton* button = new wxButton(frame, wxID_ANY, wxT("SUCCEED"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onLocalizeTableSucceed), NULL, this);

  button = new wxButton(frame, wxID_ANY, wxT("ABORT"));
  sizer->Add(button, 1, wxEXPAND);
  button->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(BilliardsTool::onLocalizeTableAborted), NULL, this);
  localize_table_frame_ = frame;
  localize_table_current_goal_ = goal;
  goal.setAccepted();
  frame->Show();
}

void BilliardsTool::planShotCallback(const billiards_msgs::PlanShotGoalConstPtr& goal)
{
  billiards_msgs::PlanShotResult result;
  result.shot = getShotPlan();
  plan_shot_as_.setSucceeded(result);
}

geometry_msgs::PoseStamped BilliardsTool::getLocalizedPose()
{
  Ogre::Vector3 pos = localize_manip_->getSceneNode()->_getDerivedPosition();
  Ogre::Quaternion orient = localize_manip_->getSceneNode()->_getDerivedOrientation();
  rviz::ogreToRobot(pos);
  rviz::ogreToRobot(orient);

  tf::TransformListener* tf = manager_->getFrameManager()->getTFClient();

  ros::Time time;
  tf->getLatestCommonTime("table", "table_nav", time, 0);
  tf::Stamped<tf::Point> tf_pos(tf::Point(pos.x, pos.y, pos.z), time, "table");
  tf::Stamped<tf::Quaternion> tf_orient(tf::Quaternion(orient.x, orient.y, orient.z, orient.w), time, "table");
  try
  {
    tf->transformPoint("table_nav", tf_pos, tf_pos);
    tf->transformQuaternion("table_nav", tf_orient, tf_orient);
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Caught exception transforming from table_nav: %s", e.what());
  }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "table_nav";
  pose.header.stamp = time;
  pose.pose.position.x = tf_pos.x();
  pose.pose.position.y = tf_pos.y();
  pose.pose.position.z = tf_pos.z();

  pose.pose.orientation.x = tf_orient.x();
  pose.pose.orientation.y = tf_orient.y();
  pose.pose.orientation.z = tf_orient.z();
  pose.pose.orientation.w = tf_orient.w();
  return pose;
}

void BilliardsTool::updateTableFrame()
{
  localized_table_pub_.publish(getLocalizedPose());
}

void BilliardsTool::update(float wall_dt, float ros_dt)
{
  if (manager_->getFrameManager()->getTransform("table", ros::Time(), table_pos_, table_orient_, false))
  {
    scene_node_->setPosition(table_pos_);
    scene_node_->setOrientation(table_orient_);
  }

  publishTable();
  publishShotPlan();
  updateTableFrame();
}

BilliardsTool::Ball* BilliardsTool::pickBall(uint32_t x, uint32_t y, Ogre::Viewport* vp)
{
  rviz::SelectionManager* sel_man = manager_->getSelectionManager();
  sel_man->select(vp, x, y, x, y, rviz::SelectionManager::Replace);

  // Copy the selection
  rviz::M_Picked selection = sel_man->getSelection();
  sel_man->setSelection(rviz::M_Picked());

  rviz::M_Picked::iterator it = selection.begin();
  rviz::M_Picked::iterator end = selection.end();
  for (; it != end; ++it)
  {
    rviz::CollObjectHandle handle = it->second.handle;
    for (size_t i = 0; i < balls_.size(); ++i)
    {
      Ball* b = balls_[i];
      if (b->coll == handle)
      {
        return b;
      }
    }
  }

  return 0;
}

Ogre::Vector3 BilliardsTool::getBallPosInTableFrame(Ball* b)
{
  Ogre::Vector3 pos = b->shape->getPosition();
  //rviz::ogreToRobot(pos);

#if 0
  tf::TransformListener* tf = manager_->getFrameManager()->getTFClient();
  tf::Stamped<tf::Point> tf_pos(tf::Point(pos.x, pos.y, pos.z), ros::Time(), manager_->getFixedFrame());

  try
  {
    tf->transformPoint("table", tf_pos, tf_pos);
    pos.x = tf_pos.x();
    pos.y = tf_pos.y();
    pos.z = tf_pos.z();
  }
  catch (Ogre::Exception&)
  {

  }
#endif

  return pos;
}

bool BilliardsTool::isOnTable(Ball* b)
{
  Ogre::Vector3 pos = getBallPosInTableFrame(b);
  return !(  pos.x < Constants::BALL_RADIUS
          || pos.y < Constants::BALL_RADIUS
          || pos.x > Constants::TABLE_LENGTH - Constants::BALL_RADIUS
          || pos.y > Constants::TABLE_WIDTH - Constants::BALL_RADIUS);
}

int BilliardsTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if (event.event.ControlDown())
  {
    return move_tool_->processMouseEvent(event);
  }

  uint32_t x = event.event.GetX();
  uint32_t y = event.event.GetY();
  Ogre::Viewport* vp = event.viewport;

  if (event.event.LeftDown())
  {
    left_is_down_ = true;
    selected_ball_ = 0;
    selected_coll_ = 0;

    //Ball* ball = pickBall(x, y, event.viewport);

    rviz::SelectionManager* sel_man = manager_->getSelectionManager();
    sel_man->select(vp, x, y, x, y, rviz::SelectionManager::Replace);

    // Copy the selection
    rviz::M_Picked selection = sel_man->getSelection();
    sel_man->setSelection(rviz::M_Picked());

    rviz::M_Picked::iterator it = selection.begin();
    rviz::M_Picked::iterator end = selection.end();
    for (;!selected_coll_ && it != end; ++it)
    {
      rviz::CollObjectHandle handle = it->second.handle;

      if (bridge_manip_->collMatches(handle))
      {
        selected_bridge_manip_ = bridge_manip_;
        selected_coll_ = handle;
      }
      else if (localize_manip_->collMatches(handle))
      {
        selected_bridge_manip_ = localize_manip_;
        selected_coll_ = handle;
      }
#if 0
      else if (base_manip_->collMatches(handle))
      {
        selected_bridge_manip_ = base_manip_;
        selected_coll_ = handle;
      }
#endif
      else
      {
        for (size_t i = 0; i < balls_.size(); ++i)
        {
          Ball* b = balls_[i];
          if (b->coll == handle)
          {
            selected_ball_ = b;
            selected_coll_ = handle;
            break;
          }
        }
      }
    }
  }
  else if (left_is_down_ && event.event.Dragging())
  {
    if (selected_ball_)
    {
      Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay(((float)x) / vp->getActualWidth(), ((float)y) / vp->getActualHeight());
      Ogre::Plane table_plane(Ogre::Vector3::UNIT_Y, table_pos_.y);
      std::pair<bool, Ogre::Real> intersection = ray.intersects( table_plane );
      if (intersection.first)
      {
        Ogre::Vector3 pos = ray.getPoint(intersection.second);
        pos.y += Constants::BALL_RADIUS;
        rviz::ogreToRobot(pos);
        selected_ball_->shape->setPosition(pos);
      }
    }
    else if (selected_bridge_manip_)
    {
      selected_bridge_manip_->handle(selected_coll_, event.viewport, x, y);
    }
  }
  else if (left_is_down_ && event.event.LeftUp())
  {
    left_is_down_ = false;
    if (selected_ball_)
    {
      if (!isOnTable(selected_ball_))
      {
        //ROS_INFO("oUTSIDE");

        selected_ball_->shape->setPosition(OFFTABLE_BALL_START + Ogre::Vector3(0.0, -(selected_ball_->id * 0.1), Constants::BALL_RADIUS));
      }

      selected_ball_ = 0;
    }
    else if (selected_bridge_manip_)
    {
      selected_bridge_manip_ = 0;
    }

    selected_coll_ = 0;
  }
  else if (event.event.RightDown())
  {
    right_is_down_ = true;
    rviz::SelectionManager* sel_man = manager_->getSelectionManager();
    sel_man->select(vp, x, y, x, y, rviz::SelectionManager::Replace);

    // Copy the selection
    rviz::M_Picked selection = sel_man->getSelection();
    sel_man->setSelection(rviz::M_Picked());

    rviz::M_Picked::iterator it = selection.begin();
    rviz::M_Picked::iterator end = selection.end();
    for (;!selected_coll_ && it != end; ++it)
    {
      rviz::CollObjectHandle handle = it->second.handle;

      for (size_t i = 0; i < balls_.size(); ++i)
      {
        Ball* b = balls_[i];
        if (b->coll == handle)
        {
          selected_ball_ = b;
          selected_coll_ = handle;
          break;
        }
      }
    }
  }
  else if (right_is_down_ && event.event.Dragging())
  {
    if (selected_ball_)
    {
      Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay(((float)x) / vp->getActualWidth(), ((float)y) / vp->getActualHeight());
      Ogre::Plane table_plane(Ogre::Vector3::UNIT_Y, table_pos_.y);
      std::pair<bool, Ogre::Real> intersection = ray.intersects( table_plane );
      if (intersection.first)
      {
        Ogre::Vector3 pos = ray.getPoint(intersection.second);
        pos.y += Constants::BALL_RADIUS;
        rviz::ogreToRobot(pos);

        bridge_manip_->align(selected_ball_->shape->getPosition(), pos);
      }
    }
  }
  else if (right_is_down_ && event.event.RightUp())
  {
    right_is_down_ = false;
    selected_ball_ = 0;
    selected_coll_ = 0;
  }

  return rviz::Tool::Render;
}

BilliardsDummyDisplay::BilliardsDummyDisplay(const std::string & name, rviz::VisualizationManager * manager)
: Display(name, manager)
{
  if (!s_tool_)
  {
    s_tool_ = new BilliardsTool(manager);
    manager->addTool(s_tool_);

    ((wxFrame*)manager->getWindowManager()->getParentWindow())->SetTitle(wxT("Poolshark"));
  }
}

BilliardsDummyDisplay::~BilliardsDummyDisplay()
{
}

void BilliardsDummyDisplay::onEnable()
{
}

void BilliardsDummyDisplay::onDisable()
{
}

void BilliardsDummyDisplay::fixedFrameChanged()
{
}

void BilliardsDummyDisplay::update(float wall_dt, float ros_dt)
{
}

void BilliardsDummyDisplay::reset()
{
}

void BilliardsDummyDisplay::targetFrameChanged()
{
}

void BilliardsDummyDisplay::createProperties()
{
}

} // namespace mapping_rviz_plugin
