#include <functional>
#include <thread>
#include <mutex>
#include <list>
#include <cmath>

#include <ignition/common/Profiler.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/Link.hh>


#include "elevator_plugin/ElevatorPlugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
// Internal implementation
class ElevatorPlugin::ElevatorPluginPrivate
{
public:
  // State machine base
  class State { public: virtual ~State()=default; virtual void Start(){}; virtual bool Update(){return true;} bool started{false}; };
  class MoveState;
  class WaitState;

  // Controller
  class LiftController;

  physics::ModelPtr model;
  sdf::ElementPtr sdf;
  physics::JointPtr liftJoint;

  LiftController *liftController{nullptr};

  std::list<State*> states;
  std::mutex stateMutex;

  transport::NodePtr node;
  transport::SubscriberPtr elevatorSub;
  event::ConnectionPtr updateConnection;

  ~ElevatorPluginPrivate();
};

//////////////////////////////////////////////////
// Controller
class ElevatorPlugin::ElevatorPluginPrivate::LiftController
{
public:
  enum State { MOVING, STATIONARY };

  LiftController(physics::JointPtr _joint, float h) : state(STATIONARY), floor(0), floorHeight(h), liftJoint(_joint)
  { liftPID.Init(100000, 0, 100000.0); }

  void Reset() { prevSimTime = common::Time::Zero; }
  void SetFloor(int f) { floor = f; }
  int GetFloor() const { return floor; }
  State GetState() const { return state; }

  bool Update(const common::UpdateInfo &_info)
  {
    if (prevSimTime == common::Time::Zero) { prevSimTime = _info.simTime; return false; }
    double err = liftJoint->Position() - (floor * floorHeight);
    double force = liftPID.Update(err, _info.simTime - prevSimTime);
    prevSimTime = _info.simTime;
    liftJoint->SetForce(0, force);
    if (std::abs(err) < 0.15) { state = STATIONARY; return true; }
    state = MOVING; return false;
  }

private:
  State state;
  int floor;
  float floorHeight;
  physics::JointPtr liftJoint;
  common::Time prevSimTime{0};
  gazebo::common::PID liftPID;
};

//////////////////////////////////////////////////
// States
class ElevatorPlugin::ElevatorPluginPrivate::MoveState : public State
{
public:
  MoveState(int f, LiftController *_c) : floor(f), ctrl(_c) {}
  void Start() override { ctrl->SetFloor(floor); started = true; }
  bool Update() override
  {
    if (!started) { Start(); return false; }
    return ctrl->GetState() == LiftController::STATIONARY;
  }
private:
  int floor;
  LiftController *ctrl;
};

class ElevatorPlugin::ElevatorPluginPrivate::WaitState : public State
{
public:
  WaitState(const common::Time &_t) : waitTime(_t) {}
  void Start() override { start = common::Time::GetWallTime(); started = true; }
  bool Update() override
  {
    if (!started) { Start(); return false; }
    return (common::Time::GetWallTime() - start) >= waitTime;
  }
private:
  common::Time waitTime;
  common::Time start{0};
};

//////////////////////////////////////////////////
ElevatorPlugin::ElevatorPluginPrivate::~ElevatorPluginPrivate()
{
  delete liftController;
  for (auto s : states) delete s;
  states.clear();
}

//////////////////////////////////////////////////
// Plugin main class
GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)

ElevatorPlugin::ElevatorPlugin() : dataPtr(new ElevatorPluginPrivate), current_floor_(0) {}
ElevatorPlugin::~ElevatorPlugin() { this->dataPtr->updateConnection.reset(); }

void ElevatorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  this->dataPtr->sdf = _sdf;

  float floorHeight = 3.0;
  if (_sdf->HasElement("floor_height"))
    floorHeight = _sdf->Get<float>("floor_height");

  std::string liftJointName = _sdf->Get<std::string>("lift_joint");

  this->dataPtr->liftJoint = _model->GetJoint(liftJointName);
  if (!this->dataPtr->liftJoint)
  {
    gzerr << "[ElevatorPlugin] lift_joint missing\n";
    return;
  }

  this->dataPtr->liftController = new ElevatorPluginPrivate::LiftController(this->dataPtr->liftJoint, floorHeight);

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ElevatorPlugin::Update, this, std::placeholders::_1));

  // Initialize Gazebo transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(_model->GetWorld()->Name());

  // Subscribe to Gazebo topic
  std::string topicName = "~/elevator";
  if (_sdf->HasElement("topic"))
    topicName = _sdf->Get<std::string>("topic");

  this->dataPtr->elevatorSub = this->dataPtr->node->Subscribe(topicName, &ElevatorPlugin::OnElevator, this);

  // ROS 2 Node
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // ROS 2 Service
  this->srv_ = this->ros_node_->create_service<std_srvs::srv::SetBool>(
    "/elevator_cmd",
    std::bind(&ElevatorPlugin::RosServiceCb, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->ros_node_->get_logger(), "ElevatorPlugin ROS2 service /elevator_cmd ready");
  RCLCPP_INFO(this->ros_node_->get_logger(), "ElevatorPlugin Gazebo topic %s ready", topicName.c_str());
}

//////////////////////////////////////////////////
// NEW: Check if robot "mobile_manipulator" is inside elevator
bool ElevatorPlugin::IsRobotInsideElevator() const
{
  auto world = this->dataPtr->model->GetWorld();
  auto robot = world->ModelByName("mobile_manipulator");

  if (!robot)
  {
    gzdbg << "[ElevatorPlugin] Robot 'mobile_manipulator' not found in world.\n";
    return false;
  }

  // Get elevator car link
  auto carLink = this->dataPtr->model->GetLink("car_link");
  if (!carLink)
  {
    gzerr << "[ElevatorPlugin] car_link not found!\n";
    return false;
  }

  // Get bounding box of elevator car
  auto carBox = carLink->BoundingBox();

  // Expand box slightly to be more forgiving
  ignition::math::Vector3d expand(0.2, 0.2, 0.2);
  carBox.Min() -= expand;
  carBox.Max() += expand;

  //  Get ROBOT'S CENTER POSITION
  auto robotPos = robot->WorldPose().Pos();

  //  Check if robot's center is inside elevator bounding box
  bool inside = carBox.Contains(robotPos);

  if (!inside)
  {
    gzdbg << "[ElevatorPlugin] Robot center NOT inside elevator (pos=" << robotPos << ")\n";
  }

  return inside;
}

//////////////////////////////////////////////////
void ElevatorPlugin::RosServiceCb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  //  SAFETY CHECK: Is robot inside?
  if (!IsRobotInsideElevator())
  {
    res->success = false;
    res->message = "Robot 'mobile_manipulator' is not inside elevator. Movement canceled.";
    RCLCPP_WARN(this->ros_node_->get_logger(), "Robot not inside elevator — movement canceled.");
    return;
  }

  int target = current_floor_;
  if (req->data)
    target += 1;  // Go up
  else
    target = std::max(0, current_floor_ - 1);  // Go down

  //  MOVE WITH 0.3s DELAY (since robot is inside)
  MoveToFloor(target);

  current_floor_ = target;
  res->success = true;
  res->message = "Robot detected inside — moving to floor " + std::to_string(target) + " after 0.3s delay";
}

//////////////////////////////////////////////////
//  Modified: Now uses 0.3s delay
void ElevatorPlugin::MoveToFloor(const int f)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Clear any pending states
  for (auto s : this->dataPtr->states) delete s;
  this->dataPtr->states.clear();

  //  0.3 second delay before moving
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::WaitState(common::Time(0, 300000000))); // 0.3s = 300ms
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::MoveState(f, this->dataPtr->liftController));
}

//////////////////////////////////////////////////
void ElevatorPlugin::OnElevator(ConstGzStringPtr &_msg)
{
  try
  {
    int f = std::stoi(_msg->data());

    //  SAFETY CHECK
    if (!IsRobotInsideElevator())
    {
      gzerr << "[ElevatorPlugin] Robot not inside — ignoring floor command " << f << "\n";
      return;
    }

    //  MOVE WITH 0.3s DELAY
    MoveToFloor(f);
    current_floor_ = f;
    gzmsg << "Robot inside — moving to floor " << f << " (with 0.3s delay)\n";
  }
  catch (...)
  {
    gzerr << "Invalid elevator command: " << _msg->data() << "\n";
  }
}

//////////////////////////////////////////////////
void ElevatorPlugin::Update(const common::UpdateInfo &_info)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  if (!this->dataPtr->states.empty())
  {
    if (this->dataPtr->states.front()->Update())
    {
      delete this->dataPtr->states.front();
      this->dataPtr->states.pop_front();
    }
  }

  this->dataPtr->liftController->Update(_info);
}

//////////////////////////////////////////////////
void ElevatorPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  for (auto s : this->dataPtr->states) delete s;
  this->dataPtr->states.clear();
  this->dataPtr->liftController->Reset();
}