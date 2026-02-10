#include "MobileArmController.h"

#include <mc_rbdyn/RobotLoader.h>
#include <thread>
#include <chrono>

MobileArmController::MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt,
                                         const mc_rtc::Configuration &config)
    : mc_control::MCController(
          {rm,
           mc_rbdyn::RobotLoader::get_robot_module(
               "env", "/usr/local/share/mc_dingo", std::string("dingo")),
           mc_rbdyn::RobotLoader::get_robot_module(
               "env",
               std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH) +
                   "/../mc_int_obj_description",
               std::string("door")),
           mc_rbdyn::RobotLoader::get_robot_module(
               "env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
               std::string("ground"))},
          dt) {
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  postureTask->stiffness(1.0);
  solver().setContacts({{}});
  // addContact({"ur5e", "dingo", "Base", "Base"});
  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  double friction = mc_rbdyn::Contact::defaultFriction;
  dof[2] = 0.0;
  dof[3] = 0.0;
  dof[4] = 0.0;
  addContact({"ground", "dingo", "AllGround", "Base", friction, dof});
  mc_rtc::log::success("MobileArmController init done ");
}

bool MobileArmController::run() {
  if (count < 1000) {
    count++;
    return mc_control::MCController::run();
  }
  if (phase == APPROACH && handTask_->eval().norm() < 0.5 &&
      handTask_->speed().norm() < 1e-4) {
    addContact({"ur5e", "door", "Wrist", "Handle"});
    solver().removeTask(handTask_);
    postureTask->reset();
    doorPostureTask_->target({{"handle", {-1.0}}});
    phase = HANDLE;
  } else if (phase == HANDLE && doorPostureTask_->eval().norm() < 0.01) {
    doorPostureTask_->target({{"door", {-0.5}}});
    phase = OPEN;
  }
  return mc_control::MCController::run();
}

void MobileArmController::reset(
    const mc_control::ControllerResetData &reset_data) {
  auto inf = std::numeric_limits<double>::infinity();
  robots().robot(1).tl()[0] = {0, 0, -inf, -inf, -inf, 0};
  robots().robot(1).tu()[0] = {0, 0, inf, inf, inf, 0};
  // dingoDynamics_ = std::make_shared<mc_solver::KinematicsConstraint>(
  //     robots(), 1, solver().dt());
  // solver().addConstraintSet(*dingoDynamics_);
  doorKinematics_ = std::make_shared<mc_solver::KinematicsConstraint>(
      robots(), 2, solver().dt());
  solver().addConstraintSet(*doorKinematics_);
  mc_control::MCController::reset(reset_data);
  robots().robot(0).posW(
      sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 0.0845)));
  robots().robot(1).posW(
      sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 1.0)));
  robots().robot(2).posW(
      sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(1.0, 1.0, 0)));
  doorPostureTask_ =
      std::make_shared<mc_tasks::PostureTask>(solver(), 2, 5.0, 1000.0);
  solver().addTask(doorPostureTask_);
  handTask_ =
      std::make_shared<mc_tasks::SurfaceTransformTask>("Wrist", robots(), 0);
  solver().addTask(handTask_);
  handTask_->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.0)) *
                    robots().robot(2).surfacePose("Handle"));
  dingoEndEffectorTask_ =
      std::make_shared<mc_tasks::EndEffectorTask>("base_link", robots(),
      1);
  dingoEndEffectorTask_->positionTask->stiffness(3.0);
  dingoEndEffectorTask_->positionTask->weight(1e100);
  dingoEndEffectorTask_->orientationTask->stiffness(1.0);
  dingoEndEffectorTask_->orientationTask->weight(1e100);
  solver().addTask(dingoEndEffectorTask_);
  dingoEndEffectorTask_->add_ef_pose({Eigen::Vector3d(1.0, 0.0, 0.0)});
}

CONTROLLER_CONSTRUCTOR("MobileArmController", MobileArmController)
