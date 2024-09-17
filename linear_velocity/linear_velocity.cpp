#include <cmath>
#include <array>
#include <atomic>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fstream>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>


#include "tcp_mg.h"
#include "franka_ik_He.hpp"


// ############################################
// function to write the TCP position to a file
// ############################################

void writeTCPPosition(double time, const Eigen::Vector3d& position, const Eigen::Vector3d& pose, const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_d, const Eigen::Vector3d& desired_position, const Eigen::Vector3d& desired_rpy, std::ofstream& file) {
  file << time << " " << position[0] << " " << position[1] << " " << position[2] << " "
       << pose[0] << " " << pose[1] << " " << pose[2] << " "
       << rpy[0] << " " << rpy[1] << " " << rpy[2] << " " 
       << rpy_d[0] << " " << rpy_d[1] << " " << rpy_d[2] + M_PI << " "
       << desired_position[0] << " " << desired_position[1] << " " << desired_position[2] << " "
       << desired_rpy[0] << " " << desired_rpy[1] << " " << desired_rpy[2] << "\n";
}


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Create a text file for logging TCP positions
  std::ofstream outputFile("tcp_positions.txt");

  // Connect to the robot
  franka::Robot robot("192.168.40.45");
  franka::Model model = robot.loadModel();

//* #################################################################
//* ############### parameters for the controlers ###################
//* #################################################################

  constexpr double k_p{6.0};  // NOLINT (readability-identifier-naming)
  constexpr double k_i{0.0};  // NOLINT (readability-identifier-naming)
  constexpr double k_d{0.0};  // NOLINT (readability-identifier-naming)

//* #################################################################
//* #################################################################
//* #################################################################

  // parameter for halting the movement
  std::atomic_bool running{true};

  // main function for robot operations
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);


    // First move the robot to a suitable joint configuration
    std::array<double, 6> q_goal = {{0.5, 0.3, 0.5, -3 * M_PI_4, 0, M_PI_2}};  
    MotionGenerator motion_generator(0.5, q_goal); 
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // set collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{300, 300, 300, 250, 250, 200, 200}});
    robot.setCartesianImpedance({{300, 300, 300, 30, 30, 30}});


    franka::RobotState initial_state = robot.readOnce();

    Eigen::VectorXd initial_pose_ext(3), pose_error_integral(3), pose_error_derivative(3), pose_d(3), last_pose(3), next_pose(3), pose_final_l(3), pose_final_r(3), pose_aux(3);
    pose_error_integral.setZero();
 //!#############################################  
    Eigen::VectorXd vel_error(7), vel_error_integral(7), vel_error_derivative(7);
 //!#############################################  
    Eigen::Vector3d initial_position;
    double time = 0.0;
    auto get_position = [](const franka::RobotState& robot_state) {
      return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                             robot_state.O_T_EE[14]);
    };

    // the constant for discretisizing the trajectory
    int i=10000; //! TU SAM <=============================================
    int initial_i = i;
    double reference = 1e-4;

    // from another age, maybe will be used later
    pose_d << get_position(initial_state);

    //will probably be used later for roatation
    Eigen::Quaterniond rotation_d;
    Eigen::Matrix3d Rot, Rot_d, current_Rot;

    Eigen::VectorXd allowed_error(3);
    allowed_error << 0.0001, 0.0001, 0.0001;

 //!#############################################
    vel_error_integral.setZero();
    vel_error_derivative.setZero();
 //!#############################################


  //* #################################################################
  //* ############### from task space to joint space ##################
  //* #################################################################

  // grabbing the beginning robot pose
  std::array<double, 16> pose_array = initial_state.O_T_EE_d;
  // for now, changing the coordinates I want to change

  pose_final_l << 0.4, 0.0, 0.5;
  pose_aux << (pose_final_l - get_position(initial_state))/i;

  // rotation part of the movement
  pose_final_r << 3 * M_PI_4, -M_PI_4, -2 * M_PI_4; 
  //defining my first quaternion from the starting position
  Rot << pose_array[0], pose_array[4], pose_array[8],
        pose_array[1], pose_array[5], pose_array[9],
        pose_array[2], pose_array[6], pose_array[10];
  Eigen::Quaterniond q_0_(Rot);
  q_0_ = q_0_.normalized();

  Eigen::AngleAxisd rollAngle(pose_final_r[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pose_final_r[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(pose_final_r[2], Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q_1_ = yawAngle * pitchAngle * rollAngle;
  q_1_ = q_1_.normalized();


  //* #################################################################
  //* #################################################################
  //* #################################################################

  //? ##############################################################################
  //? #################### BEGINNING OF THE CALLBACK FUNCTION ######################
  //? ##############################################################################

    std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
      motion_control_callback = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      if (time == 0.0) {// Define your reference value
          last_pose[0] = 0.0;
          last_pose[1] = 0.0;
          last_pose[2] = 0.0;
      }

      
      next_pose = get_position(robot_state);
      last_pose = next_pose;

      std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_current_unique(robot_state.q.data());

      Eigen::VectorXd pose_ext(3), desired_joint_motion(7), control_output(3), control_output_aux(3), pose_ext_prior(3);
//!###################################################################
      Eigen::VectorXd vel_d(6), j_vel_d(7), vel_d_aux(6), vel_error_prior(7), q_desired(7);
//!###################################################################
      desired_joint_motion.setZero();
      pose_ext << pose_d - get_position(robot_state);

//!###################################################################
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! OVDJE POCINJE NOVI DIO KODA KOJI SE VEZE NA NOVI UPRAVLJACKI ZAKON
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!###################################################################

      if (i > 0) {
        pose_d += pose_aux;
        double t_slerp = 1.0 - (static_cast<double>(i) / initial_i);
        rotation_d = q_0_.slerp(t_slerp, q_1_);
        if (rotation_d.coeffs().isMuchSmallerThan(reference)) {
          rotation_d = q_0_;
        }
        i--;
      } else {
        pose_d = pose_final_l;
        rotation_d = q_1_;
      }

      Rot_d = rotation_d.normalized().toRotationMatrix();

      pose_array[0] = Rot_d(0, 0); pose_array[1] = Rot_d(1, 0); pose_array[2] = Rot_d(2, 0);
      pose_array[4] = Rot_d(0, 1); pose_array[5] = Rot_d(1, 1); pose_array[6] = Rot_d(2, 1);
      pose_array[8] = Rot_d(0, 2); pose_array[9] = Rot_d(1, 2); pose_array[10] = Rot_d(2, 2);

      pose_array[12] = pose_d[0];
      pose_array[13] = pose_d[1];
      pose_array[14] = pose_d[2];

      Eigen::Vector3d rpy_ff = Rot_d.eulerAngles(0, 1, 2);

      // vel_d.head(3) = pose_d;
      // vel_d.tail(3) = rpy_ff;

      vel_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

      j_vel_d = (jacobian.transpose() * vel_d);
      
      std::array<double, 7> q_actual = initial_state.q;

      // initializing the desired joint positions which will be sent to the robot
      std::array< std::array<double, 7>, 4 > q_j_final_aux;
      std::array<double, 7> q_j_final;
      q_j_final = franka_IK_EE_CC(pose_array, q_actual[6], q_actual);

      //mapping so that i can use it in the control lawrpy
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_j_final_map(q_j_final.data());

      vel_error = q_j_final_map - q_current_unique;

      vel_error_integral += period.toSec() * (vel_error);

      vel_error_derivative = (vel_error - vel_error_prior) / period.toSec();

      if(time == 0.0){
        vel_error_derivative.setZero();
      }

      vel_error_prior << vel_error;

      std::array<double, 16> current_pose_array = robot_state.O_T_EE_d;

      current_Rot << current_pose_array[0], current_pose_array[4], current_pose_array[8],
             current_pose_array[1], current_pose_array[5], current_pose_array[9],
             current_pose_array[2], current_pose_array[6], current_pose_array[10];

      Eigen::Vector3d rpy = current_Rot.eulerAngles(0, 1, 2);

      desired_joint_motion << (k_p * vel_error + k_i * vel_error_integral + k_d * vel_error_derivative);

//!###################################################################
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! OVDJE ZAVRSAVA NOVI DIO KODA KOJI SE VEZE NA NOVI UPRAVLJACKI ZAKON
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!###################################################################
      
      std::array<double, 7> desired_joint_motion_array;
      Eigen::VectorXd::Map(&desired_joint_motion_array[0], desired_joint_motion.size()) =
          desired_joint_motion;

      //! Write TCP position to file
      writeTCPPosition(time, next_pose, pose_final_l, rpy, pose_final_r, pose_d, rpy_ff, outputFile);  

      //checking the error marging so that i can stop the movement, otherwise it would go on forever
      if ((q_current_unique - q_j_final_map).cwiseAbs().maxCoeff() <= allowed_error.maxCoeff()) {
         running = false;
         return franka::MotionFinished(franka::JointVelocities(desired_joint_motion_array));
      }
      return desired_joint_motion_array;
    };
  std::cout << "WARNING: Make sure sure that no endeffector is mounted and that the robot's last "
                  "joint is "
                  "in contact with a horizontal rigid surface before starting. Keep in mind that "
                  "collision thresholds are set to high values."
                << std::endl
                << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
  // start real-time control loop
  robot.control(motion_control_callback);

  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  outputFile.close();

  system("python3 ../tcp_positions.py");


  return 0;
}
