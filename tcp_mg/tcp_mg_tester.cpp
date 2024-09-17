#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>
#include <fstream>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "tcp_mg.h"

// Function to write joint states to a file
void writeJointStates(const franka::RobotState& state, std::ofstream& file) {
    for (double joint : state.q) {
        file << joint << " ";
    }
    file << "\n";
}

// Function to write end effector position and orientation to a file
void writeEndEffectorState(const franka::RobotState& state, const Eigen::Vector3d& reference_position, const Eigen::Vector3d& reference_rpy, std::ofstream& file) {
    Eigen::Map<const Eigen::Matrix4d> transform(state.O_T_EE.data());
    Eigen::Vector3d position = transform.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d rpy = rotation.eulerAngles(0, 1, 2); // Roll, Pitch, Yaw

    file << state.time.toSec() << " "
         << position.transpose() << " "
         << rpy.transpose() << " "
         << reference_position.transpose() << " "
         << reference_rpy.transpose() << "\n";
}

int main() {
    // Connect to the robot
    franka::Robot robot("192.168.40.45");

    std::array<double, 6> starting_position = {{0.5, 0.3, 0.5, -3 * M_PI_4, 0, M_PI_2}};
    std::array<double, 6> final_position = {{0.4, 0.0, 0.5, 3 * M_PI_4, -M_PI_4, -2 * M_PI_4}};

    // Create MotionGenerator objects for the starting and final positions
    MotionGenerator starting_motion_generator(0.1, starting_position);
    MotionGenerator final_motion_generator(0.1, final_position);

    // Open a file to write joint states
    std::ofstream jointStateFile("joint_states.txt");
    if (!jointStateFile.is_open()) {
        std::cerr << "Failed to open joint states file for writing." << std::endl;
        return -1;
    }

    // Open a file to write end effector states
    std::ofstream eeStateFile("end_effector_states.txt");
    if (!eeStateFile.is_open()) {
        std::cerr << "Failed to open end effector states file for writing." << std::endl;
        return -1;
    }

    // Reference position and orientation
    Eigen::Vector3d reference_position(final_position[0], final_position[1], final_position[2]);
    Eigen::Vector3d reference_rpy(final_position[3], final_position[4], final_position[5]);

    try {
        std::cout << "Moving to final position..." << std::endl;
        robot.control(starting_motion_generator);   

        robot.control([&final_motion_generator, &jointStateFile, &eeStateFile, &reference_position, &reference_rpy](const franka::RobotState& state, franka::Duration dt) -> franka::CartesianPose {
            writeJointStates(state, jointStateFile);
            writeEndEffectorState(state, reference_position, reference_rpy, eeStateFile);
            return final_motion_generator(state, dt);
        });

        std::cout << "Motion completed successfully." << std::endl;

    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        jointStateFile.close();
        eeStateFile.close();
        return -1;
    }

    // Close the output files
    jointStateFile.close();
    eeStateFile.close();

    // Call the Python script to generate the graph
    system("python3 ../plot_joint_states.py");   
    system("python3 ../plot_end_effector_states.py");

    return 0;
}