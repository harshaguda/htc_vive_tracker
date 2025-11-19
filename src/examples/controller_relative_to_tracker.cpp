#include "htc_vive_tracker.h"
#include <openvr.h>
#include <cmath>
#include <iomanip>
#include <unistd.h>

// Structure to hold transformation matrix
struct Transform {
    double matrix[3][4];  // 3x4 transformation matrix
};

// Function to convert pose and quaternion to transformation matrix
void PoseQuatToMatrix(const double pose[3], const double quat[4], Transform& transform) {
    // quat = [w, x, y, z]
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];
    
    // Convert quaternion to rotation matrix
    transform.matrix[0][0] = 1 - 2*y*y - 2*z*z;
    transform.matrix[0][1] = 2*x*y - 2*w*z;
    transform.matrix[0][2] = 2*x*z + 2*w*y;
    transform.matrix[0][3] = pose[0];
    
    transform.matrix[1][0] = 2*x*y + 2*w*z;
    transform.matrix[1][1] = 1 - 2*x*x - 2*z*z;
    transform.matrix[1][2] = 2*y*z - 2*w*x;
    transform.matrix[1][3] = pose[1];
    
    transform.matrix[2][0] = 2*x*z - 2*w*y;
    transform.matrix[2][1] = 2*y*z + 2*w*x;
    transform.matrix[2][2] = 1 - 2*x*x - 2*y*y;
    transform.matrix[2][3] = pose[2];
}

// Function to invert a transformation matrix
void InvertTransform(const Transform& input, Transform& output) {
    // For a transformation matrix T = [R | t], the inverse is [R^T | -R^T * t]
    // Since R is a rotation matrix, R^T = R^-1
    
    // Transpose the rotation part
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            output.matrix[i][j] = input.matrix[j][i];
        }
    }
    
    // Compute -R^T * t
    for (int i = 0; i < 3; i++) {
        output.matrix[i][3] = 0;
        for (int j = 0; j < 3; j++) {
            output.matrix[i][3] -= output.matrix[i][j] * input.matrix[j][3];
        }
    }
}

// Function to multiply two transformation matrices
void MultiplyTransforms(const Transform& t1, const Transform& t2, Transform& result) {
    // result = t1 * t2
    
    // Multiply rotation parts (3x3 * 3x3)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.matrix[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                result.matrix[i][j] += t1.matrix[i][k] * t2.matrix[k][j];
            }
        }
    }
    
    // Compute translation: R1 * t2 + t1
    for (int i = 0; i < 3; i++) {
        result.matrix[i][3] = t1.matrix[i][3];
        for (int j = 0; j < 3; j++) {
            result.matrix[i][3] += t1.matrix[i][j] * t2.matrix[j][3];
        }
    }
}

// Function to extract position from transformation matrix
void ExtractPosition(const Transform& transform, double position[3]) {
    position[0] = transform.matrix[0][3];
    position[1] = transform.matrix[1][3];
    position[2] = transform.matrix[2][3];
}

// Function to compute relative position of controller with respect to tracker
void ComputeRelativePosition(CHtc_Vive_Tracker& vt, 
                              const std::string& controller_name,
                              const std::string& tracker_name,
                              double relative_position[3]) {
    double controller_pose[3], controller_quat[4];
    double tracker_pose[3], tracker_quat[4];
    
    // Get absolute poses
    bool controller_success = vt.GetDevicePoseQuaternion(controller_name, controller_pose, controller_quat);
    bool tracker_success = vt.GetDevicePoseQuaternion(tracker_name, tracker_pose, tracker_quat);
    
    if (!controller_success || !tracker_success) {
        std::cout << "Error: Could not get poses for both devices" << std::endl;
        relative_position[0] = relative_position[1] = relative_position[2] = 0.0;
        return;
    }
    
    // Convert to transformation matrices
    Transform T_world_controller, T_world_tracker;
    PoseQuatToMatrix(controller_pose, controller_quat, T_world_controller);
    PoseQuatToMatrix(tracker_pose, tracker_quat, T_world_tracker);
    
    // Compute inverse of tracker transform
    Transform T_tracker_world;
    InvertTransform(T_world_tracker, T_tracker_world);
    
    // Compute relative transform: T_tracker_controller = T_tracker_world * T_world_controller
    Transform T_tracker_controller;
    MultiplyTransforms(T_tracker_world, T_world_controller, T_tracker_controller);
    
    // Extract position
    ExtractPosition(T_tracker_controller, relative_position);
}

// Function to compute distance between controller and tracker
double ComputeDistance(const double relative_position[3]) {
    return sqrt(relative_position[0] * relative_position[0] +
                relative_position[1] * relative_position[1] +
                relative_position[2] * relative_position[2]);
}

int main(int argc, char *argv[]) {
    bool verbose = false;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-v") {
            verbose = true;
        }
        if (std::string(argv[i]) == "-h") {
            std::cout << "Usage: " << argv[0] << " [-v] [-h]" << std::endl;
            std::cout << "  -v : Verbose mode" << std::endl;
            std::cout << "  -h : Show this help" << std::endl;
            std::cout << "\nThis example prints the position of controller_1 relative to tracker_1" << std::endl;
            return 0;
        }
    }
    
    // Initialize HTC Vive Tracker
    CHtc_Vive_Tracker vt;
    
    if (!vt.InitializeVR(verbose)) {
        std::cout << "Failed to initialize VR system" << std::endl;
        return 1;
    }
    
    // Check if required devices are detected
    if (vt.GetAllDeviceNames().size() == 1) {
        std::cout << "No devices detected. Check that devices are connected and paired" << std::endl;
        return 1;
    }
    
    std::cout << "VR System initialized successfully" << std::endl;
    vt.PrintAllDetectedDevices();
    std::cout << std::endl;
    
    // Define device names
    std::string controller_name = "controller_1";
    std::string tracker_name = "tracker_1";
    
    // Check if both devices are detected
    if (!vt.IsDeviceDetected(controller_name)) {
        std::cout << "Error: " << controller_name << " not detected!" << std::endl;
        return 1;
    }
    
    if (!vt.IsDeviceDetected(tracker_name)) {
        std::cout << "Error: " << tracker_name << " not detected!" << std::endl;
        return 1;
    }
    
    std::cout << "Both " << controller_name << " and " << tracker_name << " detected!" << std::endl;
    std::cout << "\nPress Ctrl+C to exit\n" << std::endl;
    
    // Main loop - continuously print relative position
    while (true) {
        // Update device poses
        vt.Update();
        
        // Compute relative position
        double relative_position[3];
        ComputeRelativePosition(vt, controller_name, tracker_name, relative_position);
        
        // Compute distance
        double distance = ComputeDistance(relative_position);
        
        // Print results
        std::cout << "\r" << std::flush;  // Clear line
        std::cout << "Controller position relative to Tracker: ";
        std::cout << "X=" << std::fixed << std::setprecision(3) << relative_position[0] << "m, ";
        std::cout << "Y=" << std::fixed << std::setprecision(3) << relative_position[1] << "m, ";
        std::cout << "Z=" << std::fixed << std::setprecision(3) << relative_position[2] << "m ";
        std::cout << "| Distance=" << std::fixed << std::setprecision(3) << distance << "m   ";
        std::cout << std::flush;
        
        // Sleep for a short time (100ms)
        usleep(100000);
    }
    
    return 0;
}
