#include <Eigen/Dense>
#include <iostream>
#include <cmath>

int main() {
    // Create a sample rotation matrix
    Eigen::Matrix3d R;
    // ref : https://www.andre-gaschler.com/rotationconverter/
    R << 0.4741599, -0.7384602,  0.4794255,
         0.6726189, -0.0475419, -0.7384602,
         0.5681164,  0.6726189,  0.4741599;

    float sy = sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
    bool singular = sy < 1e-6; // If

    float roll, pitch, yaw;
    if (!singular){
        roll = atan2(R(2,1) , R(2,2));
        pitch = atan2(-R(2,0), sy);
        yaw = atan2(R(1,0), R(0,0));
    } else{
        roll = atan2(-R(1,2), R(1,1));
        pitch = atan2(-R(2,0), sy);
        yaw = 0;
    }

    // // Extract roll, pitch, and yaw
    // double roll, pitch, yaw;
    // pitch = asin(-R(2,0));
    // roll = atan2(R(2,1), R(2,2));
    // yaw = atan2(R(1,0), R(0,0));

    // // Convert to degrees
    // roll *= 180/M_PI;
    // pitch *= 180/M_PI;
    // yaw *= 180/M_PI;

    // Print results
    std::cout << "Roll: " << roll << std::endl;
    std::cout << "Pitch: " << pitch << std::endl;
    std::cout << "Yaw: " << yaw << std::endl;

    return 0;
}