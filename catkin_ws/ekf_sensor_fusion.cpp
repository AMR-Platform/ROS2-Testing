/*  ekf_sensor_fusion.cpp  ------------------------------------------------------
    Build:  g++ -std=c++17 -O2 ekf_sensor_fusion.cpp -o ekf \
            -I /usr/include/eigen3
    Run  :  cat log.csv | ./ekf
            (where log.csv contains timestamp, dS, dTheta_odom, yaw_imu_deg)

    Input CSV fields ------------------------------------------------------------
        t_sec,  dS_m,  dTheta_odom_deg,  yaw_imu_deg
    Example:
        0.02, 0.0015, 0.10,  12.45
        0.04, 0.0016, 0.12,  12.57
        ...
    -----------------------------------------------------------------------------
*/

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

using Vec3   = Eigen::Vector3d;
using Mat3   = Eigen::Matrix3d;
using Row3   = Eigen::RowVector3d;
using namespace std::literals;

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// configure noise (tune for your platform)
const Mat3  Q = (Vec3{1e-4, 1e-4, 1e-5}.asDiagonal());  // process noise
const double R_yaw = 0.02 * DEG2RAD;                   // IMU yaw σ ≈ 0.02 °

int main()
{
    // ── EKF state ----------------------------------------------------------------
    Vec3 x = Vec3::Zero();  // [x, y, theta]
    Mat3 P = Mat3::Identity() * 1e-3;

    // measurement matrix H and noise R (scalar here)
    Row3 H;  H << 0, 0, 1;
    const double R = R_yaw * R_yaw;

    std::string line;
    std::cout << "t,x,y,theta_deg,Pxx,Pyy,Ptt\n";

    while (std::getline(std::cin, line))
    {
        if (line.empty() || line[0]=='#') continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);

        double t, dS, dTh_odom_deg, yaw_imu_deg;
        if (!(ss >> t >> dS >> dTh_odom_deg >> yaw_imu_deg)) continue;

        const double dTh = dTh_odom_deg * DEG2RAD;
        const double yaw_meas = yaw_imu_deg * DEG2RAD;

        // ──────────────────────── PREDICTION STEP ────────────────────────────
        const double cosTh = std::cos(x(2));
        const double sinTh = std::sin(x(2));

        Vec3 f;                    // predicted motion increment
        f << dS * cosTh,
             dS * sinTh,
             dTh;
        x += f;

        // Jacobian F = ∂f/∂x  (identity + small perturbations)
        Mat3 F = Mat3::Identity();
        F(0,2) = -dS * sinTh;
        F(1,2) =  dS * cosTh;

        P = F * P * F.transpose() + Q;

        // ──────────────────────── UPDATE  STEP ───────────────────────────────
        double y_err = yaw_meas - x(2);                 // innovation
        // normalise angle to (-π, π]
        y_err = std::atan2(std::sin(y_err), std::cos(y_err));

        double S = H * P * H.transpose() + R;
        Eigen::Vector3d K = (P * H.transpose()) / S;    // 3×1 Kalman gain

        x += K * y_err;
        P = (Mat3::Identity() - K * H) * P;

        // fix theta to (-π, π] for readability
        x(2) = std::atan2(std::sin(x(2)), std::cos(x(2)));

        // ──────────────────────── OUTPUT ────────────────────────────────────
        std::cout << t << ','
                  << x(0) << ',' << x(1) << ',' << x(2)*RAD2DEG << ','
                  << P(0,0) << ',' << P(1,1) << ',' << P(2,2) << '\n';
    }
    return 0;
}
