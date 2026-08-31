#pragma once

// Algorithm.h
// 这一文件只保存“算法层”的声明，不依赖 MFC。
// 因而既可以被 MFC 界面调用，也可以单独编译测试。

#include <vector>

const double RN_PI = 3.14159265358979323846;

struct Point2D
{
    double x;
    double y;

    Point2D();
    Point2D(double px, double py);
};

struct Pose2D
{
    double x;
    double y;
    double theta;

    Pose2D();
    Pose2D(double px, double py, double ptheta);
};

struct Obstacle
{
    double x;
    double y;
    double radius;

    Obstacle();
    Obstacle(double px, double py, double pradius);
};

struct SensorConfig
{
    double gpsFrequency;       // GPS 刷新频率，建议 1~10 Hz。
    double gpsNoiseSigma;      // GPS 位置噪声标准差，建议 0.2~3.0 m。
    double insFrequency;       // INS 刷新频率，建议 20~200 Hz。
    double insVelocitySigma;   // INS 速度噪声标准差，建议 0.01~0.20 m/s。
    int lidarBeams;            // 2D 雷达一周的采样点数，建议 90~720。
    double lidarMaxRange;      // 雷达最大量程，建议 5~50 m。
    double lidarNoiseProb;     // 噪点概率，取值 0~0.20。
    double lidarRangeSigma;    // 测距噪声标准差。

    SensorConfig();
};

struct GPSData
{
    double x;
    double y;
    bool valid;
    bool fresh;

    GPSData();
};

struct INSData
{
    Pose2D pose;
    double velocity;
    double yawRate;

    INSData();
};

struct LidarScan
{
    std::vector<double> angles; // 相对机器人正前方的角度，单位 rad。
    std::vector<double> ranges; // 对应射线的距离，单位 m。
    double minRange;

    LidarScan();
};

struct ControlCommand
{
    double linear;
    double angular;

    ControlCommand();
    ControlCommand(double v, double w);
};

struct WheelCommand
{
    double leftVelocity;
    double rightVelocity;
    double leftPercent;
    double rightPercent;

    WheelCommand();
};

double ClampValue(double value, double low, double high);
double NormalizeAngle(double angle);
double Distance2D(double x1, double y1, double x2, double y2);

class SensorSimulator
{
public:
    SensorSimulator();

    void SetConfig(const SensorConfig& config);
    void Reset(const Pose2D& initialPose);
    void Step(double dt,
              const Pose2D& truePose,
              double trueVelocity,
              double trueYawRate,
              GPSData& gps,
              INSData& ins);

    LidarScan MakeLidarScan(const Pose2D& pose,
                            const std::vector<Obstacle>& obstacles,
                            double mapWidth,
                            double mapHeight) const;

private:
    double Uniform01() const;
    double Gaussian(double sigma) const;
    double RayCircleDistance(double ox, double oy,
                             double dx, double dy,
                             const Obstacle& obstacle) const;
    double RayWallDistance(double ox, double oy,
                           double dx, double dy,
                           double mapWidth, double mapHeight) const;

private:
    SensorConfig m_config;
    double m_gpsClock;
    double m_insClock;
    double m_insVelocityBias;
    double m_insYawBias;
    INSData m_lastINS;
    GPSData m_lastGPS;
};

class FusionEngine
{
public:
    FusionEngine();

    void Reset(const Pose2D& initialPose);
    Pose2D WeightedFusion(const GPSData& gps,
                          const INSData& ins,
                          double gpsWeight) const;
    Pose2D EKFStep(double dt,
                   const GPSData& gps,
                   const INSData& ins,
                   double processNoiseQ,
                   double gpsNoiseR);

private:
    Pose2D m_state;
    double m_p[3];
    bool m_initialized;
};

class AvoidancePlanner
{
public:
    AvoidancePlanner();

    ControlCommand ComputeAPF(const Pose2D& pose,
                              const Point2D& goal,
                              const LidarScan& scan,
                              double maxLinear,
                              double maxAngular,
                              double safetyDistance,
                              double repulsiveGain) const;

    ControlCommand ComputeDWA(const Pose2D& pose,
                              const Point2D& goal,
                              const std::vector<Obstacle>& obstacles,
                              double mapWidth,
                              double mapHeight,
                              double maxLinear,
                              double maxAngular,
                              double safetyDistance) const;

private:
    double ClearanceAt(double x, double y,
                       const std::vector<Obstacle>& obstacles,
                       double mapWidth, double mapHeight) const;
};

class PIDController
{
public:
    PIDController();

    void SetGains(double kp, double ki, double kd);
    void Reset();
    double Update(double target, double actual, double dt);

private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_integral;
    double m_lastError;
    bool m_firstUpdate;
};

WheelCommand DifferentialDrive(double linearVelocity,
                               double angularVelocity,
                               double wheelTrack,
                               double maximumWheelVelocity);

