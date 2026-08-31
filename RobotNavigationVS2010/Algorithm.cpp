#include "stdafx.h"
#include "Algorithm.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

Point2D::Point2D() : x(0.0), y(0.0)
{
}

Point2D::Point2D(double px, double py) : x(px), y(py)
{
}

Pose2D::Pose2D() : x(0.0), y(0.0), theta(0.0)
{
}

Pose2D::Pose2D(double px, double py, double ptheta)
    : x(px), y(py), theta(ptheta)
{
}

Obstacle::Obstacle() : x(0.0), y(0.0), radius(1.0)
{
}

Obstacle::Obstacle(double px, double py, double pradius)
    : x(px), y(py), radius(pradius)
{
}

SensorConfig::SensorConfig()
    : gpsFrequency(5.0),
      gpsNoiseSigma(0.8),
      insFrequency(100.0),
      insVelocitySigma(0.04),
      lidarBeams(180),
      lidarMaxRange(20.0),
      lidarNoiseProb(0.01),
      lidarRangeSigma(0.03)
{
}

GPSData::GPSData() : x(0.0), y(0.0), valid(false), fresh(false)
{
}

INSData::INSData() : pose(), velocity(0.0), yawRate(0.0)
{
}

LidarScan::LidarScan() : minRange(0.0)
{
}

ControlCommand::ControlCommand() : linear(0.0), angular(0.0)
{
}

ControlCommand::ControlCommand(double v, double w) : linear(v), angular(w)
{
}

WheelCommand::WheelCommand()
    : leftVelocity(0.0), rightVelocity(0.0),
      leftPercent(0.0), rightPercent(0.0)
{
}

double ClampValue(double value, double low, double high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

double NormalizeAngle(double angle)
{
    while (angle > RN_PI)
        angle -= 2.0 * RN_PI;
    while (angle < -RN_PI)
        angle += 2.0 * RN_PI;
    return angle;
}

double Distance2D(double x1, double y1, double x2, double y2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

SensorSimulator::SensorSimulator()
    : m_config(),
      m_gpsClock(0.0),
      m_insClock(0.0),
      m_insVelocityBias(0.0),
      m_insYawBias(0.0),
      m_lastINS(),
      m_lastGPS()
{
}

void SensorSimulator::SetConfig(const SensorConfig& config)
{
    m_config = config;
}

void SensorSimulator::Reset(const Pose2D& initialPose)
{
    m_gpsClock = 0.0;
    m_insClock = 0.0;
    m_insVelocityBias = 0.0;
    m_insYawBias = 0.0;
    m_lastINS = INSData();
    m_lastINS.pose = initialPose;
    m_lastGPS = GPSData();
    m_lastGPS.x = initialPose.x;
    m_lastGPS.y = initialPose.y;
}

double SensorSimulator::Uniform01() const
{
    return (static_cast<double>(std::rand()) + 1.0) /
           (static_cast<double>(RAND_MAX) + 2.0);
}

double SensorSimulator::Gaussian(double sigma) const
{
    const double u1 = Uniform01();
    const double u2 = Uniform01();
    const double standardNormal =
        std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * RN_PI * u2);
    return sigma * standardNormal;
}

void SensorSimulator::Step(double dt,
                           const Pose2D& truePose,
                           double trueVelocity,
                           double trueYawRate,
                           GPSData& gps,
                           INSData& ins)
{
    m_gpsClock += dt;
    m_insClock += dt;
    m_lastGPS.fresh = false;

    const double gpsPeriod = 1.0 / std::max(0.1, m_config.gpsFrequency);
    if (m_gpsClock >= gpsPeriod)
    {
        while (m_gpsClock >= gpsPeriod)
            m_gpsClock -= gpsPeriod;

        double gpsNoiseX = Gaussian(m_config.gpsNoiseSigma);
        double gpsNoiseY = Gaussian(m_config.gpsNoiseSigma);

        // 用约 1% 的概率制造明显异常值，用于展示异常值剔除。
        if (Uniform01() < 0.01)
        {
            gpsNoiseX += Gaussian(8.0 * m_config.gpsNoiseSigma);
            gpsNoiseY += Gaussian(8.0 * m_config.gpsNoiseSigma);
        }

        m_lastGPS.x = truePose.x + gpsNoiseX;
        m_lastGPS.y = truePose.y + gpsNoiseY;
        m_lastGPS.valid = true;
        m_lastGPS.fresh = true;
    }

    const double insPeriod = 1.0 / std::max(1.0, m_config.insFrequency);
    while (m_insClock >= insPeriod)
    {
        m_insClock -= insPeriod;

        // 随机游走模拟零偏随时间缓慢变化。
        m_insVelocityBias += Gaussian(0.002) * std::sqrt(insPeriod);
        m_insYawBias += Gaussian(0.001) * std::sqrt(insPeriod);

        m_lastINS.velocity = trueVelocity + m_insVelocityBias +
                             Gaussian(m_config.insVelocitySigma);
        m_lastINS.yawRate = trueYawRate + m_insYawBias +
                            Gaussian(0.5 * m_config.insVelocitySigma);

        m_lastINS.pose.theta = NormalizeAngle(
            m_lastINS.pose.theta + m_lastINS.yawRate * insPeriod);
        m_lastINS.pose.x += m_lastINS.velocity *
                            std::cos(m_lastINS.pose.theta) * insPeriod;
        m_lastINS.pose.y += m_lastINS.velocity *
                            std::sin(m_lastINS.pose.theta) * insPeriod;
    }

    gps = m_lastGPS;
    ins = m_lastINS;
}

double SensorSimulator::RayCircleDistance(double ox, double oy,
                                          double dx, double dy,
                                          const Obstacle& obstacle) const
{
    const double fx = ox - obstacle.x;
    const double fy = oy - obstacle.y;
    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - obstacle.radius * obstacle.radius;
    const double discriminant = b * b - 4.0 * c;

    if (discriminant < 0.0)
        return -1.0;

    const double root = std::sqrt(discriminant);
    const double t1 = (-b - root) / 2.0;
    const double t2 = (-b + root) / 2.0;

    if (t1 >= 0.0)
        return t1;
    if (t2 >= 0.0)
        return t2;
    return -1.0;
}

double SensorSimulator::RayWallDistance(double ox, double oy,
                                        double dx, double dy,
                                        double mapWidth,
                                        double mapHeight) const
{
    double nearest = 1.0e30;
    double t = 0.0;
    double hit = 0.0;

    if (std::fabs(dx) > 1.0e-9)
    {
        t = (0.0 - ox) / dx;
        hit = oy + t * dy;
        if (t >= 0.0 && hit >= 0.0 && hit <= mapHeight)
            nearest = std::min(nearest, t);

        t = (mapWidth - ox) / dx;
        hit = oy + t * dy;
        if (t >= 0.0 && hit >= 0.0 && hit <= mapHeight)
            nearest = std::min(nearest, t);
    }

    if (std::fabs(dy) > 1.0e-9)
    {
        t = (0.0 - oy) / dy;
        hit = ox + t * dx;
        if (t >= 0.0 && hit >= 0.0 && hit <= mapWidth)
            nearest = std::min(nearest, t);

        t = (mapHeight - oy) / dy;
        hit = ox + t * dx;
        if (t >= 0.0 && hit >= 0.0 && hit <= mapWidth)
            nearest = std::min(nearest, t);
    }

    return nearest;
}

LidarScan SensorSimulator::MakeLidarScan(
    const Pose2D& pose,
    const std::vector<Obstacle>& obstacles,
    double mapWidth,
    double mapHeight) const
{
    LidarScan scan;
    const int beams = std::max(8, m_config.lidarBeams);
    scan.angles.resize(beams);
    scan.ranges.resize(beams);
    scan.minRange = m_config.lidarMaxRange;

    for (int i = 0; i < beams; ++i)
    {
        const double relativeAngle = -RN_PI +
            2.0 * RN_PI * static_cast<double>(i) /
            static_cast<double>(beams);
        const double worldAngle = pose.theta + relativeAngle;
        const double dx = std::cos(worldAngle);
        const double dy = std::sin(worldAngle);

        double range = RayWallDistance(pose.x, pose.y, dx, dy,
                                       mapWidth, mapHeight);
        for (size_t k = 0; k < obstacles.size(); ++k)
        {
            const double candidate = RayCircleDistance(
                pose.x, pose.y, dx, dy, obstacles[k]);
            if (candidate >= 0.0)
                range = std::min(range, candidate);
        }

        range = std::min(range, m_config.lidarMaxRange);
        range += Gaussian(m_config.lidarRangeSigma);

        // 噪点被模拟为随机出现的一次虚假近距离回波。
        if (Uniform01() < m_config.lidarNoiseProb)
            range = Uniform01() * m_config.lidarMaxRange;

        range = ClampValue(range, 0.05, m_config.lidarMaxRange);
        scan.angles[i] = relativeAngle;
        scan.ranges[i] = range;
        scan.minRange = std::min(scan.minRange, range);
    }

    return scan;
}

FusionEngine::FusionEngine()
    : m_state(), m_initialized(false)
{
    m_p[0] = 1.0;
    m_p[1] = 1.0;
    m_p[2] = 0.2;
}

void FusionEngine::Reset(const Pose2D& initialPose)
{
    m_state = initialPose;
    m_p[0] = 1.0;
    m_p[1] = 1.0;
    m_p[2] = 0.2;
    m_initialized = true;
}

Pose2D FusionEngine::WeightedFusion(const GPSData& gps,
                                    const INSData& ins,
                                    double gpsWeight) const
{
    if (!gps.valid)
        return ins.pose;

    double weight = ClampValue(gpsWeight, 0.0, 1.0);
    const double disagreement = Distance2D(gps.x, gps.y,
                                           ins.pose.x, ins.pose.y);

    // GPS 与 INS 相差 8 m 以上时，把 GPS 视为明显异常值。
    if (disagreement > 8.0)
        weight = 0.0;

    Pose2D result;
    result.x = weight * gps.x + (1.0 - weight) * ins.pose.x;
    result.y = weight * gps.y + (1.0 - weight) * ins.pose.y;
    result.theta = ins.pose.theta;
    return result;
}

Pose2D FusionEngine::EKFStep(double dt,
                             const GPSData& gps,
                             const INSData& ins,
                             double processNoiseQ,
                             double gpsNoiseR)
{
    if (!m_initialized)
        Reset(ins.pose);

    const double q = std::max(1.0e-8, processNoiseQ);
    const double r = std::max(1.0e-8, gpsNoiseR);

    // 预测：用 INS 测得的速度和角速度推进机器人状态。
    m_state.theta = NormalizeAngle(m_state.theta + ins.yawRate * dt);
    m_state.x += ins.velocity * std::cos(m_state.theta) * dt;
    m_state.y += ins.velocity * std::sin(m_state.theta) * dt;

    // 简化为对角协方差，便于课程设计中观察 Q、R 的影响。
    m_p[0] += q * dt;
    m_p[1] += q * dt;
    m_p[2] += 0.2 * q * dt;

    if (gps.valid && gps.fresh)
    {
        const double residualX = gps.x - m_state.x;
        const double residualY = gps.y - m_state.y;

        // 创新过大时拒绝本次 GPS，从而抑制离群点。
        if (std::sqrt(residualX * residualX + residualY * residualY) < 8.0)
        {
            const double gainX = m_p[0] / (m_p[0] + r);
            const double gainY = m_p[1] / (m_p[1] + r);
            m_state.x += gainX * residualX;
            m_state.y += gainY * residualY;
            m_p[0] *= (1.0 - gainX);
            m_p[1] *= (1.0 - gainY);
        }
    }

    // INS 航向用于缓慢校正预测航向。
    const double headingR = std::max(0.01, 0.25 * r);
    const double headingGain = m_p[2] / (m_p[2] + headingR);
    const double headingResidual = NormalizeAngle(ins.pose.theta - m_state.theta);
    m_state.theta = NormalizeAngle(m_state.theta +
                                   headingGain * headingResidual);
    m_p[2] *= (1.0 - headingGain);

    return m_state;
}

AvoidancePlanner::AvoidancePlanner()
{
}

ControlCommand AvoidancePlanner::ComputeAPF(
    const Pose2D& pose,
    const Point2D& goal,
    const LidarScan& scan,
    double maxLinear,
    double maxAngular,
    double safetyDistance,
    double repulsiveGain) const
{
    const double goalDx = goal.x - pose.x;
    const double goalDy = goal.y - pose.y;
    const double goalDistance = std::sqrt(goalDx * goalDx + goalDy * goalDy);

    if (goalDistance < 0.5)
        return ControlCommand(0.0, 0.0);

    // 单位吸引力向量指向终点。
    double forceX = goalDx / std::max(0.001, goalDistance);
    double forceY = goalDy / std::max(0.001, goalDistance);
    const double influenceDistance = std::max(1.2, 2.5 * safetyDistance);

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double range = scan.ranges[i];
        if (range >= influenceDistance)
            continue;

        const double worldAngle = pose.theta + scan.angles[i];
        const double safeRange = std::max(0.15, range);
        double magnitude = repulsiveGain *
            (1.0 / safeRange - 1.0 / influenceDistance) /
            (safeRange * safeRange);

        // 多射线累计时除以射线数，避免增加雷达点数改变算法量级。
        magnitude /= std::max(1.0, static_cast<double>(scan.ranges.size()) / 90.0);
        forceX -= magnitude * std::cos(worldAngle);
        forceY -= magnitude * std::sin(worldAngle);
    }

    const double desiredHeading = std::atan2(forceY, forceX);
    const double headingError = NormalizeAngle(desiredHeading - pose.theta);

    double linear = maxLinear * std::min(1.0, goalDistance / 5.0);
    linear *= std::max(0.0, std::cos(headingError));

    if (scan.minRange < safetyDistance)
        linear *= ClampValue(scan.minRange / safetyDistance, 0.0, 1.0);

    const double angular = ClampValue(2.2 * headingError,
                                      -maxAngular, maxAngular);
    return ControlCommand(ClampValue(linear, 0.0, maxLinear), angular);
}

double AvoidancePlanner::ClearanceAt(
    double x, double y,
    const std::vector<Obstacle>& obstacles,
    double mapWidth, double mapHeight) const
{
    double clearance = std::min(std::min(x, mapWidth - x),
                                std::min(y, mapHeight - y));

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        const double obstacleClearance =
            Distance2D(x, y, obstacles[i].x, obstacles[i].y) -
            obstacles[i].radius;
        clearance = std::min(clearance, obstacleClearance);
    }
    return clearance;
}

ControlCommand AvoidancePlanner::ComputeDWA(
    const Pose2D& pose,
    const Point2D& goal,
    const std::vector<Obstacle>& obstacles,
    double mapWidth,
    double mapHeight,
    double maxLinear,
    double maxAngular,
    double safetyDistance) const
{
    if (Distance2D(pose.x, pose.y, goal.x, goal.y) < 0.5)
        return ControlCommand(0.0, 0.0);

    const double robotRadius = 0.45;
    double bestScore = -1.0e30;
    ControlCommand best(0.0, 0.0);

    // 枚举一组候选线速度和角速度。
    for (int vi = 0; vi <= 6; ++vi)
    {
        const double v = maxLinear * static_cast<double>(vi) / 6.0;
        for (int wi = -7; wi <= 7; ++wi)
        {
            const double w = maxAngular * static_cast<double>(wi) / 7.0;
            Pose2D predicted = pose;
            double minimumClearance = 1.0e30;
            bool collision = false;

            // 向前滚动预测 2 秒。
            for (int step = 0; step < 20; ++step)
            {
                const double predictDt = 0.1;
                predicted.theta = NormalizeAngle(predicted.theta + w * predictDt);
                predicted.x += v * std::cos(predicted.theta) * predictDt;
                predicted.y += v * std::sin(predicted.theta) * predictDt;

                const double clearance = ClearanceAt(
                    predicted.x, predicted.y, obstacles,
                    mapWidth, mapHeight) - robotRadius;
                minimumClearance = std::min(minimumClearance, clearance);

                if (clearance < safetyDistance)
                {
                    collision = true;
                    break;
                }
            }

            if (collision)
                continue;

            const double goalDistance = Distance2D(
                predicted.x, predicted.y, goal.x, goal.y);
            const double goalHeading = std::atan2(goal.y - predicted.y,
                                                  goal.x - predicted.x);
            const double headingError = std::fabs(
                NormalizeAngle(goalHeading - predicted.theta));

            // 距终点近、朝向终点、间隙大、速度高的轨迹得分更高。
            const double score = -1.0 * goalDistance
                               - 2.0 * headingError
                               + 0.8 * std::min(minimumClearance, 5.0)
                               + 1.2 * v;

            if (score > bestScore)
            {
                bestScore = score;
                best.linear = v;
                best.angular = w;
            }
        }
    }

    // 所有运动轨迹均危险时原地转向终点。
    if (bestScore < -1.0e20)
    {
        const double goalHeading = std::atan2(goal.y - pose.y,
                                              goal.x - pose.x);
        best.angular = ClampValue(
            1.5 * NormalizeAngle(goalHeading - pose.theta),
            -maxAngular, maxAngular);
    }

    return best;
}

PIDController::PIDController()
    : m_kp(1.2), m_ki(0.05), m_kd(0.08),
      m_integral(0.0), m_lastError(0.0), m_firstUpdate(true)
{
}

void PIDController::SetGains(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PIDController::Reset()
{
    m_integral = 0.0;
    m_lastError = 0.0;
    m_firstUpdate = true;
}

double PIDController::Update(double target, double actual, double dt)
{
    const double safeDt = std::max(1.0e-6, dt);
    const double error = target - actual;
    m_integral += error * safeDt;
    m_integral = ClampValue(m_integral, -10.0, 10.0);

    double derivative = 0.0;
    if (!m_firstUpdate)
        derivative = (error - m_lastError) / safeDt;

    m_lastError = error;
    m_firstUpdate = false;
    return m_kp * error + m_ki * m_integral + m_kd * derivative;
}

WheelCommand DifferentialDrive(double linearVelocity,
                               double angularVelocity,
                               double wheelTrack,
                               double maximumWheelVelocity)
{
    WheelCommand command;
    command.leftVelocity = linearVelocity -
                           0.5 * wheelTrack * angularVelocity;
    command.rightVelocity = linearVelocity +
                            0.5 * wheelTrack * angularVelocity;

    const double maximum = std::max(0.01, maximumWheelVelocity);
    command.leftPercent = 100.0 *
        ClampValue(command.leftVelocity / maximum, -1.0, 1.0);
    command.rightPercent = 100.0 *
        ClampValue(command.rightVelocity / maximum, -1.0, 1.0);
    return command;
}
