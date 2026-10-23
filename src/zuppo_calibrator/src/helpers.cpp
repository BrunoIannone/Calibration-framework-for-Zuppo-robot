#include "helpers.h"
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

void saveCalibrationToLogFormat(const std::string &filename, const Zuppo &zuppo)
{
    std::ofstream out(filename);
    if (!out)
    {
        std::cerr << "Error: Could not open " << filename << " for writing calibration log.\n";
        return;
    }

    out << std::defaultfloat
        << std::setprecision(std::numeric_limits<float>::max_digits10);

    // 1. Save Unicycle Drive Parameters
    out << "# CONFIG DRIVE " << 0 << " "
        << "k_left:" << zuppo._kl << " "
        << "k_right:" << zuppo._kr << " "
        << "baseline:" << zuppo._wb << "\n";

    for (size_t lidar_idx = 0; lidar_idx < zuppo._lidar_vec.size(); lidar_idx++)
    {
        const auto &r_X_l = zuppo._lidar_vec[lidar_idx]._r_X_l;
        // 2. Save LIDAR Extrinsics Parameters
        float tx = r_X_l.translation().x();
        float ty = r_X_l.translation().y();
        float tz = r_X_l.translation().z();

        Eigen::Quaternionf q(r_X_l.linear());
        q.normalize();

        // double theta = Eigen::Rotation2Dd(zuppo._lidar_vec[lidar_idx]._r_X_l.linear()).angle();
        // double qz = std::sin(theta / 2.0);
        // double qw = std::cos(theta / 2.0);

        out << "# CONFIG LIDAR " << lidar_idx << " "
            << "tx:" << tx << " "
            << "ty:" << ty << " "
            << "tz:" << tz << " "
            << "qx:" << q.x() << " "
            << "qy:" << q.y() << " "
            << "qz:" << q.z() << " "
            << "qw:" << q.w() << "\n";
    }
    std::cout << GREEN << "Calibration successfully saved: " << RESET << filename << std::endl;
}

Eigen::Vector2f parseOriginFromYaml(const std::string &input_path)
{
    std::filesystem::path path(input_path);
    path.replace_extension(".yaml");

    YAML::Node config = YAML::LoadFile(path.string()); // throws if file missing/invalid

    if (!config["origin"] || !config["origin"].IsSequence() || config["origin"].size() < 2)
    {
        throw std::runtime_error("Key 'origin' with valid [x, y, z] array not found in: " + path.string());
    }

    float x = config["origin"][0].as<float>();
    float y = config["origin"][1].as<float>();
    return Eigen::Vector2f(x, y);
}

Eigen::Isometry2f odomAt(const std::map<uint64_t, Eigen::Isometry2f> &buf, const uint64_t t)
{
    if (buf.empty())
        return Eigen::Isometry2f::Identity();
    auto it = buf.lower_bound(t);
    if (it == buf.end())
        return std::prev(it)->second;
    if (it == buf.begin())
        return it->second;
    auto next = it, prev = std::prev(it);
    float alpha = (float)(t - prev->first) / (float)(next->first - prev->first);
    Eigen::Isometry2f T;
    float a0 = Eigen::Rotation2Df(prev->second.linear()).angle();
    float a1 = Eigen::Rotation2Df(next->second.linear()).angle();
    T.linear() = Eigen::Rotation2Df((1.f - alpha) * a0 + alpha * a1).toRotationMatrix();
    T.translation() = (1.f - alpha) * prev->second.translation() + alpha * next->second.translation();
    return T;
}
