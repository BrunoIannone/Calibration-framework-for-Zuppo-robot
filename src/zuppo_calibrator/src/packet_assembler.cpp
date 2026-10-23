#include "packet_assembler.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

PacketAssembler::PacketAssembler(const std::string &path)
{
    _path = path;
    assemble();
}

LidarConfig PacketAssembler::parseLidarConfigLines(const std::string &line)
{
    LidarConfig cfg;

    std::istringstream ss(line);
    std::string tok;

    while (ss >> tok)
    {
        const auto colon = tok.find(':');
        if (colon == std::string::npos)
            continue;

        const std::string key = tok.substr(0, colon);
        const float val = std::stof(tok.substr(colon + 1));
        if (key == "tx")
            cfg.tx = val;
        else if (key == "ty")
            cfg.ty = val;
        else if (key == "tz")
            cfg.tz = val;
        else if (key == "qx")
            cfg.qx = val;
        else if (key == "qy")
            cfg.qy = val;
        else if (key == "qz")
            cfg.qz = val;
        else if (key == "qw")
            cfg.qw = val;
    }
    return cfg;
}
UnicycleConfig PacketAssembler::parseUnicycleConfigLines(const std::string &line) const
{
    UnicycleConfig ucfg;
    std::istringstream ss(line);
    std::string tok;

    while (ss >> tok)
    {
        const auto colon = tok.find(':');
        if (colon == std::string::npos)
            continue;

        const std::string key = tok.substr(0, colon);
        const float val = std::stof(tok.substr(colon + 1));

        if (key == "k_left")
            ucfg.kl = val;
        else if (key == "k_right")
            ucfg.kr = val;
        else if (key == "baseline")
            ucfg.wb = val;
        else if (key == "tv_max")
            ucfg.tv_max = val;
        else if (key == "rv_max")
            ucfg.rv_max = val;
        else if (key == "ta_max")
            ucfg.ta_max = val;
        else if (key == "ra_max")
            ucfg.ra_max = val;
    }
    return ucfg;
}
LogData PacketAssembler::parseLog()
{
    LogData data;
    std::ifstream f(_path);
    if (!f)
    {
        std::cerr << "PACKET ASSEMBLER: " << RED << "Cannot open " << RESET << _path << std::endl;
        return data;
    }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty())
            continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "#") // Config lines
        {
            std::string cfg_kw, kind;
            ss >> cfg_kw >> kind;

            if (cfg_kw == "CONFIG" && kind == "LIDAR")
            {
                size_t lidar_idx;
                ss >> lidar_idx;
                if (data.lidar_configs.size() <= lidar_idx)
                    data.lidar_configs.resize(lidar_idx + 1);
                data.lidar_configs[lidar_idx] = parseLidarConfigLines(line);
            }
            else if (cfg_kw == "CONFIG" && kind == "DRIVE")
            {
                data.unicycle_config = parseUnicycleConfigLines(line);
            }
        }
        // else if (tag == "ODOM")
        // {
        //     OdomEntry e;
        //     ss >> e.timestamp >> e.x >> e.y >> e.theta >> e.tv >> e.rv;
        //     data.odom.push_back(e);
        // }
        else if (tag == "LIDAR")
        {
            LidarPacket p;
            ss >> p.timestamp >> p.idx >> p.start_angle >> p.end_angle >> p.speed;
            for (int i = 0; i < PACKET_SIZE; ++i)
                ss >> p.points[i].distance >> p.points[i].confidence;
            data.lidar.push_back(p);
        }
        else if (tag == "SERVO")
        {
            ServoEntry e;
            uint16_t servo_id;
            ss >> e.timestamp >> servo_id >> e.ticks[servo_id];

            uint16_t other_id = servo_id;
            std::streampos pos;

            // Skip duplicates of the first servo
            while (true)
            {
                pos = f.tellg();

                if (!std::getline(f, line))
                    throw std::runtime_error("Servo Log terminated unexpectedly");

                ss = std::istringstream(line);
                ss >> tag;
                ss >> e.timestamp >> other_id;

                if (tag != "SERVO")
                {
                    std::cerr << "Unexpected tag: " << tag << " at timestamp: " << e.timestamp << std::endl;
                    throw std::runtime_error("Expected SERVO at timestamp: " + std::to_string(e.timestamp));
                }

                if (other_id != servo_id)
                    break;
            }

            // Store the first occurrence of the other servo
            ss >> e.ticks[other_id];

            // Skip duplicates of the other servo
            while (true)
            {
                pos = f.tellg();

                if (!std::getline(f, line))
                    break; // EOF is OK

                ss = std::istringstream(line);
                ss >> tag;

                if (tag != "SERVO")
                {
                    f.seekg(pos);
                    break;
                }

                uint64_t ts;
                uint16_t id;
                ss >> ts >> id;

                if (id != other_id)
                {
                    f.seekg(pos); // put back for the outer loop
                    break;
                }

                // duplicate -> ignore
            }

            data.servo.push_back(e);
        }
    }
    return data;
}

OdomEntry PacketAssembler::nearestOdom(const std::vector<OdomEntry> &odom, uint64_t ts) const
{
    if (odom.empty())
        return {};

    auto it = std::lower_bound(
        odom.begin(), odom.end(), ts,
        [](const OdomEntry &e, uint64_t t)
        { return e.timestamp < t; });

    if (it == odom.begin())
        return *it;
    if (it == odom.end())
        return *std::prev(it);

    auto prev = std::prev(it);
    const uint64_t diff_next = it->timestamp - ts;
    const uint64_t diff_prev = ts - prev->timestamp;

    return (diff_prev <= diff_next) ? *prev : *it;
}

ServoEntry PacketAssembler::nearestServo(const std::vector<ServoEntry> &servo, uint64_t ts) const
{
    if (servo.empty())
        return {};

    auto it = std::lower_bound(
        servo.begin(), servo.end(), ts,
        [](const ServoEntry &e, uint64_t t)
        { return e.timestamp < t; });

    if (it == servo.begin())
        return *it;
    if (it == servo.end())
        return *std::prev(it);

    auto prev = std::prev(it);
    const uint64_t diff_next = it->timestamp - ts;
    const uint64_t diff_prev = ts - prev->timestamp;

    return (diff_prev <= diff_next) ? *prev : *it;
}
std::vector<Scan> PacketAssembler::assembleScans(const std::vector<LidarPacket> &pkts,
                                                 uint32_t lidar_idx) const
{
    std::vector<Scan> scans;
    Scan current;
    int prev_idx = -1; // index of last accepted packet
    int packet_count = 0;
    for (size_t i = 0; i < pkts.size(); ++i)
    {
        const auto &pkt = pkts[i];
        if (pkt.idx != lidar_idx)
            continue;
        ++packet_count;

        // New scan when start_angle resets below previous end_angle
        bool new_scan = false;
        if (prev_idx >= 0 && pkts[static_cast<size_t>(prev_idx)].start_angle > pkts[static_cast<size_t>(prev_idx)].end_angle)
        {
            new_scan = true;
        }
        prev_idx = static_cast<int>(i);
        if (new_scan)
        {
            if (!current.points.empty())
                scans.push_back(current);
            current.points.clear();
        }
        if (current.points.empty())
            current.timestamp = pkt.timestamp;

        // Angular step between the 12 points (in hundredths of degrees)
        int d_angle = static_cast<int>(pkt.end_angle - pkt.start_angle);
        if (d_angle < 0)
            d_angle += 36000;
        const float step = d_angle / 11.f; // 12 points, 11 intervals

        for (int j = 0; j < PACKET_SIZE; ++j)
        {
            uint16_t dist_mm = pkt.points[j].distance;
            // std::cerr<<dist_mm<<std::endl;
            if (dist_mm < 200 || dist_mm > 12000)
                continue;
            if (pkt.points[j].confidence < 10)
                continue;

            const float angle_hdeg = pkt.start_angle + j * step;
            const float angle_rad = angle_hdeg * (float)M_PI / 18000.f;
            const float d = dist_mm * 1e-3f;
            current.points.push_back({d * std::cos(angle_rad), -d * std::sin(angle_rad)});
        }
        // keep last packet's end_angle reference alive via pkts[i-1] in next iteration
    }
    if (!current.points.empty())
        scans.push_back(current);
    std::cerr << "\n  -LiDAR " << lidar_idx << ": " << packet_count << " packets" << std::endl;
    return scans;
}
std::vector<Frame> PacketAssembler::assembleFrames(const LogData &data, const uint32_t lidar_idx)
{
    const auto scans = PacketAssembler::assembleScans(data.lidar, lidar_idx);
    std::vector<Frame> frames;
    frames.reserve(scans.size());
    for (auto &scan : scans)
    {
        Frame f;
        f.scan = std::move(scan);
        // f.odom = nearestOdom(data.odom, f.scan.timestamp);
        f.servo = nearestServo(data.servo, f.scan.timestamp);
        frames.push_back(std::move(f));
    }
    return frames;
}

void PacketAssembler::assemble()
{
    std::cerr << YELLOW << "Processing log: " << RESET << "[" << _path << "]" << std::endl;
    _data = parseLog();
    std::cerr << GREEN << "Processing complete:\n"
              << RESET << "-Number of odom entries: " << _data.odom.size()
              << "\n-Number of servo entries: " << _data.servo.size() << "\n-Number of LiDAR packets: " << _data.lidar.size()
              << "\n-Number of LiDARs: " << _data.lidar_configs.size() << std::endl;

    size_t num_lidars = _data.lidar_configs.size();
    if (num_lidars == 0)
    {
        std::cerr << "PACKET ASSEMBLER: " << RED << "No LiDAR config found" << RESET << std::endl;
        return;
    }
    // Build frames and extrinsics for each lidar
    _all_frames.resize(num_lidars);
    _all_T.resize(num_lidars);

    for (size_t li = 0; li < num_lidars; ++li)
    {
        _all_frames[li] = assembleFrames(_data, li);
        std::cerr << "  -Lidar " << li << ": " << _all_frames[li].size() << " scans" << std::endl;
        const auto &cfg = _data.lidar_configs[li];
        _all_T[li] = Eigen::Isometry3f::Identity();
        _all_T[li].translation() << cfg.tx, cfg.ty, cfg.tz;
        _all_T[li].linear() = Eigen::Quaternionf(cfg.qw, cfg.qx, cfg.qy, cfg.qz).toRotationMatrix();
    }
}
