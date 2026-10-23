#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "structs.h"

/**
 * @brief Class responsible for parsing logs. It assembles LiDAR packets in full scans, and matches them with the odometry.
 */
class PacketAssembler
{
public:
    /**
     * @brief Construct a new Packet Assembler and automatically triggers processing.
     * @param path Path to the log data file.
     */
    PacketAssembler(const std::string &path);
    PacketAssembler() {};

    // Getters

    /**
     * @brief Read-write reference to compiled frames for all sensors.
     * @return Multi-dimensional array where indices represent `[lidar_id][frame_idx]`.
     */
    inline std::vector<std::vector<Frame>> &getAllFrames() { return _all_frames; }

    /**
     * @brief Read-only reference to compiled frames for all sensors.
     * @return Multi-dimensional array where indices represent `[lidar_id][frame_idx]`.
     */
    inline const std::vector<std::vector<Frame>> &getAllFrames() const { return _all_frames; }

    /**
     * @brief Read-write reference to retrieved relative lidar poses r_T_l.
     * @return Array of Isometry transformations for each mounted lidar.
     */
    inline std::vector<Eigen::Isometry3f> &getAllT() { return _all_T; }

    /**
     * @brief Read-only reference  to retrieved relative lidar poses r_T_l.
     * @return Array of Isometry transformations for each mounted lidar.
     */
    inline const std::vector<Eigen::Isometry3f> &getAllT() const { return _all_T; }

    /**
     * @brief Read-write reference to the raw log dataset elements.
     * @return Reference to the parsed structured database log mapping object.
     */
    inline LogData &getLogData() { return _data; }

    /**
     * @brief Accesses the raw log dataset elements (immutable context).
     * @return Read-only reference  to the parsed structured database log mapping object.
     */
    inline const LogData &getLogData() const { return _data; }

    /**
     * @brief Accesses the loaded unicycle model configuration parameters.
     * @return UnicycleConfig struct containing the loaded calibration parameters.
     */

    inline const UnicycleConfig &getUnicycleConfig() const { return _data.unicycle_config; }

    /**
     * @brief Assemble consecutive LiDAR packets into a full 360° scan.
     * @param pkts Vector of lidar packets.
     * @param lidar_idx Lindar index for which we need to assemble the packets.
     * @return Vector of Scans in chronological order.
     */
    std::vector<Scan> assembleScans(const std::vector<LidarPacket> &pkts, uint32_t lidar_idx) const;

    /**
     * @brief Finds the closest temporal odometry entry matching a target timestamp.
     * @param odom Robot odometry entry.
     * @param ts Target timestamp.
     * @return Temporal closest matching odometry record.
     */
    OdomEntry nearestOdom(const std::vector<OdomEntry> &odom, uint64_t ts) const;

    /**
     * @brief Finds the closest temporal servo entry matching a target timestamp.
     * @param servo Robot servo entry.
     * @param ts Target timestamp.
     * @return Temporal closest matching servo record.
     */
    ServoEntry nearestServo(const std::vector<ServoEntry> &servo, uint64_t ts) const;

private:
    // Private Processing Implementations

    /**
     * @brief Launches the assembly process.
     */
    void assemble();

    /**
     * @brief Parses the text input file and fills all the structs.
     * @return The full data structure containing parsed log elements.
     */
    LogData parseLog();

    /**
     * @brief Extracts lidar relative pose data from file.
     * @param line Raw input text configuration line.
     * @return LidarConfig struct.
     */
    LidarConfig parseLidarConfigLines(const std::string &line);

    /**
     * @brief Extracts unicycle model initial configuration parameters from file.
     * @param line Raw input text configuration line.
     * @return UnicycleConfig struct.
     */
    UnicycleConfig parseUnicycleConfigLines(const std::string &line) const;

    /**
     * @brief Assembles all frames from data. A frame will have Timestamp matching scans and servo entries.
     * @param data Full dataset coming from LogData()
     * @param lidar_idx Target index of the lidar of interest.
     * @return Vector array containing paired data.
     */
    std::vector<Frame> assembleFrames(const LogData &data, uint32_t lidar_idx);

private:
    // Private Member Variables
    std::string _path;
    LogData _data;                               ///< Full parsed data from LogData().
    std::vector<std::vector<Frame>> _all_frames; ///< Vector containing the vector of all frames for each lidar (i.e. [[frames lidar 0],[frames lidar 1],...[frames lidar n]])
    std::vector<Eigen::Isometry3f> _all_T;       ///< The relative transform of each LiDAR with respect to the robot.
};