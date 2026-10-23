#pragma once

#include <string>
#include <map>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sys/time.h>
#include <iostream>
#include "params.h"
#include "voxel.h"
#include "voxel3d.h"

#include "zuppo.h"
/**
 * @brief Get current system time in milliseconds
 * @return Current time in ms
 */
inline double getTimeMs()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    double ret = (double)tv.tv_sec * 1000 + (double)tv.tv_usec * 1e-3;
    return ret;
}

/**
 * @brief Print progress statistics for measurement processing
 * @param idx Current measurement index
 * @param total Total number of measurements
 * @param start_time Reference to timer start timestamp
 * @param is_timer_running Reference to timer status flag
 */
inline void printProgressStats(size_t idx, size_t total, double &start_time, bool &is_timer_running)
{
    if (!is_timer_running)
    {
        start_time = getTimeMs();
        is_timer_running = true;
    }
    if (idx % 100 == 0 && idx > 0)
    {
        const double end_time = getTimeMs();
        is_timer_running = false;
        std::cout << (end_time - start_time) / 1000 << " seconds" << std::endl;
        std::cout << "Processed " << idx << " / " << total << " measurements.\n";
    }
}

/**
 * @brief Clear input stream buffer
 */
inline void clearInput()
{
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

/**
 * @brief Prompt user for a Yes/No input
 * @param prompt Display message
 * @return True if user inputs 'y', false if 'n'
 */
inline bool askYesNo(const std::string &prompt)
{
    char answer;
    while (true)
    {
        std::cout << RED << prompt << RESET << " (y/n): ";
        if (std::cin >> answer)
        {
            clearInput();
            char clean_answer = std::tolower(static_cast<unsigned char>(answer));
            if (clean_answer == 'y')
                return true;
            if (clean_answer == 'n')
                return false;
        }
        else
        {
            std::cin.clear();
            clearInput();
        }
        std::cout << YELLOW << "Invalid input. Please enter 'y' or 'n'.\n"
                  << RESET;
    }
}

/**
 * @brief Save calibration results to a text log file
 * @param filename Target file path
 * @param zuppo Reference to Zuppo object containing parameters
 */
void saveCalibrationToLogFormat(const std::string &filename, const Zuppo &zuppo);

inline void askAndSave(Zuppo &zuppo, const std::string &prompt_msg)
{
    if (askYesNo(prompt_msg))
    {
        saveCalibrationToLogFormat("../calibrated_log_params.txt", zuppo);
    }
}

// /**
//  * @brief Check if a CUDA-compatible GPU is available
//  * @return True if at least one CUDA device is found
//  */
// inline bool hasGPU()
// {
//     int deviceCount = 0;
//     cudaError_t error = cudaGetDeviceCount(&deviceCount);
//     if (error != cudaSuccess)
//     {
//         return false; // Driver not found, no GPU, or CUDA runtime missing
//     }
//     return deviceCount > 0;
// }

/**
 * @brief Parse map origin coordinates from a YAML configuration file
 * @param input_path Path to the configuration file
 * @return 2D vector containing map origin (x, y)
 */
Eigen::Vector2f parseOriginFromYaml(const std::string &input_path);

/**
 * @brief Interpolate 2D odometry pose at a given timestamp
 * @param buf Map of timestamps to 2D Isometries
 * @param t Target timestamp
 * @return Interpolated 2D Isometry pose
 */
Eigen::Isometry2f odomAt(const std::map<uint64_t, Eigen::Isometry2f> &buf, uint64_t t);
