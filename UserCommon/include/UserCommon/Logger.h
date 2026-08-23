#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <Common/Types.h>


namespace user_common_330371063_324976703 {
using namespace common;



/**
 * @brief A lightweight streaming logger for simulation visualizations.
 * Outputs to a JSON Lines (.jsonl) file directly to avoid large memory overhead.
 */
class Logger {
public:
    static void init(const std::string& path);
    static void close();
    static void setStep(std::size_t step_id);
    
    static void logMovement(double x_cm, double y_cm, double z_cm, 
                            double h_angle_deg, double v_angle_deg, 
                            const std::string& action);
                            
    static void logScan(double relative_h_angle_deg, double relative_v_angle_deg);
    
    static void logVoxel(int x, int y, int z, int value);

private:
    static std::ofstream out_stream_;
    static std::size_t current_step_id_;
    static std::mutex mutex_;
};



} // namespace user_common_330371063_324976703
