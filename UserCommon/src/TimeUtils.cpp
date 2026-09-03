#include <UserCommon/TimeUtils.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace user_common_330371063_324976703 {

std::string TimeUtils::generate_iso_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return time_ss.str();
}

std::string TimeUtils::generate_folder_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y%m%d%H%M%S");
    return time_ss.str();
}

} // namespace user_common_330371063_324976703
