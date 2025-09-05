#include <mavros/utils.h>

namespace mavros{
namespace utils{

/**
 * enum_value(function)
 * @brief Helper to get enum value from strongly typed enum (enum class).
 * 		通用模板函数 enum_value，核心作用是将任意枚举类型（包括强类型枚举 enum class）的值，安全转换为其对应的底层整数类型（如 int、uint8_t 等）
 */
timesync_mode timesync_mode_from_str(const std::string &mode){
    for(size_t idx = 0; idx < timesync_mode_strings.size(); idx++){
        if(timesync_mode_strings[idx] == mode){
            std::underlying_type<timesync_mode>::type rv = idx;
            return static_cast<timesync_mode> (rv);
        }
    }

    ROS_ERROR_STREAM_NAMED("uas", "TM: Unknown mode: " << mode);
	return timesync_mode::NONE;
}

/**
 * to_string
 * @brief [User Define] Get string repr for timesync_mode
 * 		通过 timesync_mode 获取 std::string 枚举的字符串
 */
std::string to_string(timesync_mode e){
    size_t idx = enum_value(e);
    if(idx >= timesync_mode_strings.size()){
        return std::to_string(idx);
    }
    
    return timesync_mode_strings[idx];
}

}
}
    
