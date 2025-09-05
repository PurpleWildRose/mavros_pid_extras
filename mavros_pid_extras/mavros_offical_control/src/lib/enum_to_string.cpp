#include <mavros/enum_to_string.h>
#include <mavros/utils.h>

namespace mavros{
namespace utils{
using mavlink::minimal::MAV_AUTOPILOT;
using mavlink::minimal::MAV_TYPE;
using mavlink::minimal::MAV_STATE;
using mavlink::minimal::MAV_COMPONENT;
using mavlink::common::MAV_ESTIMATOR_TYPE;
using mavlink::common::ADSB_ALTITUDE_TYPE;
using mavlink::common::ADSB_EMITTER_TYPE;
using mavlink::common::GPS_FIX_TYPE;
using mavlink::common::MAV_MISSION_RESULT;
using mavlink::common::MAV_FRAME;
using mavlink::common::MAV_DISTANCE_SENSOR;
using mavlink::common::LANDING_TARGET_TYPE;

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_AUTOPILOT 枚举的字符串
 * @return std::string
 */
std::string to_string(MAV_AUTOPILOT e){
    size_t idx = enum_value(e);
    if(idx >= mav_ardupilot_strings.size()){
        return std::to_string(idx);
    }

    return mav_ardupilot_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_TYPE 枚举的字符串
 * @return std::string
 */
std::string to(MAV_TYPE e) {
    size_t idx = enum_value(e);
    if(idx >= mav_state_strings.size()){
        return std::to_string(idx);
    }

    return mav_state_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 ADSB_ALTITUDE_TYPE 枚举的字符串
 * @return std::string
 */
std::string to(ADSB_ALTITUDE_TYPE e) {
    size_t idx = enum_value(e);
    if(idx >= mav_attitude_type_strings.size()){
        return std::to_string(idx);
    }

    return mav_attitude_type_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 ADSB_EMITTER_TYPE 枚举的字符串
 * @return std::string
 */
std::string to(ADSB_EMITTER_TYPE e) {
    size_t idx = enum_value(e);
    if(idx >= mav_emitting_type_strings.size()){
        return std::to_string(idx);
    }

    return mav_emitting_type_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_ESTIMATOR_TYPE 枚举的字符串
 * @return std::string
 */
std::string to(MAV_ESTIMATOR_TYPE e) {
    size_t idx = enum_value(e);
    if(idx >= mav_estimator_type_strings.size()){
        return std::to_string(idx);
    }

    return mav_estimator_type_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 GPS_FIX_TYPE 枚举的字符串
 * @return std::string
 */
std::string to(GPS_FIX_TYPE e) {
    size_t idx = enum_value(e);
    if(idx >= gps_fix_type_strings.size()){
        return std::to_string(idx);
    }

    return gps_fix_type_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_MISSION_RESULT 枚举的字符串
 * @return std::string
 */
std::string to(MAV_MISSION_RESULT e) {
    size_t idx = enum_value(e);
    if(idx >= mav_mission_result_strings.size()){
        return std::to_string(idx);
    }

    return mav_mission_result_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_DISTANCE_SENSOR 枚举的字符串
 * @return std::string
 */
std::string to(MAV_DISTANCE_SENSOR e) {
    size_t idx = enum_value(e);
    if(idx >= mav_distance_sensor_strings.size()){
        return std::to_string(idx);
    }

    return mav_distance_sensor_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 LANDING_TARGET_TYPE 枚举的字符串
 * @return std::string
 */
std::string to(LANDING_TARGET_TYPE e) {
    size_t idx = enum_value(e);
    if(idx >= landing_target_type_strings.size()){
        return std::to_string(idx);
    }

    return landing_target_type_strings[idx];
}

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_COMPONENT 枚举的字符串
 * @return std::string
 */
std::string to(MAV_COMPONENT e) {
    auto idx = enum_value(e);
    auto it = mav_comp_id_strings.find(idx);

    if(it == mav_comp_id_strings.end()){
        return std::to_string(idx);
    }

    return it->second;
}

/***********************************************convert: mavlink<--->std::string**************************************************** */
/*            

/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_TYPE 枚举的字符串
 * @return std::string
 */
std::string to_string(MAV_TYPE e) {
    size_t idx = enum_value(e);

    if(idx >= mav_type_strings.size()){
        return std::to_string(idx);
    }

    return mav_type_strings[idx];
}


/**
 * mav_type_from_str
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_TYPE 枚举value
 * @return std::string
 */
MAV_TYPE mav_type_from_str(const std::string &str){
    for(size_t idx = 0; idx < mav_type_strings.size(); idx++){
        if(str == mav_type_strings[idx]){
            std::underlying_type<MAV_TYPE>::type rv = idx;
            return static_cast<MAV_TYPE>(rv);
        }
    }

    ROS_ERROR_STREAM_NAMED("uas", "TYPE: Unknown LANDING_TARGET_TYPE: " << str << ". Defaulting to LIGHT_BEACON");
	return MAV_TYPE::GENERIC;
}


/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 LANDING_TARGET_TYPE 枚举的字符串
 * @return std::string
 */
std::string to_string(LANDING_TARGET_TYPE e) {
    size_t idx = enum_value(e);

    if(idx >= landing_target_type_strings.size()){
        return std::to_string(idx);
    }

    return landing_target_type_strings[idx];
}


/**
 * landing_target_type_from_str
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 LANDING_TARGET_TYPE 枚举value
 * @return std::string
 */
LANDING_TARGET_TYPE landing_target_type_from_str(const std::string &str){
    for(size_t idx = 0; idx < landing_target_type_strings.size(); idx++){
        if(str == landing_target_type_strings[idx]){
            std::underlying_type<LANDING_TARGET_TYPE>::type rv = idx;
            return static_cast<LANDING_TARGET_TYPE>(rv);
        }
    }

    ROS_ERROR_STREAM_NAMED("uas", "TYPE: Unknown LANDING_TARGET_TYPE: " << str << ". Defaulting to LIGHT_BEACON");
	return LANDING_TARGET_TYPE::LIGHT_BEACON;
}


/**
 * to_string
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_FRAME 枚举的字符串
 * @return std::string
 */
std::string to_string(MAV_FRAME e) {
    size_t idx = enum_value(e);

    if(idx >= mav_frame_strings.size()){
        return std::to_string(idx);
    }

    return mav_frame_strings[idx];
}


/**
 * mav_frame_from_str
 * @brief [User Define] Retrieve alias of the orientation received by MAVLink msg.
 * 	                    要获取 MAV_FRAME 枚举value
 * @return std::string
 */
MAV_FRAME mav_frame_from_str(const std::string &str){
    for(size_t idx = 0; idx < mav_frame_strings.size(); idx++){
        if(str == mav_frame_strings[idx]){
            std::underlying_type<MAV_FRAME>::type rv = idx;
            return static_cast<MAV_FRAME>(rv);
        }
    }

    ROS_ERROR_STREAM_NAMED("uas", "TYPE: Unknown LANDING_TARGET_TYPE: " << str << ". Defaulting to LIGHT_BEACON");
	return MAV_FRAME::LOCAL_NED;
}


}
}