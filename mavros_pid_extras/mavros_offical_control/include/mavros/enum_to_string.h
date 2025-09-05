#pragma once

#include <array>
#include <unordered_map>
#include <mavros_proxy/mavlink_dialect.h>
#include <ros/console.h>

namespace mavros{
namespace utils{
/**
 * mav_ardupilot_strings
 * 
 * @brief [USER Define] Micro air vehicle / autopilot classes. This identifies the individual model. 
 *                      用于标识自动驾驶仪（飞控）类型的枚举，用于明确无人机所使用的飞控系统型号或类型（如 PX4、ArduPilot 等）。这一信息有助于地面站、周边设备或其他系统适配对应的功能、协议扩展或交互逻辑。
 * @page mavlink/minimal/minimal.hpp
 * --> MAV_AUTOPILOT
 */
static const std::array<const std::string, 21> mav_ardupilot_strings{{
/*  0 */ "Generic autopilot",             // Generic autopilot, full support for everything
/*  1 */ "Reserved for future use",       // Reserved for future use.
/*  2 */ "SLUGS autopilot",               // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
/*  3 */ "ArduPilot",                     // ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org
/*  4 */ "OpenPilot",                     // OpenPilot, http://openpilot.org
/*  5 */ "Generic autopilot only supporting simple waypoints", // Generic autopilot only supporting simple waypoints
/*  6 */ "Generic autopilot supporting waypoints and other simple navigation commands", // Generic autopilot supporting waypoints and other simple navigation commands
/*  7 */ "Generic autopilot supporting the full mission command set", // Generic autopilot supporting the full mission command set
/*  8 */ "No valid autopilot",            // No valid autopilot, e.g. a GCS or other MAVLink component
/*  9 */ "PPZ UAV",                       // PPZ UAV - http://nongnu.org/paparazzi
/* 10 */ "UAV Dev Board",                 // UAV Dev Board
/* 11 */ "FlexiPilot",                    // FlexiPilot
/* 12 */ "PX4 Autopilot",                 // PX4 Autopilot - http://px4.io/
/* 13 */ "SMACCMPilot",                   // SMACCMPilot - http://smaccmpilot.org
/* 14 */ "AutoQuad",                      // AutoQuad -- http://autoquad.org
/* 15 */ "Armazila",                      // Armazila -- http://armazila.com
/* 16 */ "Aerob",                         // Aerob -- http://aerob.ru
/* 17 */ "ASLUAV autopilot",              // ASLUAV autopilot -- http://www.asl.ethz.ch
/* 18 */ "SmartAP Autopilot",             // SmartAP Autopilot - http://sky-drones.com
/* 19 */ "AirRails",                      // AirRails - http://uaventure.com
/* 20 */ "Fusion Reflex",                 // Fusion Reflex - https://fusion.engineering
}};


/**
 * mav_type_strings
 * @brief [User Define] MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).
 *                      用于标识飞行器 / 设备类型的核心枚举，用于明确 MAVLink 网络中设备的物理类型（如固定翼飞机、多旋翼无人机、地面站等）。它是不同设备之间识别彼此属性的基础，确保通信双方能根据设备类型调整交互逻辑（如对无人机和地面站发送不同类型的指令）。
 * @page mavlink/minimal/minimal.hpp
 * --> MAV_TYPE
 */
static const std::array<const std::string, 43> mav_type_strings{{
/*  0 */ "Generic micro air vehicle",     // Generic micro air vehicle
/*  1 */ "Fixed wing aircraft",           // Fixed wing aircraft.
/*  2 */ "Quadrotor",                     // Quadrotor
/*  3 */ "Coaxial helicopter",            // Coaxial helicopter
/*  4 */ "Normal helicopter with tail rotor", // Normal helicopter with tail rotor.
/*  5 */ "Ground installation",           // Ground installation
/*  6 */ "Operator control unit",         // Operator control unit / ground control station
/*  7 */ "Airship",                       // Airship, controlled
/*  8 */ "Free balloon",                  // Free balloon, uncontrolled
/*  9 */ "Rocket",                        // Rocket
/* 10 */ "Ground rover",                  // Ground rover
/* 11 */ "Surface vessel",                // Surface vessel, boat, ship
/* 12 */ "Submarine",                     // Submarine
/* 13 */ "Hexarotor",                     // Hexarotor
/* 14 */ "Octorotor",                     // Octorotor
/* 15 */ "Tricopter",                     // Tricopter
/* 16 */ "Flapping wing",                 // Flapping wing
/* 17 */ "Kite",                          // Kite
/* 18 */ "Onboard companion controller",  // Onboard companion controller
/* 19 */ "Two",                           // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
/* 20 */ "Quad",                          // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
/* 21 */ "Tiltrotor VTOL",                // Tiltrotor VTOL
/* 22 */ "VTOL reserved 2",               // VTOL reserved 2
/* 23 */ "VTOL reserved 3",               // VTOL reserved 3
/* 24 */ "VTOL reserved 4",               // VTOL reserved 4
/* 25 */ "VTOL reserved 5",               // VTOL reserved 5
/* 26 */ "Gimbal",                        // Gimbal
/* 27 */ "ADSB system",                   // ADSB system
/* 28 */ "Steerable",                     // Steerable, nonrigid airfoil
/* 29 */ "Dodecarotor",                   // Dodecarotor
/* 30 */ "Camera",                        // Camera
/* 31 */ "Charging station",              // Charging station
/* 32 */ "FLARM collision avoidance system", // FLARM collision avoidance system
/* 33 */ "Servo",                         // Servo
/* 34 */ "Open Drone ID. See https:",     // Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.
/* 35 */ "Decarotor",                     // Decarotor
/* 36 */ "Battery",                       // Battery
/* 37 */ "Parachute",                     // Parachute
/* 38 */ "Log",                           // Log
/* 39 */ "OSD",                           // OSD
/* 40 */ "IMU",                           // IMU
/* 41 */ "GPS",                           // GPS
/* 42 */ "Winch",                         // Winch
}};


/**
 * mav_state_strings
 * @brief [User Define] 
 *                      描述飞行器 / 设备状态的枚举类型，用于标识无人机（或其他 MAVLink 设备）当前的系统状态（如初始化、待命、飞行中、故障等）。它是地面站监控设备状态的核心依据，也是设备间判断彼此可用性的重要参考。
 * @page mavlink/minimal/minimal.hpp
 * --> MAV_STATE
 */
static const std::array<const std::string, 9> mav_state_strings{{
/*  0 */ "Uninit",                        // Uninitialized system, state is unknown.                                                                                        未初始化状态（系统刚启动，正在进行基础初始化）
/*  1 */ "Boot",                          // System is booting up.                                                                                                          启动状态（初始化系统核心组件，如加载固件、配置参数）
/*  2 */ "Calibrating",                   // System is calibrating and not flight-ready.                                                                                    校准状态（正在进行传感器校准，如加速度计、磁力计校准）
/*  3 */ "Standby",                       // System is grounded and on standby. It can be launched any time.                                                                待命状态（系统初始化完成，等待用户指令）
/*  4 */ "Active",                        // System is active and might be already airborne. Motors are engaged.                                                            活动状态（正在执行任务或被手动控制）
/*  5 */ "Critical",                      // System is in a non-normal flight mode. It can however still navigate.                                                          紧急状态（系统出现严重异常，但仍可部分工作）
/*  6 */ "Emergency",                     // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.  应急状态（系统面临致命故障，执行应急程序）
/*  7 */ "Poweroff",                      // System just initialized its power-down sequence, will shut down now.                                                           关机状态（系统正在关闭或已断电）
/*  8 */ "Flight_Termination",            // System is terminating itself.                                                                                                  飞行终止状态（主动触发的终止程序，如安全气囊弹出、 parachute 开伞）
}};


/**
 * mav_attitude_adsb_strings
 * @brief [User Define] Enumeration of the ADSB altimeter types
 *                      描述 ADS-B（自动相关监视 - 广播）消息中高度数据类型 的枚举，用于明确高度值的基准（如气压基准、几何基准）和计算方式，确保接收方（如地面站、其他飞行器）能正确解析高度信息。
 * @param PRESSURE_QNH          气压高度（基于标准大气压基准）                  单位：英尺（ADS-B 标准默认），需通过气压传感器计算。
 * @param GEOMETRIC             几何高度（基于 WGS84 椭球面，即 “海拔高度”      单位：英尺，通常由 GNSS（如 GPS）提供。
 * @param NONE                  未知高度类型
 * 
 * @page mavros/common/common.h
 * --> ADSB_ALTITUDE_TYPE
 */
static const std::array<const std::string, 3> mav_attitude_type_strings{{
/*  0 */ "PRESSURE_QNH",                  // Altitude reported from a Baro source using QNH reference
/*  1 */ "GEOMETRIC",                     // Altitude reported from a GNSS source
/*  2 */ "NONE",                           // /*  | */
}};


/**
 * mav_emitting_type_strings
 * @brief [User Define] ADSB classification for the type of vehicle emitting the transponder signal.
 *                      描述 ADS-B（自动相关监视 - 广播）系统中航空器 / 目标类型
 * @page mavlink/common/common.h
 * --> ADSB_EMITTER_TYPE
 */
static const std::array<const std::string, 20> mav_emitting_type_strings{{
/*  0 */ "NO_INFO",                     // 无类型信息（目标未广播自身类型）
/*  1 */ "LIGHT",                       // 轻型航空器（如私人固定翼飞机，最大起飞重量 < 7000 千克）
/*  2 */ "SMALL",                       // 小型航空器（起飞重量 7000~15000 千克，如小型运输机）
/*  3 */ "LARGE",                       // 大型航空器（起飞重量 15000~75000 千克，如中型客机）
/*  4 */ "HIGH_VORTEX_LARGE",           
/*  5 */ "HEAVY",                       
/*  6 */ "HIGHLY_MANUV",
/*  7 */ "ROTOCRAFT",
/*  8 */ "UNASSIGNED",
/*  9 */ "GLIDER",
/* 10 */ "LIGHTER_AIR",
/* 11 */ "PARACHUTE",
/* 12 */ "ULTRA_LIGHT",
/* 13 */ "UNASSIGNED2",
/* 14 */ "UAV",
/* 15 */ "SPACE",
/* 16 */ "UNASSGINED3",
/* 17 */ "EMERGENCY_SURFACE",
/* 18 */ "SERVICE_SURFACE",
/* 19 */ "POINT_OBSTACLE",
}};


/**
 * mav_estimator_type_strings
 * @brief [User Define] Enumeration of estimator types
 *                 用于标识飞行器状态估计算法 / 模块类型
 * @page mavlink/common/common.h
 * 
 * --> MAV_ESTIMATOR_TYPE 
 */
static const std::array<const std::string, 9> mav_estimator_type_strings{{
/*  0 */ "UNKNOWN",                       // Unknown type of the estimator.
/*  1 */ "NAIVE",                         // This is a naive estimator without any real covariance feedback.
/*  2 */ "VISION",                        // Computer vision based estimate. Might be up to scale.
/*  3 */ "VIO",                           // Visual-inertial estimate.
/*  4 */ "GPS",                           // Plain GPS estimate.
/*  5 */ "GPS_INS",                       // Estimator integrating GPS and inertial sensing.
/*  6 */ "MOCAP",                         // Estimate from external motion capturing system.
/*  7 */ "LIDAR",                         // Estimator based on lidar sensor input.
/*  8 */ "AUTOPILOT",                     // Estimator on autopilot.
}};


/**
 * gps_fix_type_strings
 * @brief [User Define] Type of GPS fix.
 *                      描述 GPS 定位状态 / 精度等级 
 * @page mavlink/common/common.h
 * --> GPS_FIX_TYPE
 */
static const std::array<const std::string, 9> gps_fix_type_strings{{
/*  0 */ "NO_GPS",                        // No GPS connected                               无定位（GPS 未锁定卫星，或未接收到有效信号）
/*  1 */ "NO_FIX",                        // No position information, GPS is connected      航位推算（无实际 GPS 定位，基于历史位置和运动模型估算）
/*  2 */ "2D_FIX",                        // 2D position                                    二维定位（仅能提供纬度和经度，无海拔信息，通常需要至少 3 颗卫星）
/*  3 */ "3D_FIX",                        // 3D position                                    三维定位（能提供纬度、经度和海拔，通常需要至少 4 颗卫星）
/*  4 */ "DGPS",                          // DGPS/SBAS aided 3D position                    差分 GPS 定位（使用 DGPS 或 SBAS 等增强信号，精度提升至米级）
/*  5 */ "RTK_FLOAT",                     // RTK float, 3D position                         RTK 浮动解（实时动态差分，精度厘米级，但解算未完全收敛）
/*  6 */ "RTK_FIXED",                     // RTK Fixed, 3D position                         RTK 固定解（实时动态差分，解算完全收敛，最高精度）
/*  7 */ "STATIC",                        // Static fixed, typically used for base stations 静态定位（设备静止时的高精度解算，多用于基准站）
/*  8 */ "PPP",                           // PPP, 3D position.                              精密单点定位（无需基准站，通过全球精密星历实现厘米级精度）
}};


/**
 * mav_mission_result_strings
 * @brief [User Define] Result of mission operation (in a MISSION_ACK message).
 *                      用于定义任务操作结果的枚举类型，用于明确无人机在处理任务（如航点上传、任务执行、指令确认等）时的状态或失败原因。
 * @page mavlink/common/common.h
 * --> MAV_MISSION_RESULT
 */
static const std::array<const std::string, 16> mav_mission_result_strings{{
/*  0 */ "mission accepted OK",           // mission accepted OK                                                                                                                任务成功接受:无人机已成功接收并认可任务（如航点上传完成、指令被接纳），是正常处理的标志。 
/*  1 */ "Generic error / not accepting mission commands at all right now.", // Generic error / not accepting mission commands at all right now.                                通用错误:未明确分类的失败（如系统内部异常），无人机无法处理任务但原因不具体。
/*  2 */ "Coordinate frame is not supported.", // Coordinate frame is not supported.                                                                                            不支持的坐标帧:任务中使用的坐标系（如 MAV_FRAME 定义的帧）不为无人机所支持。
/*  3 */ "Command is not supported.",     // Command is not supported.                                                                                                          不支持的命令:任务中包含无人机无法识别或不支持的指令（如特定设备控制命令）。
/*  4 */ "Mission items exceed storage space.", // Mission items exceed storage space.                                                                                          存储空间不足:无人机的任务存储容量不足，无法容纳当前要上传的任务项（如航点数量过多）。
/*  5 */ "One of the parameters has an invalid value.", // One of the parameters has an invalid value.                                                                          任务参数无效:任务项中存在不符合要求的参数，但未指定具体哪个参数。
/*  6 */ "param1 has an invalid value.",  // param1 has an invalid value.                                                                                                       特定参数无效:
/*  7 */ "param2 has an invalid value.",  // param2 has an invalid value.
/*  8 */ "param3 has an invalid value.",  // param3 has an invalid value.
/*  9 */ "param4 has an invalid value.",  // param4 has an invalid value.
/* 10 */ "x / param5 has an invalid value.", // x / param5 has an invalid value.
/* 11 */ "y / param6 has an invalid value.", // y / param6 has an invalid value.
/* 12 */ "z / param7 has an invalid value.", // z / param7 has an invalid value.
/* 13 */ "Mission item received out of sequence", // Mission item received out of sequence                                                                                      任务项顺序错误:接收到的任务项序号不连续或不符合预期（如先收到序号 3，再收到序号 1）。
/* 14 */ "Not accepting any mission commands from this communication partner.", // Not accepting any mission commands from this communication partner.                          任务被拒绝:无人机明确拒绝从当前通信方接收任务（如权限限制、安全策略限制）。
/* 15 */ "Current mission operation cancelled (e.g. mission upload, mission download).", // Current mission operation cancelled (e.g. mission upload, mission download).        任务操作被取消:当前任务操作（如上传、下载）被主动中止（如地面站取消或无人机紧急中断）。
}};


/**
 * mav_distance_sensor_strings
 * @brief [User Define] Enumeration of distance sensor types 
 *                      描述距离传感器类型的枚举，用于标识无人机所搭载的距离传感器（如超声波、激光雷达等）的类型，帮助地面站或飞控系统理解传感器数据的特性（如测量范围、精度等）。
 * 
 * @page mavlink/common/common.h
 * --> MAV_DISTANCE_SENSOR
 */
static const std::array<const std::string, 5> mav_distance_sensor_strings{{
/*  0 */ "LASER",                         // Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units  激光雷达（LiDAR）传感器，通过激光测量距离，精度高（通常厘米级）
/*  1 */ "ULTRASOUND",                    // Ultrasound rangefinder, e.g. MaxBotix units                    超声波传感器，通过声波反射测量距离，精度较低（分米级）
/*  2 */ "INFRARED",                      // Infrared rangefinder, e.g. Sharp units                         红外传感器，通过红外信号反射测量距离，易受环境光干扰   
/*  3 */ "RADAR",                         // Radar type, e.g. uLanding units                                雷达传感器，抗干扰能力强（不受雨雾影响），测量距离远
/*  4 */ "UNKNOWN",                       // Broken or unknown type, e.g. analog units                      未知类型的距离传感器（未识别或未定义）
}};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * landing_target_type_strings
 * @brief [User Define] Type of landing target
 *                      描述着陆目标类型的枚举，用于标识无人机所跟踪的着陆目标（如着陆垫、跑道、标志物等）的具体类型，帮助飞控系统和地面站明确目标特性，适配不同的着陆逻辑。
 * 
 * @page mavlink/common/common.h
 * --> LANDING_TARGET_TYPE
 */
static const std::array<std::string, 4> landing_target_type_strings{{
/*  0 */ "LIGHT_BEACON",                  // Landing target signaled by light beacon (ex: IR-LOCK)
/*  1 */ "RADIO_BEACON",                  // Landing target signaled by radio beacon (ex: ILS, NDB)
/*  2 */ "VISION_FIDUCIAL",               // Landing target represented by a fiducial marker (ex: ARTag)
/*  3 */ "VISION_OTHER",                  // Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
}};


/**
 * 
 * @brief [User Define] Legacy component ID values for particular types of hardware/software that might make up a MAVLink system (autopilot, cameras, servos, avoidance systems etc.).
 *                      标识系统组件类型的枚举，用于明确 MAVLink 网络中各设备（如无人机、地面站、传感器）内部的具体组件（如飞控核心、GPS 模块、相机等）。它帮助消息接收方识别 “谁在发送消息” 或 “消息针对哪个组件”，是 MAVLink 通信中组件级寻址的核心机制。
 * 
 * @page mavlink/minimal/minimal.hpp
 * --> MAV_COMPONENT
 */
static const std::unordered_map<typename std::underlying_type<mavlink::minimal::MAV_COMPONENT>::type, const std::string> mav_comp_id_strings{{
{   0, "ALL" },                           // Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message.主自动驾驶仪（飞控核心），负责无人机的导航、控制和任务管理，是无人机最核心的组件（如 PX4、ArduPilot 飞控）。
{   1, "AUTOPILOT1" },                    // System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.
{  25, "USER1" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  26, "USER2" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  27, "USER3" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  28, "USER4" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  29, "USER5" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  30, "USER6" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  31, "USER7" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  32, "USER8" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  33, "USER9" },                         // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  34, "USER10" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  35, "USER11" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  36, "USER12" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  37, "USER13" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  38, "USER14" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  39, "USER15" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  40, "USER16" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  41, "USER17" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  42, "USER18" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  43, "USER19" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  44, "USER20" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  45, "USER21" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  46, "USER22" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  47, "USER23" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  48, "USER24" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  49, "USER25" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  50, "USER26" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  51, "USER27" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  52, "USER28" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  53, "USER29" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  54, "USER30" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  55, "USER31" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  56, "USER32" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  57, "USER33" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  58, "USER34" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  59, "USER35" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  60, "USER36" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  61, "USER37" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  62, "USER38" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  63, "USER39" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  64, "USER40" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  65, "USER41" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  66, "USER42" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  67, "USER43" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  68, "TELEMETRY_RADIO" },               // Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages).
{  69, "USER45" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  70, "USER46" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  71, "USER47" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  72, "USER48" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  73, "USER49" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  74, "USER50" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  75, "USER51" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  76, "USER52" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  77, "USER53" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  78, "USER54" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  79, "USER55" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  80, "USER56" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  81, "USER57" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  82, "USER58" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  83, "USER59" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  84, "USER60" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  85, "USER61" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  86, "USER62" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  87, "USER63" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  88, "USER64" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  89, "USER65" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  90, "USER66" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  91, "USER67" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  92, "USER68" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  93, "USER69" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  94, "USER70" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  95, "USER71" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  96, "USER72" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  97, "USER73" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  98, "USER74" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{  99, "USER75" },                        // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
{ 100, "CAMERA" },                        // Camera #1.
{ 101, "CAMERA2" },                       // Camera #2.
{ 102, "CAMERA3" },                       // Camera #3.
{ 103, "CAMERA4" },                       // Camera #4.
{ 104, "CAMERA5" },                       // Camera #5.
{ 105, "CAMERA6" },                       // Camera #6.
{ 140, "SERVO1" },                        // Servo #1.
{ 141, "SERVO2" },                        // Servo #2.
{ 142, "SERVO3" },                        // Servo #3.
{ 143, "SERVO4" },                        // Servo #4.
{ 144, "SERVO5" },                        // Servo #5.
{ 145, "SERVO6" },                        // Servo #6.
{ 146, "SERVO7" },                        // Servo #7.
{ 147, "SERVO8" },                        // Servo #8.
{ 148, "SERVO9" },                        // Servo #9.
{ 149, "SERVO10" },                       // Servo #10.
{ 150, "SERVO11" },                       // Servo #11.
{ 151, "SERVO12" },                       // Servo #12.
{ 152, "SERVO13" },                       // Servo #13.
{ 153, "SERVO14" },                       // Servo #14.
{ 154, "GIMBAL" },                        // Gimbal #1.
{ 155, "LOG" },                           // Logging component.
{ 156, "ADSB" },                          // Automatic Dependent Surveillance-Broadcast (ADS-B) component.
{ 157, "OSD" },                           // On Screen Display (OSD) devices for video links.
{ 158, "PERIPHERAL" },                    // Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.
{ 159, "QX1_GIMBAL" },                    // Gimbal ID for QX1.
{ 160, "FLARM" },                         // FLARM collision alert component.
{ 161, "PARACHUTE" },                     // Parachute component.
{ 171, "GIMBAL2" },                       // Gimbal #2.
{ 172, "GIMBAL3" },                       // Gimbal #3.
{ 173, "GIMBAL4" },                       // Gimbal #4
{ 174, "GIMBAL5" },                       // Gimbal #5.
{ 175, "GIMBAL6" },                       // Gimbal #6.
{ 180, "BATTERY" },                       // Battery #1.
{ 181, "BATTERY2" },                      // Battery #2.
{ 189, "MAVCAN" },                        // CAN over MAVLink client.
{ 190, "MISSIONPLANNER" },                // Component that can generate/supply a mission flight plan (e.g. GCS or developer API).
{ 191, "ONBOARD_COMPUTER" },              // Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
{ 192, "ONBOARD_COMPUTER2" },             // Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
{ 193, "ONBOARD_COMPUTER3" },             // Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
{ 194, "ONBOARD_COMPUTER4" },             // Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
{ 195, "PATHPLANNER" },                   // Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).
{ 196, "OBSTACLE_AVOIDANCE" },            // Component that plans a collision free path between two points.
{ 197, "VISUAL_INERTIAL_ODOMETRY" },      // Component that provides position estimates using VIO techniques.
{ 198, "PAIRING_MANAGER" },               // Component that manages pairing of vehicle and GCS.
{ 200, "IMU" },                           // Inertial Measurement Unit (IMU) #1.
{ 201, "IMU_2" },                         // Inertial Measurement Unit (IMU) #2.
{ 202, "IMU_3" },                         // Inertial Measurement Unit (IMU) #3.
{ 220, "GPS" },                           // GPS #1.
{ 221, "GPS2" },                          // GPS #2.
{ 236, "ODID_TXRX_1" },                   // Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
{ 237, "ODID_TXRX_2" },                   // Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
{ 238, "ODID_TXRX_3" },                   // Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
{ 240, "UDP_BRIDGE" },                    // Component to bridge MAVLink to UDP (i.e. from a UART).
{ 241, "UART_BRIDGE" },                   // Component to bridge to UART (i.e. from UDP).
{ 242, "TUNNEL_NODE" },                   // Component handling TUNNEL messages (e.g. vendor specific GUI of a component).
{ 250, "SYSTEM_CONTROL" },                // Component for handling system messages (e.g. to ARM, takeoff, etc.).
}};


/**
 * mav_frame_strings
 * @brief [User Define] Coordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles.
 *                      定义坐标帧（Coordinate Frames） 的核心枚举类型，用于明确描述位置、姿态、速度等空间信息所基于的参考坐标系。
 *                      MAV_FRAME 涵盖了无人机常用的各类坐标系，可分为全局坐标系、本地坐标系、机体坐标系
 * @page mavlink/common/common.h
 * --> MAV_FRAME
 */
static const std::array<std::string, 22> mav_frame_strings{{
// 全局坐标系，包含 WGS84 纬度、经度和 MSL（平均海平面）高度（海拔）。
/*  0 */ "GLOBAL",                        // Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
// 本地 NED（北 - 东 - 下）坐标系，原点为无人机起飞点或 Home 点，X 轴朝北、Y 轴朝东、Z 轴朝下。
/*  1 */ "LOCAL_NED",                     // NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
/*  2 */ "MISSION",                       // NOT a coordinate frame, indicates a mission command.
// 全局相对高度坐标系，经纬度为 WGS84 绝对坐标，高度为相对于无人机 Home 点的高度（而非海拔）。
/*  3 */ "GLOBAL_RELATIVE_ALT",           // Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
// 本地 ENU（东 - 北 - 上）坐标系，原点同上，X 轴朝东、Y 轴朝北、Z 轴朝上（更符合地面坐标系习惯）。
/*  4 */ "LOCAL_ENU",                     // ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
// 整数化全局坐标系，经纬度以 “度 ×1e7” 表示（如纬度 30°→300000000），节省传输带宽。
/*  5 */ "GLOBAL_INT",                    // Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).
/*  6 */ "GLOBAL_RELATIVE_ALT_INT",       // Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home location.
// 本地偏移 NED 坐标系，原点为无人机当前位置，用于描述相对当前位置的目标点（如 “向前 10 米”）。
/*  7 */ "LOCAL_OFFSET_NED",              // NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
// 机体 NED 坐标系，原点为无人机质心，X 轴沿机身向前、Y 轴向右、Z 轴向下（随无人机姿态旋转）。
/*  8 */ "BODY_NED",                      // Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/accelaration values.
// 机体偏移 NED 坐标系（已部分废弃），原点为无人机质心，但坐标轴不随姿态完全旋转（保留历史兼容性）。
/*  9 */ "BODY_OFFSET_NED",               // This is the same as MAV_FRAME_BODY_FRD.
// 全局地形相对高度坐标系，高度为相对于地面（地形）的高度。
/* 10 */ "GLOBAL_TERRAIN_ALT",            // Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
/* 11 */ "GLOBAL_TERRAIN_ALT_INT",        // Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
/* 12 */ "BODY_FRD",                      // FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The forward axis is aligned to the front of the vehicle in the horizontal plane.
/* 13 */ "RESERVED_13",                   // MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).
/* 14 */ "RESERVED_14",                   // MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).
/* 15 */ "RESERVED_15",                   // MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).
/* 16 */ "RESERVED_16",                   // MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).
/* 17 */ "RESERVED_17",                   // MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).
/* 18 */ "RESERVED_18",                   // MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).
/* 19 */ "RESERVED_19",                   // MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).
/* 20 */ "LOCAL_FRD",                     // FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
/* 21 */ "LOCAL_FLU",                     // FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
}};

///////////////////////////////////////////////////////convert: mavlink-->std::string////////////////////////////////////////////////////////////////////

std::string to_string(mavlink::minimal::MAV_AUTOPILOT e);
std::string to_string(mavlink::minimal::MAV_STATE e);
std::string to_string(mavlink::common::ADSB_ALTITUDE_TYPE e);
std::string to_string(mavlink::common::ADSB_EMITTER_TYPE e);
std::string to_string(mavlink::common::MAV_ESTIMATOR_TYPE e);
std::string to_string(mavlink::common::GPS_FIX_TYPE e);
std::string to_string(mavlink::common::MAV_MISSION_RESULT e);
std::string to_string(mavlink::common::MAV_DISTANCE_SENSOR e);
std::string to_string(mavlink::minimal::MAV_COMPONENT e);

////////////////////////////////////////////////////////convert: mavlink<--->std::string///////////////////////////////////////////////////////////////

std::string to_string(mavlink::minimal::MAV_TYPE e);
mavlink::minimal::MAV_TYPE mav_type_from_str(const std::string &str);

std::string to_string(mavlink::common::LANDING_TARGET_TYPE e);
mavlink::common::LANDING_TARGET_TYPE landing_target_type_from_str(const std::string &str);

std::string to_string(mavlink::common::MAV_FRAME e);
mavlink::common::MAV_FRAME mav_frame_from_str(const std::string &str);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
}
