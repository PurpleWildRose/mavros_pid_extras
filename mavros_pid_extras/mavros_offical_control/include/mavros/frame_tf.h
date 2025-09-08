/**************************************************************************************************************************************************************
 * @file frame_tf.h
 * @brief rame_tf.h，专注于坐标系变换与数据格式转换，是无人机导航与控制中 “空间数据标准化” 的关键工具。
 * @addtogroup nodelib
 **************************************************************************************************************************************************************/
#pragma once

#include <array>
#include <Eigen/Eigen> 								// Eigen 库（线性代数与几何计算核心库），用于矩阵、向量、四元数的运算（无人机坐标系变换的数学基础）。
#include <Eigen/Geometry>
#include <ros/assert.h>								// ROS 的断言宏（如 ROS_ASSERT_MSG），用于调试时检查参数合法性（如协方差矩阵大小是否匹配）。

// for Covariance types								// 引入与 “空间数据” 相关的 ROS 消息类型，用于后续定义 “与 ROS 消息匹配的数据类型”
#include <sensor_msgs/Imu.h>						// IMU 消息（包含角速度协方差，用于定义 3x3 协方差类型）。
#include <geometry_msgs/Point.h>					// 几何消息（点、向量、四元数、带协方差的位姿，用于定义坐标、姿态与协方差类型）。
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace mavros{
namespace ftf{
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
// 协方差矩阵用于描述数据的不确定性（如位置误差、姿态误差），ROS 消息中协方差以 “一维数组” 存储（行优先），此处定义与 ROS 消息对应的类型别名，方便后续操作：
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * Covariance3d
 * @brief 3x3 协方差矩阵类型，匹配 sensor_msgs/Imu 消息中 “角速度协方差” 的类型（_angular_velocity_covariance_type 本质是 boost::array<double, 9>，9 个元素对应 3x3 矩阵的行优先存储）。
 * 
 * @attention 用途：描述 3 维数据的不确定性（如角速度误差、线速度误差）。
 */
using Covariance3d = sensor_msgs::Imu::_angular_velocity_covariance_type;

/**
 * Covariance6d
 * @brief 6x6 协方差矩阵类型，匹配 geometry_msgs/PoseWithCovariance 消息中 “位姿协方差” 的类型（_covariance_type 是 boost::array<double, 36>，36 个元素对应 6x6 矩阵）。
 * 
 * @attention 用途：描述 “位置（3 维）+ 姿态（3 维）” 的联合不确定性。
 */
using Covariance6d = geometry_msgs::PoseWithCovariance::_covariance_type;

/**
 * Covariance9d
 * @brief 9x9 协方差矩阵类型，自定义为 boost::array<double, 81>（81=9x9）。
 * 
 * @attention 用途：描述更高维度数据的不确定性（如 “位置 + 速度 + 姿态” 的联合误差）。
 */
using Covariance9d = boost::array<double, 81>;


//-----------------------------------------------------------------------------------------------------------------------------------------------------------
// Eigen 库操作矩阵时需 “Eigen 矩阵类型”，但 ROS 协方差是 “一维数组”，此处通过 Eigen::Map 将 “一维数组” 映射为 “Eigen 矩阵”（零拷贝，直接操作原数据），避免数据拷贝的开销 
// Eigen::Map：Eigen 的映射类，将 “外部内存中的数据” 视为 Eigen 矩阵 / 向量。
// Eigen::RowMajor：指定矩阵按 “行优先” 存储（与 ROS 消息的协方差存储格式一致，避免数据错乱）。
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
using EigenMapCovariance3d = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;
using EigenMapConstCovariance3d = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;

using EigenMapCovariance6d = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;
using EigenMapConstCovariance6d = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;

using EigenMapCovariance9d = Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >;
using EigenMapConstCovariance9d = Eigen::Map<const Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >;


//-----------------------------------------------------------------------------------------------------------------------------------------------------------
// 定义枚举类型，明确 “从哪个坐标系变换到哪个坐标系”，避免函数参数歧义（如 “NED→ENU” 和 “ENU→NED” 是完全相反的变换）。
// Static Transformation 的缩写，代表 “固定坐标系间的变换”（变换关系不随时间变化）。
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
enum class StaticTF{
    /* NED_TO_ENU：从 NED 坐标系（北 - 东 - 下，无人机常用惯性系）变换到 ENU 坐标系（东 - 北 - 上，ROS 标准惯性系）。 */
    NED_TO_ENU,
    ENU_TO_NED,
    /* AIRCRAFT_TO_BASELINK：从 机体坐标系（Aircraft）（原点在机身重心，X 轴向前）变换到 基准坐标系（Baselink）（原点在机器人底座，X 轴向前，常用于多传感器校准）。 */
    AIRCRAFT_TO_BASELINK,
    BASELINK_TO_AIRCRAFT,
    /* ABSOLUTE_FRAME_*：“绝对帧下的变换”—— 此处 “绝对帧” 指局部惯性系（如 NED/ENU），变换时需考虑姿态在绝对帧中的参考（而非仅坐标系间的相对旋转）。 */
    ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK,
    ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT
};


//-----------------------------------------------------------------------------------------------------------------------------------------------------------
// 枚举值用途：实现 ECEF 与 ENU 坐标系的转换（如将 GPS 输出的 ECEF 坐标转换为 ROS 常用的 ENU 坐标）
// ECEF：Earth-Centered, Earth-Fixed（地心地固坐标系），是全球统一的绝对坐标系（原点在地球质心，X 轴指向本初子午线与赤道交点，Z 轴指向北极）。
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
enum class StaticEcefTF{
    ECEF_TO_ENU,
    ENU_TO_ECEF
};

//=========================================================================DETAIL============================================================================
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DETAIL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//=========================================================================START=============================================================================
namespace detail{
/**
 * transform_orientation
 * @brief 
 * 
 */
Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform);



}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DETAIL <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//===========================================================================END=============================================================================



/**
 * 
 */
template<class T>
inline T transform_orientation_ned_enu(const T &in) {
	return detail::transform_orientation(in, StaticTF::NED_TO_ENU);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Utils Tools >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//=========================================================================START=============================================================================

/**
 * quaternion_from_rpy
 * @brief 将 “滚转 - 俯仰 - 偏航” 欧拉角（rpy 向量）转换为 Eigen 四元数（Eigen::Quaterniond），解决欧拉角 “万向锁” 问题（当俯仰角为 ±90° 时，滚转与偏航轴重合，导致姿态无法唯一表示）。
 * 
 * @param const Eigen::Vector3d &rpy—— 输入欧拉角向量，各分量定义严格遵循 YPR-ZYX 公约：
 * 
 * @var rpy.x()：Roll（滚转）—— 绕 X 轴 旋转的角度（右手定则：X 轴向前，逆时针滚转为正）；
 * @var rpy.y()：Pitch（俯仰）—— 绕 Y 轴 旋转的角度（右手定则：Y 轴向右，抬头为正）；
 * @var rpy.z()：Yaw（偏航）—— 绕 Z 轴 旋转的角度（右手定则：Z 轴向上，逆时针偏航为正）；
 * @attention 单位：弧度（rad），需确保输入角度不是角度制（若为角度制，需先乘以 M_PI/180 转换为弧度）。
 */
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);

inline Eigen::Quaterniond quaternion_from_rpy(const double roll, const double pitch, const double yaw) {
    return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
}

/**
 * quaternion_to_rpy
 * @brief 将 Eigen 四元数（q）反向转换为 “滚转 - 俯仰 - 偏航” 欧拉角（遵循 YPR-ZYX 公约），用于姿态的 “人类可读化”（如地面站显示滚转 / 俯仰 / 偏航数值，而非四元数的 w/x/y/z 分量）。
 * @param const Eigen::Quaterniond &q—— 输入四元数，需确保是归一化四元数（Eigen 四元数默认会归一化，但若手动构造四元数，需调用 q.normalized() 确保规范性，否则转换结果会出错）。
 * 
 * @details 1. 四元数转旋转矩阵：q.toRotationMatrix()—— 将四元数转换为 3x3 旋转矩阵（旋转矩阵可直观表示坐标系间的旋转关系）；
 * @details 2. 旋转矩阵提取欧拉角：eulerAngles(2, 1, 0)——Eigen 旋转矩阵的 eulerAngles 方法按 “指定轴顺序” 提取欧拉角，参数 (2, 1, 0) 对应：
 *              2：第一旋转轴（最外层旋转）——Z 轴（Eigen 中轴索引：X=0，Y=1，Z=2）；
 *              1：第二旋转轴（中间层旋转）——Y 轴；
 *              0：第三旋转轴（最内层旋转）——X 轴；
 *             此时提取的欧拉角顺序为 Z（偏航）→ Y（俯仰）→ X（滚转），存储在向量中为 [Yaw, Pitch, Roll]；
 * @details 3. 向量反转：.reverse()—— 将 [Yaw, Pitch, Roll] 反转为 [Roll, Pitch, Yaw]，与输入 rpy 向量的分量顺序（x=Roll，y=Pitch，z=Yaw）一致，确保接口对称。
 */
Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q);

inline void quaternion_to_rpy(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    const auto rpy = quaternion_to_rpy(q);
    roll = rpy.x();
    pitch = rpy.y();
    yaw = rpy.z();
}
  
/**
 * quaternion_get_yaw
 * @brief 直接从四元数中提取偏航角（Yaw），避免完整的 “四元数→旋转矩阵→欧拉角” 转换（减少计算量，适合高频场景如无人机航向控制）。
 * @param const Eigen::Quaterniond &q—— 输入归一化四元数，四元数分量定义为 q = [q0, q1, q2, q3] = [w, x, y, z]（Eigen 四元数的内部存储顺序为 x/y/z/w，但此处通过 q.w()/q.x() 显式获取，避免顺序错误）。
 * 
 * @attention 公式来源于维基百科 “四元数与欧拉角转换” 章节，专门针对 YPR-ZYX 公约 的偏航角推导：
 *              1. 四元数分量映射：q0 = w（实部），q1 = x（X 轴虚部），q2 = y（Y 轴虚部），q3 = z（Z 轴虚部）；
 *              2. 偏航角计算公式：Yaw=arctan2(2(q0q3+q1q2),1−2(q22+q32))
 * @def 优势：无需转换旋转矩阵，直接通过四则运算计算，计算效率比 quaternion_to_rpy 高 3~5 倍，适合无人机每秒数百次的航向更新需求。
 */
double quaternion_get_yaw(const Eigen::Quaterniond &q);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Utils Tools <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//===========================================================================END=============================================================================
}
}
