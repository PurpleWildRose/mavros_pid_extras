##
# 脚本 install_geographiclib_dataset.sh 是 MAVROS 提供的官方数据集安装脚本，会自动下载并部署
#
# This module verifies the installation of the GeographicLib datasets and warns
# if it doesn't detect them.
##

# 大地水准面模型    用于海拔高度计算（如 WGS84 椭球面到大地水准面的转换）
find_path(GEOGRAPHICLIB_GEOID_PATH NAMES geoids PATH_SUFFIXES share/GeographicLib share/geographiclib)
# 重力场模型    用于高精度重力场计算（如卫星轨道、惯性导航修正）
find_path(GEOGRAPHICLIB_GRAVITY_PATH_ NAMES gravity PATH_SUFFIXES share/GeographicLib)
# 磁场模型    用于地磁场计算（如磁罗盘校准、地磁导航）
find_path(GEOGRAPHICLIB_MAGNETIC_PATH_ NAMES magnetic PATH_SUFFIXES share/GeographicLib)

if(NOT GEOGRAPHICLIB_GEOID_PATH)
  message(STATUS "No geoid model datasets found. This will result on a SIGINT! Please execute the script install_geographiclib_dataset.sh in /mavros/scripts")
else()
  message(STATUS "Geoid model datasets found in: " ${GEOGRAPHICLIB_GEOID_PATH}/geoid)
  set(GEOGRAPHICLIB_GEOID_PATH ${GEOGRAPHICLIB_GEOID_PATH}/geoid)
endif()
if(NOT GEOGRAPHICLIB_GRAVITY_PATH_)
  message(STATUS "No gravity field model datasets found. Please execute the script install_geographiclib_dataset.sh in /mavros/scripts")
else()
  message(STATUS "Gravity Field model datasets found in: " ${GEOGRAPHICLIB_GRAVITY_PATH_}/gravity)
  set(GEOGRAPHICLIB_GRAVITY_PATH ${GEOGRAPHICLIB_GRAVITY_PATH_}/gravity)
endif()
if(NOT GEOGRAPHICLIB_MAGNETIC_PATH_)
  message(STATUS "No magnetic field model datasets found. Please execute the script install_geographiclib_dataset.sh in /mavros/scripts")
else()
  message(STATUS "Magnetic Field model datasets found in: " ${GEOGRAPHICLIB_MAGNETIC_PATH_}/magnetic)
  set(GEOGRAPHICLIB_MAGNETIC_PATH ${GEOGRAPHICLIB_MAGNETIC_PATH_}/magnetic)
endif()
