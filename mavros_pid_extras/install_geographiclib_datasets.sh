#!/bin/bash
# Script to install the model datasets required
# to GeographicLib apply certain conversions

if [[ $UID != 0 ]]; then
	echo "This script require root privileges!" 1>&2
	exit 1
fi

# Install datasets
run_get() {
	local dir="$1"
	local tool="$2"
	local model="$3"

	files=$(shopt -s nullglob dotglob; echo /usr/share/GeographicLib/$dir/$model* /usr/local/share/GeographicLib/$dir/$model*)
	if (( ${#files} )); then
		echo "GeographicLib $tool dataset $model already exists, skipping"
		return
	fi

	echo "Installing GeographicLib $tool $model"
	geographiclib-get-$tool $model >/dev/null 2>&1
	
	files=$(shopt -s nullglob dotglob; echo /usr/share/GeographicLib/$dir/$model* /usr/local/share/GeographicLib/$dir/$model*)
	if (( ! ${#files} )); then
		echo "Error while installing GeographicLib $tool $model"
		return
	fi
}

# check which command script is available
# geographiclib-get-geoids 是 GeographicLib（一个用于地理空间计算的开源库）提供的命令行工具，专门用于下载和安装大地水准面模型（geoid models）。这些模型是将椭球面高度（如 GPS 直接输出的高度）转换为正高（海拔高度，以平均海平面为基准）的关键数据。
if hash geographiclib-get-geoids; then
    # 调用自定义函数 run_get 下载 egm96-5 大地水准面模型（用于将椭球面高度转换为正高）。
    # 参数含义：geoids（数据集类型）、geoids（存储目录）、egm96-5（具体模型名称）。
	run_get geoids geoids egm96-5
    # 下载 egm96 重力模型（用于地球重力场计算）。
	run_get gravity gravity egm96
    # 下载 emm2015 磁场模型（用于地球磁场计算）。
	run_get magnetic magnetic emm2015
elif hash geographiclib-datasets-download; then # only allows install the goid model dataset
	geographiclib-datasets-download egm96_5;
else
	echo "OS not supported! Check GeographicLib page for supported OS and lib versions." 1>&2
fi
