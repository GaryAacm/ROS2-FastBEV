#!/usr/bin/env bash
set -e

source /opt/ros/foxy/setup.bash
source /root/autodl-tmp/ros2_ws/install/setup.bash

# 1) 先强制系统 libffi（解决 libp11-kit 的 LIBFFI_BASE_7.0）
if [ -f /usr/lib/x86_64-linux-gnu/libffi.so.7 ]; then
  export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7:${LD_PRELOAD:-}
fi

# 2) 系统库路径放最前，避免被 conda 覆盖
SYS_LIBS="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu"

# 3) FastBEV / TensorRT / CUDA
TRT_LIBS="/root/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib"
CUDA_LIBS="/usr/local/cuda/lib64"
FASTBEV_LIBS="/root/autodl-tmp/CUDA-FastBEV/build"

# 4) 只把 spconv/cumm 的 .libs 目录加进来（这里通常没有 libffi/libtinfo）
SPCONV_LIBS="/root/miniconda3/envs/fastbev/lib/python3.8/site-packages/spconv_cu113.libs"
CUMM_LIBS="/root/miniconda3/envs/fastbev/lib/python3.8/site-packages/cumm_cu113.libs"

# 5) conda env lib 放最后（只为 libspconv.so / 其它依赖兜底）
CONDA_LIBS="/root/miniconda3/envs/fastbev/lib"

export LD_LIBRARY_PATH="${SYS_LIBS}:${FASTBEV_LIBS}:${TRT_LIBS}:${CUDA_LIBS}:${SPCONV_LIBS}:${CUMM_LIBS}:${CONDA_LIBS}:${LD_LIBRARY_PATH:-}"

export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}

exec ros2 run fastbev_trt_ros fastbev_node "$@"
