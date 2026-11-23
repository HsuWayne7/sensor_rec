#!/usr/bin/env bash
# 新筆電第一步: 裝顯卡驅動、CUDA 13.0、ROS 2 Jazzy、ZED SDK 5.5.0 (和原始筆電同版本)
# 前提: Ubuntu 24.04、有網路。跑完要重開機，再跑 install.sh
# 用法: ~/sensor_rec/install_base.sh [ZED SDK 安裝檔路徑]
set -eu
step() { echo; echo "===== $* ====="; }
. /etc/os-release
[ "$VERSION_ID" = "24.04" ] || { echo "[ERR] 需要 Ubuntu 24.04 (現在是 $VERSION_ID)"; exit 1; }

sudo apt-get update
sudo apt-get install -y curl wget git zstd software-properties-common

step "1. NVIDIA 顯卡驅動 (原始筆電: nvidia-driver-595-open)"
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then echo "[OK] 已安裝: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader)"
else sudo apt-get install -y nvidia-driver-595-open || sudo ubuntu-drivers install; fi

step "2. CUDA 13.0 (NVIDIA 官方 apt 源)"
if [ ! -d /usr/local/cuda-13.0 ]; then
  wget -q -O /tmp/cuda-keyring.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
  sudo dpkg -i /tmp/cuda-keyring.deb
  sudo apt-get update
  sudo apt-get install -y cuda-toolkit-13-0
else echo "[OK] 已安裝"; fi
grep -q '/usr/local/cuda/bin' ~/.bashrc || cat >> ~/.bashrc <<'EOF'
export CUDA_HOME=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
EOF

step "3. ROS 2 Jazzy desktop"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  sudo add-apt-repository -y universe
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')
  wget -q -O /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-jazzy-desktop ros-dev-tools
else echo "[OK] 已安裝"; fi

step "4. ZED SDK 5.5.0 (CUDA 13 / Ubuntu 24)"
if [ ! -d /usr/local/zed ]; then
  RUN=${1:-}
  if [ -z "$RUN" ]; then
    RUN=/tmp/ZED_SDK_v5.5.0.zstd.run
    # 原始筆電的安裝檔: /mnt/hdd/linux_data/installers/ZED_SDK_Ubuntu24_cuda13.0_tensorrt10.13_v5.5.0.zstd.run
    # 官網下載失敗就到 https://www.stereolabs.com/developers/release 手動下載 5.5.0，然後把路徑當參數給這個腳本
    wget -O "$RUN" https://download.stereolabs.com/zedsdk/5.5/cu13/ubuntu24 || { echo "[ERR] 下載失敗，請手動下載 ZED SDK 5.5.0"; exit 1; }
  fi
  chmod +x "$RUN"
  "$RUN" -- silent
else echo "[OK] 已安裝"; fi

step "完成"
echo "請重開機，之後執行: ~/sensor_rec/install.sh"
