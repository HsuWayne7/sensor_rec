#!/usr/bin/env bash
# 在新筆電上把錄製環境裝成跟原始筆電一模一樣 (Ubuntu 24.04 + ROS 2 Jazzy)
# 用法: git clone <repo> ~/sensor_rec && ~/sensor_rec/install.sh
# 會用到 sudo (apt / make install / udev / nmcli)，可重複執行，已完成的步驟會跳過。
# 前提: 已裝好 ROS 2 Jazzy (/opt/ros/jazzy)、NVIDIA driver + CUDA、ZED SDK 5.5.0 (見 README)
set -eu
R=$(cd "$(dirname "$0")" && pwd)
source "$R/patches/versions.env"
JOBS=$(( $(nproc) > 4 ? $(nproc) - 2 : 2 ))

step() { echo; echo "===== $* ====="; }

# repo 必須在 ~/sensor_rec (rec_all.sh 等用 $HOME/sensor_rec 路徑)
if [ "$R" != "$HOME/sensor_rec" ]; then
  echo "[ERR] 請把 repo clone 到 ~/sensor_rec (現在在 $R)"; exit 1
fi
[ -f /opt/ros/jazzy/setup.bash ] || { echo "[ERR] 先裝 ROS 2 Jazzy"; exit 1; }
[ -d /usr/local/zed ] || { echo "[ERR] 先裝 ZED SDK $ZED_SDK_VERSION (見 README)"; exit 1; }

clone_at() {  # clone_at <url> <dir> <commit> <patch>
  local url=$1 dir=$2 commit=$3 patch=$4
  if [ ! -d "$dir/.git" ]; then git clone "$url" "$dir"; fi
  git -C "$dir" fetch --tags -q origin || true
  git -C "$dir" checkout -q -f "$commit"
  if [ -s "$patch" ]; then git -C "$dir" apply "$patch"; fi
  echo "[OK] $dir @ ${commit:0:7} + $(basename "$patch")"
}

step "1. apt 套件"
sudo apt-get update
sudo apt-get install -y git cmake build-essential python3-colcon-common-extensions python3-rosdep \
  python3-yaml python3-numpy libusb-1.0-0-dev libssl-dev libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosbag2-storage-mcap ros-jazzy-rviz2 \
  ros-jazzy-rqt-image-view ros-jazzy-pcl-ros iputils-ping
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

step "2. Livox-SDK2 (裝到 /usr/local)"
if [ ! -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
  [ -d ~/Livox-SDK2 ] || git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
  cmake -S ~/Livox-SDK2 -B ~/Livox-SDK2/build
  make -C ~/Livox-SDK2/build -j"$JOBS"
  sudo make -C ~/Livox-SDK2/build install
else echo "[OK] 已安裝"; fi

step "3. livox_ros_driver2 (~/ws_livox)"
mkdir -p ~/ws_livox/src
clone_at https://github.com/Livox-SDK/livox_ros_driver2.git ~/ws_livox/src/livox_ros_driver2 \
  "$LIVOX_ROS_DRIVER2_COMMIT" "$R/patches/livox_ros_driver2.diff"
cp ~/ws_livox/src/livox_ros_driver2/package_ROS2.xml ~/ws_livox/src/livox_ros_driver2/package.xml
cp -r ~/ws_livox/src/livox_ros_driver2/launch_ROS2 ~/ws_livox/src/livox_ros_driver2/launch
( set +u; source /opt/ros/jazzy/setup.bash; cd ~/ws_livox
  colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_BUILD_TYPE=Release )

step "4. librealsense v2.53.1 (T265 用，RSUSB backend + python binding)"
clone_at https://github.com/IntelRealSense/librealsense.git ~/librealsense \
  "$LIBREALSENSE_COMMIT" "$R/patches/librealsense.diff"
if [ ! -f ~/librealsense/build/wrappers/python/pyrealsense2.cpython-312-x86_64-linux-gnu.so ]; then
  cmake -S ~/librealsense -B ~/librealsense/build -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_WITH_CUDA=OFF
  make -C ~/librealsense/build -j"$JOBS"
  sudo make -C ~/librealsense/build install
else echo "[OK] 已編譯"; fi
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

step "5. t265_ros (~/t265_ws，原始碼在 repo 的 drivers/t265_ros)"
mkdir -p ~/t265_ws/src
[ -e ~/t265_ws/src/t265_ros ] || ln -s "$R/drivers/t265_ros" ~/t265_ws/src/t265_ros
( set +u; source /opt/ros/jazzy/setup.bash; cd ~/t265_ws; colcon build )

step "6. zed-ros2-wrapper (~/zed_ws)"
mkdir -p ~/zed_ws/src
clone_at https://github.com/stereolabs/zed-ros2-wrapper.git ~/zed_ws/src/zed-ros2-wrapper \
  "$ZED_ROS2_WRAPPER_COMMIT" "$R/patches/zed-ros2-wrapper.diff"
( set +u; source /opt/ros/jazzy/setup.bash; cd ~/zed_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release )

step "7. ZED 設定檔 ~/zed_cfg -> repo"
if [ -L ~/zed_cfg ]; then echo "[OK] 已是 symlink"
elif [ -e ~/zed_cfg ]; then echo "[!!] ~/zed_cfg 已存在且不是 symlink，沒動它 (rec_all.sh 會讀它)"
else ln -s "$R/zed_cfg" ~/zed_cfg; fi

step "8. 光達網卡 (192.168.1.5/24，光達 192.168.1.110)"
if nmcli -t -f NAME connection show | grep -qx livox; then echo "[OK] 已有 livox 連線"
else
  echo "有線網卡:"; nmcli -t -f DEVICE,TYPE device | grep ethernet || true
  read -rp "光達接哪張網卡? (原始筆電是 enp43s0): " IF
  sudo nmcli connection add type ethernet con-name livox ifname "$IF" \
    ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv6.method disabled
fi
echo "[!!] 若網卡名稱不是 enp43s0，rec_all.sh 只是錯誤訊息提到它，不影響錄製"

step "完成"
echo "測試: 開光達、ZED/T265 直插筆電 -> ~/sensor_rec/rec_all.sh test1"
echo "沒有 /mnt/hdd/linux_data/bags 時會錄到 ~/bags (或 OUT_DIR=... 指定)"
