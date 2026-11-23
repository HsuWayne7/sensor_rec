#!/usr/bin/env bash
# 一鍵: 啟動 MID-360 + ZED2 + T265 -> 確認所有 topic 有資料 -> ros2 bag record (mcap)
# 用法:
#   rec_all.sh [bag名稱]          Ctrl+C 停止錄製 (會先關 bag 再關 driver)
#   DURATION=120 rec_all.sh h4    錄 120 秒自動停
#   rec_all.sh --expo             只開 ZED,即時看亮度調曝光/增益/白平衡,不錄
# 環境變數: OUT_DIR (預設 /mnt/hdd/linux_data/bags,HDD 不在就 ~/bags)、FORCE=1 (topic 檢查失敗也照錄)、ZED_CFG
#           NO_VIZ=1 (錄製時不開 rviz2 看 ZED 影像)
#           EXTR (外參檔,預設 ~/sensor_rec/extrinsics.yaml,發到 /tf_static 並複製進 bag)
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ZED_CFG=${ZED_CFG:-$HOME/zed_cfg/zed_lock.yaml}
if [ ! -f "$ZED_CFG" ]; then
  echo "[ERR] 找不到 ZED 設定檔 $ZED_CFG (被刪了? 看看垃圾桶 ~/.local/share/Trash/files/)"; exit 1
fi
# 預設 (11/9 定案，11/21 從 1080p15 改回): HD720@30、曝光 25 / 增益 30 / 白平衡 4500K 全手動,備份在 ~/zed_cfg/zed_lock_e25g30wb45.yaml (1080p15 版在 zed_lock_1080p15_e12g30wb45.yaml)
echo "[..] ZED 設定 $ZED_CFG: $(grep -E 'auto_exposure_gain|exposure:|gain:|auto_whitebalance|whitebalance_temperature' "$ZED_CFG" | tr -s ' ' | tr '\n' ' ')"
EXTR=${EXTR:-$HERE/extrinsics.yaml}
HDD_BAGS=/mnt/hdd/linux_data/bags
if [ -z "${OUT_DIR:-}" ]; then
  # 預設錄到 HDD (實測 65 MB/s,錄製只要 ~10 MB/s);HDD 沒掛載就退回 ~/bags
  if [ -d "$HDD_BAGS" ] && [ -w "$HDD_BAGS" ]; then OUT_DIR=$HDD_BAGS
  else OUT_DIR=$HOME/bags; echo "[!!] HDD 沒掛載,改錄到 $OUT_DIR"; fi
fi

TOPICS=(
  /livox/lidar /livox/imu
  /t265/odom /t265/imu/gyro /t265/imu/accel
  /zed/zed_node/rgb/color/rect/image/compressed
  /zed/zed_node/rgb/color/rect/camera_info
  /zed/zed_node/imu/data
  /zed/zed_node/odom /zed/zed_node/pose /zed/zed_node/pose/status
  /tf /tf_static
)

# ---- 環境: 三個 ws 疊在同一個 shell ----
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ws_livox/install/local_setup.bash"
source "$HOME/t265_ws/install/local_setup.bash"
source "$HOME/zed_ws/install/local_setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$HOME/sensor_rec/cyclonedds_local.xml}
# T265 用自己編的 librealsense 2.53.1 (/usr/local 下有個空的 pyrealsense2 資料夾會擋住)
export PYTHONPATH=$HOME/librealsense/build/wrappers/python:${PYTHONPATH:-}

EXPO=0
if [ "${1:-}" = "--expo" ]; then EXPO=1; shift; fi
NAME=${1:-rec_$(date +%Y%m%d_%H%M%S)}
[ $EXPO = 1 ] && NAME=expo
BAG=$OUT_DIR/$NAME
LOGS=$OUT_DIR/logs/$NAME
if [ $EXPO = 0 ] && [ -e "$BAG" ]; then echo "[ERR] $BAG 已存在"; exit 1; fi
mkdir -p "$LOGS"

PIDS=()   # 每個都是 setsid 開的 process group leader
NAMES=()

start() {  # start <名稱> <pgrep pattern> <cmd...>
  local name=$1 pat=$2; shift 2
  if pgrep -f "$pat" >/dev/null; then
    # 多半是上次中斷留下的 (可能已經壞掉,例如 Livox bind failed),關掉重開
    echo "[..] 發現舊的 $name 還在跑,先關掉再重開"
    pkill -INT -f "$pat"
    for _ in $(seq 50); do pgrep -f "$pat" >/dev/null || break; sleep 0.2; done
    pkill -KILL -f "$pat" 2>/dev/null; sleep 1
  fi
  echo "[..] 啟動 $name  (log: $LOGS/$name.log)"
  setsid env --default-signal=INT "$@" >"$LOGS/$name.log" 2>&1 &
  PIDS+=($!); NAMES+=("$name")
}

stop_group() {  # stop_group <pid> <名稱>
  local pid=$1 name=$2
  # leader (ros2 launch) 掛了但子程序可能還在 (例如 launch 報錯時 container 留下來),整組清掉
  if ! kill -0 "$pid" 2>/dev/null; then kill -INT -- -"$pid" 2>/dev/null; return; fi
  kill -INT -- -"$pid" 2>/dev/null
  for _ in $(seq 50); do kill -0 "$pid" 2>/dev/null || return; sleep 0.2; done
  echo "[!!] $name 10s 沒結束,送 SIGKILL"
  kill -KILL -- -"$pid" 2>/dev/null
}

BAG_PID=
cleanup() {
  trap - EXIT
  trap '' INT TERM   # 收尾中再按 Ctrl+C 不要中斷,不然 driver 會留在背景
  echo
  if [ -n "$BAG_PID" ]; then
    echo "[..] 停止錄製 (等 mcap 寫完)"
    stop_group "$BAG_PID" bag
  fi
  for i in "${!PIDS[@]}"; do
    echo "[..] 關閉 ${NAMES[$i]}"
    stop_group "${PIDS[$i]}" "${NAMES[$i]}"
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# driver 比光達先起來會連不上 (bind failed / 沒 topic),先等光達 ping 得到,不通就不錄
LIDAR_IP=${LIDAR_IP:-192.168.1.110}
if [ $EXPO = 0 ] && ! ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1; then
  echo "[..] 等 MID-360 ($LIDAR_IP) 開機 (最多 60s)"
  for _ in $(seq 60); do ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1 && break; done
  if ! ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1; then
    echo "[ERR] ping 不到 MID-360,不錄。檢查光達電源和網路線:"
    echo "      ip -br addr show enp43s0  要看到 UP 192.168.1.5"
    exit 1
  fi
fi

start zed   "__node:=zed_container" \
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 \
  ros_params_override_path:="$ZED_CFG"

if [ $EXPO = 1 ]; then
  echo "[..] 曝光調整模式 (ZED 設定檔: $ZED_CFG),等 ZED 影像出來再開 rqt"
  # rqt 比 topic 早開會選不到影像,所以先等第一張影像
  python3 "$HERE/sensor_check.py" --timeout 120 --window 0.5 \
    /zed/zed_node/rgb/color/rect/image/compressed >/dev/null \
    || { echo "[ERR] 120s 內沒有 ZED 影像,看 $LOGS/zed.log"; exit 1; }
  start rqt "__node:=zed_expo_view" \
    ros2 run rqt_image_view rqt_image_view /zed/zed_node/rgb/color/rect/image/compressed \
    --ros-args -r __node:=zed_expo_view
  python3 "$HERE/sensor_check.py" --expo
  exit 0
fi

start livox  "lib/livox_ros_driver2/livox_ros_driver2_node" \
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
start t265   "lib/t265_ros/t265_node" \
  ros2 launch t265_ros t265.launch.py

# 外參 -> /tf_static (zed_camera_link -> base_link -> livox_frame / t265_mount)
if [ -f "$EXTR" ]; then
  echo "[..] 外參 ($EXTR):"
  python3 "$HERE/pub_extrinsics.py" "$EXTR" --check
  case $? in
    0) ;;
    2) echo "[!!] 有外參還沒填,那幾筆不會進 bag" ;;
    *) echo "[ERR] 外參檔讀取失敗"; exit 1 ;;
  esac
  cp "$EXTR" "$LOGS/extrinsics.yaml"   # 錄完再複製進 bag 資料夾
  start extr "pub_extrinsics.py" python3 "$HERE/pub_extrinsics.py" "$EXTR"
else
  echo "[!!] 找不到外參檔 $EXTR,bag 裡不會有感測器之間的 TF"
fi

echo "[..] 等所有 topic 有資料 (最多 90s; ZED 第一次開要等深度模型載入)"
python3 "$HERE/sensor_check.py" --timeout 90 "${TOPICS[@]}"
rc=$?
if [ $rc != 0 ]; then
  for i in "${!NAMES[@]}"; do
    echo "----- ${NAMES[$i]} log 最後 8 行 -----"
    tail -n 8 "$LOGS/${NAMES[$i]}.log"
  done
  if [ "${FORCE:-0}" != 1 ]; then
    echo "[ERR] topic 檢查沒過,不錄。(FORCE=1 可強制錄)"
    exit 1
  fi
  echo "[!!] FORCE=1,照樣錄"
fi

echo "[REC] 開始錄 -> $BAG   (Ctrl+C 停止)"
setsid env --default-signal=INT ros2 bag record -s mcap -o "$BAG" "${TOPICS[@]}" >"$LOGS/bag.log" 2>&1 &
BAG_PID=$!

# 錄製中開 rviz2 即時看 ZED 影像 (訂 compressed,不另外拉 raw);NO_VIZ=1 不開,關掉視窗不影響錄製
if [ "${NO_VIZ:-0}" != 1 ] && [ -n "${DISPLAY:-}" ]; then
  start rviz "__node:=rec_zed_view" \
    rviz2 -d "$HERE/zed_view.rviz" --ros-args -r __node:=rec_zed_view
fi

STOP=0
trap 'STOP=1' INT TERM   # 錄製中 Ctrl+C = 正常停止,繼續做錄完檢查
T0=$(date +%s)
while [ $STOP = 0 ]; do
  if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo; echo "[ERR] ros2 bag record 自己結束了,看 $LOGS/bag.log"; break
  fi
  el=$(( $(date +%s) - T0 ))
  printf '\r[REC] %02d:%02d  %s ' $((el/60)) $((el%60)) "$(du -sh "$BAG" 2>/dev/null | cut -f1)"
  if [ -n "${DURATION:-}" ] && [ $el -ge "$DURATION" ]; then break; fi
  sleep 1
done

trap '' INT   # 收尾時不要再被 Ctrl+C 打斷
cleanup
trap - EXIT

# ---- 錄完檢查: 每個 topic 都要有訊息 ----
echo
if [ ! -f "$BAG/metadata.yaml" ]; then echo "[ERR] $BAG 沒有 metadata.yaml (bag 沒正常結束),可試 ros2 bag reindex $BAG"; exit 1; fi
[ -f "$LOGS/extrinsics.yaml" ] && cp "$LOGS/extrinsics.yaml" "$BAG/"
ros2 bag info "$BAG" | grep -E "Duration|Bag size|Messages"
python3 - "$BAG/metadata.yaml" "${TOPICS[@]}" <<'EOF'
import sys, yaml
info = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
dur = info['duration']['nanoseconds'] / 1e9
cnt = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
bad = 0
for t in sys.argv[2:]:
    n = cnt.get(t, 0)
    hz = '' if t == '/tf_static' else f'{n / dur:7.1f} Hz'
    flag = 'MISSING!' if n == 0 else ''
    bad += n == 0
    print(f'{t:50s} {n:8d} {hz} {flag}')
print('\n[OK] 全部 topic 都有錄到' if not bad else f'\n[ERR] {bad} 個 topic 沒資料')
EOF
