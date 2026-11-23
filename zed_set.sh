#!/usr/bin/env bash
# 即時調 ZED 曝光/增益/白平衡 (配合 rec_all.sh --expo 使用)
#   zed_set.sh 25 10          exposure=25 gain=10
#   zed_set.sh 25 10 42       再加白平衡 4200K
#   zed_set.sh --save 25 10 42  調好後寫回 ~/zed_cfg/zed_lock.yaml (下次錄製就用這組)
set -e
ZED_CFG=${ZED_CFG:-$HOME/zed_cfg/zed_lock.yaml}
SAVE=0
if [ "${1:-}" = "--save" ]; then SAVE=1; shift; fi
if [ $# -lt 2 ]; then sed -n '2,5p' "$0"; exit 1; fi
EXP=$1 GAIN=$2 WB=${3:-}

if [ $SAVE = 1 ]; then
  cp "$ZED_CFG" "$ZED_CFG.bak"
  sed -i -E "s/^( *)exposure: .*/\1exposure: $EXP/; s/^( *)gain: .*/\1gain: $GAIN/" "$ZED_CFG"
  [ -n "$WB" ] && sed -i -E "s/^( *)whitebalance_temperature: .*/\1whitebalance_temperature: $WB/" "$ZED_CFG"
  echo "[OK] 已寫入 $ZED_CFG (舊的在 .bak):"; cat "$ZED_CFG"
  exit 0
fi

# 跟 rec_all.sh 用同一個 DDS 設定,不然找不到 /zed/zed_node
set +u; source /opt/ros/jazzy/setup.bash; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$HOME/sensor_rec/cyclonedds_local.xml}
# 直接呼叫 set_parameters 服務 (ros2 param set 會經過 daemon,在這台會卡住)
python3 - "$EXP" "$GAIN" "$WB" <<'EOF'
import sys
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

names = ['video.exposure', 'video.gain', 'video.whitebalance_temperature']
params = [Parameter(n, value=int(v)).to_parameter_msg()
          for n, v in zip(names, sys.argv[1:]) if v]
rclpy.init()
node = rclpy.create_node('zed_set')
cli = node.create_client(SetParameters, '/zed/zed_node/set_parameters')
if not cli.wait_for_service(timeout_sec=5.0):
    sys.exit('[ERR] 找不到 /zed/zed_node,ZED 有開嗎? (rec_all.sh --expo)')
fut = cli.call_async(SetParameters.Request(parameters=params))
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
if not fut.done():
    sys.exit('[ERR] ZED 5 秒沒回應')
for p, r in zip(params, fut.result().results):
    if not r.successful:
        sys.exit(f'[ERR] {p.name}: {r.reason}')
EOF
echo "[OK] exposure=$EXP gain=$GAIN ${WB:+wb=$WB}  (看 --expo 視窗的數字,滿意就 zed_set.sh --save $EXP $GAIN $WB)"
