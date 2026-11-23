# sensor_rec — ZED2 + T265 + Livox MID-360 錄製

一鍵錄製：`./rec_all.sh <名稱>`（Ctrl+C 停止）。細節見 rec_all.sh 開頭註解。

## 內容
| 路徑 | 說明 |
|---|---|
| `rec_all.sh` | 啟動三顆感測器 → 檢查 13 個 topic → 錄 mcap |
| `zed_set.sh` | 搭配 `rec_all.sh --expo` 即時調曝光，`--save` 寫回設定 |
| `sensor_check.py`, `pub_extrinsics.py`, `extrinsics.yaml` | topic 檢查、外參 → /tf_static |
| `cyclonedds_local.xml`, `zed_view.rviz` | DDS 只走 loopback、錄製時看影像 |
| `zed_cfg/` | ZED 參數（`zed_lock.yaml` 是目前用的，HD720@30 e25/g30/wb4500K）；`~/zed_cfg` 是指到這裡的 symlink |
| `drivers/t265_ros/` | 自寫的 T265 ROS 2 node；`~/t265_ws/src/t265_ros` 是 symlink |
| `patches/` | 上游 driver 的固定 commit (`versions.env`) 和本地修改 |
| `install.sh` | 新筆電一鍵安裝 |

本地修改（patches/）：
- livox_ros_driver2：`MID360_config.json` 光達 IP 改 192.168.1.110
- zed-ros2-wrapper：`common_stereo.yaml` frame 改 `zed_map` / `zed_odom`（避免和 T265 的 TF 衝突）
- librealsense v2.53.1：pybind11 / pyrs_device 編譯修正（T265 只支援到 2.53.1）

## 搬到新筆電
1. 裝 Ubuntu 24.04、NVIDIA driver、CUDA、ROS 2 Jazzy (desktop)。
2. 裝 ZED SDK 5.5.0：安裝檔在 HDD `/mnt/hdd/linux_data/installers/ZED_SDK_Ubuntu24_cuda13.0_tensorrt10.13_v5.5.0.zstd.run`
   （或官網下載同版本），`chmod +x` 後執行。
3. ```bash
   git clone <這個 repo> ~/sensor_rec      # 一定要放 ~/sensor_rec
   ~/sensor_rec/install.sh
   ```
4. 在 `~/.bashrc` 不需要加任何東西，rec_all.sh 會自己 source 三個 ws。
5. ZED 第一次開會下載校正檔、建 AI 模型，要有網路且等久一點。

## 日常同步
這台改了設定（例如 `zed_set.sh --save`、外參）後：
```bash
cd ~/sensor_rec && git add -A && git commit -m "說明" && git push
```
另一台：`cd ~/sensor_rec && git pull`（driver 有改才需要重跑 install.sh）。
