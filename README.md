# UR3 + HTC Vive Teleop — Thu data cho Pi0.5 VLA với cảm biến xúc giác DIGIT

Hệ thống teleoperation điều khiển robot **UR3** bằng **HTC Vive Tracker**, thu thập demonstrations để train mô hình **Pi0.5 (Vision-Language-Action)** có tích hợp cảm biến xúc giác **DIGIT v1**.

**Task:** Gắp bánh răng thả vào cột (gear pick-and-place, contact-rich, 2 loại bánh răng to/nhỏ).

🔗 **Repo:** https://github.com/Khanhiot-ai/ur3-vive-pi05
📦 **Dataset:** https://huggingface.co/datasets/qkhanh1/ur3_pick_cube
📚 **Tham khảo:** https://huggingface.co/datasets/lerobot/berkeley_autolab_ur5

---

## 1. Tổng quan hệ thống

```
HTC Vive Tracker  ──►  UR3 (RTDE servoL @100Hz)
       │
       ├──►  2× Realsense D435i (front + wrist)
       ├──►  2× DIGIT v1 tactile (left + right, 60Hz)
       └──►  Robstride gripper (master/slave qua CAN)
                         │
                         ▼
              record_all.py  ──►  HDF5 dataset
                         │
                         ▼
        Convert_hdf5_to_lerobot.py  ──►  LeRobot v2 (train Pi0.5)
```

## 2. Phần cứng

| Thiết bị | Chi tiết |
|---|---|
| Robot | UR3 @ 192.168.1.1 (Remote Control mode), RTDE servoL |
| Camera | 2× Realsense D435i: front + wrist (480×640 RGB) |
| Tracker | HTC Vive Tracker 3.0 + 2 Lighthouse (IDs [2,3]) |
| Tactile | 2× DIGIT v1: D21383 (LEFT), D21384 (RIGHT) — QVGA 320×240 @60fps |
| Gripper | Robstride: ID 7 (master/vô lăng), ID 6 (slave/tay kẹp), CANable2 /dev/ttyACMx |

## 3. Mô hình Pi0.5 — lý do thiết kế data

Pi0.5 có 2 backbone xúc giác đông cứng, mỗi cái cần input khác:

- **DINO** (ước lượng lực): ăn ảnh DIGIT, model tự resize về 224×224.
- **V-JEPA** (phát hiện trượt/slip): ăn cửa sổ 4 frame liên tiếp @60fps (mốc t, t-2, t-4, t-6 ≈ 100ms), độ phân giải **(320 cao × 240 rộng) PORTRAIT** RGB.

→ DIGIT bắt buộc: **60Hz**, **portrait (320,240)**, **forward delta action**.

## 4. Cấu trúc HDF5 (dual-rate)

DIGIT thu ở 60Hz (callback), camera/state/action thu ở 20Hz (tick). Timestamp dùng để align sau.

```
dataset/<task>.hdf5
└── data/
    ├── demo_0/
    │   ├── obs/
    │   │   ├── image           (T20, 480, 640, 3)  uint8   Realsense front @20Hz
    │   │   ├── wrist_image     (T20, 480, 640, 3)  uint8   Realsense wrist @20Hz
    │   │   ├── digit_left      (T60, 320, 240, 3)  uint8   DIGIT trái @60Hz
    │   │   ├── digit_right     (T60, 320, 240, 3)  uint8   DIGIT phải @60Hz
    │   │   ├── digit_left_ts   (T60,)              float64 timestamp @60Hz
    │   │   ├── digit_right_ts  (T60,)              float64
    │   │   ├── state           (T20, 8)            float32 [ee_xyz, quat, gripper]
    │   │   └── timestamp       (T20,)              float64 tick @20Hz
    │   ├── actions             (T20, 7)            float32 forward delta
    │   └── attrs: success, n_frames, fps, robot=UR3, action_convention
    └── demo_1/, demo_2/, ...
```

- **state (8):** `[ee_x, ee_y, ee_z, qx, qy, qz, qw, gripper]`
- **action (7):** `[dx, dy, dz, d_roll, d_pitch, d_yaw, gripper]` — forward delta: `action[t] = pose(t+1) - pose(t)`
- Tỉ lệ T60/T20 ≈ 3 (DIGIT 60Hz / tick 20Hz). Thực đo: 1903/643 = 2.96.

Định dạng tương thích `lerobot/berkeley_autolab_ur5`. Khác berkeley:
- Gripper **liên tục [0,1]** (theo độ dày bánh răng) thay vì 0/1 nhị phân.
- Quaternion **cố định [1,0,0,0]** (task chỉ tịnh tiến, không xoay cổ tay).

## 5. Cài đặt

```bash
# ROS2 Humble + workspace
source /opt/ros/humble/setup.bash
source ~/ur5_teleop_vive/install/setup.bash

# Dependencies
pip install digit-interface evdev h5py opencv-python numpy
pip install ur-rtde            # RTDE robot control

# Quyền DIGIT (mỗi lần khởi động)
sudo chmod 666 /dev/video*

# ── CAN cho gripper (CANable2 dùng slcand) ──
# Tìm port CANable2
ls /dev/ttyACM*
# Thường là /dev/ttyACM0

# Bring up CAN interface (slcand cho CANable2 — -s8 = 1Mbps)
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

# Kiểm tra — phải thấy "state UP"
ip link show can0

# Bàn phím evdev (cho phím Home set origin trên Wayland)
sudo usermod -a -G input $USER   # rồi LOGOUT/LOGIN lại (hoặc reboot)
```

**Lưu ý CAN:**
- CANable2 là USB-CAN adapter, dùng `slcand` (serial-line CAN daemon), KHÁC với CAN controller tích hợp (dùng `ip link set can0 type can bitrate`).
- `-s8` = bitrate 1Mbps (khớp Robstride). Bảng: s0=10k, s1=20k, s2=50k, s3=100k, s4=125k, s5=250k, s6=500k, s7=800k, s8=1M.
- Nếu port khác `/dev/ttyACM0` (vd ACM1) → đổi trong lệnh slcand.
- Nếu lỗi "device busy" → `sudo killall slcand` rồi chạy lại.

## 5b. SteamVR headless — track Vive Tracker KHÔNG cần kính VR

Dự án dùng SteamVR để track Vive Tracker, nhưng **không có kính VR (HMD)**. SteamVR mặc định đòi kính → phải bật chế độ "null HMD" (headless) để chạy chỉ với tracker + lighthouse.

### Cài SteamVR

```bash
# Cài Steam (nếu chưa có)
sudo apt install steam
# Mở Steam, đăng nhập → Library → tìm "SteamVR" → Install
```

### Bật chế độ null HMD (không cần kính)

Sửa file cấu hình SteamVR để không đòi kính:

```bash
# 1. Bật null driver trong steamvr.vrsettings
nano ~/.steam/steam/config/steamvr.vrsettings
```

Thêm vào (trong block `"steamvr"`):
```json
{
   "steamvr": {
      "requireHmd": false,
      "forcedDriver": "null",
      "activateMultipleDrivers": true
   },
   "driver_null": {
      "enable": true,
      "serialNumber": "Null Headset 1",
      "modelNumber": "Null Headset",
      "windowX": 0,
      "windowY": 0,
      "windowWidth": 1920,
      "windowHeight": 1080,
      "renderWidth": 1920,
      "renderHeight": 1080,
      "secondsFromVsyncToPhotons": 0.011,
      "displayFrequency": 60.0
   }
}
```

```bash
# 2. Bật null driver trong default.vrsettings của driver_null
sudo nano ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings
```
Đổi `"enable": false` → `"enable": true`.

### Chạy SteamVR headless

```bash
# Cách 1: chạy qua Steam (mở app SteamVR — bỏ qua cảnh báo không có HMD)
# Library → SteamVR → Run

# Cách 2: chạy trực tiếp vrserver (headless, không cần Steam UI)
~/.steam/steam/steamapps/common/SteamVR/bin/linux64/vrstartup.sh
```

### Verify tracker được nhận

```bash
# Xem SteamVR có thấy tracker + lighthouse không
~/.steam/steam/steamapps/common/SteamVR/bin/linux64/vrcmd
```
Hoặc mở SteamVR status window — phải thấy 1 tracker (xanh) + 2 lighthouse (xanh).

### Lưu ý SteamVR headless

- **Bật SteamVR TRƯỚC khi chạy T1** (`vive_tf_and_joy_ros2.py`). T1 đọc tracker pose qua SteamVR (OpenVR API).
- Đèn tracker: **xanh đứng yên** = track OK; nhấp nháy/xanh dương = chưa track; đỏ = lỗi.
- 2 lighthouse phải bật + tracker trong tầm nhìn.
- Nếu SteamVR báo "Headset Not Detected" → kiểm tra lại `requireHmd: false` và null driver `enable: true`.
- Tracker cần pair lần đầu: SteamVR → Devices → Pair Controller (giữ nút tracker đến khi đèn nhấp nháy).

## 6. Chạy pipeline (8 terminal) — chi tiết từng giai đoạn

⚠️ **Bật SteamVR headless trước (mục 5b)** — T1 cần SteamVR để đọc tracker.

Mở **8 terminal**, mỗi terminal `cd` vào thư mục code và source workspace trước:
```bash
cd ~/ur5_teleop_vive/ur5_teleop_vive/ur5_teleop_vive/thesis_code
source ~/ur5_teleop_vive/install/setup.bash   # hoặc gõ: ur
```

### GIAI ĐOẠN A — Tracker (T1, T2, T3)

**T1 — Vive tracker:**
```bash
python3 vive_tf_and_joy_ros2.py
```
Đợi thấy: `Lighthouse IDs: [2, 3]` và `Tracker ID 1 published as "right_controller"`.
- ⚠️ Nếu không thấy lighthouse/tracker → kiểm tra đèn tracker (xanh = track), 2 lighthouse có sáng không.

**T2 — TF → PoseStamped:**
```bash
python3 frame_as_posestamped_ros2.py
```
Đợi thấy: `Started converting right_controller -> world at 60Hz`.

**T3 — Teleop logic:**
```bash
python3 vive_ur5_teleop_params.py
```
Đợi thấy: `Relative Control — đọc Home từ: AT Translated Set 2 keyboard` và `INIT done - WORLD ALIGNMENT + RELATIVE MODE`.
- Đây là node nhận phím Home (qua evdev) để set origin.

### GIAI ĐOẠN B — Robot + Camera (T4, T5)

**T4 — UR3 RTDE** (⚠️ robot tự về home khi khởi động — đứng xa robot):
```bash
python3 ur_follow_using_class_ros2.py
```
Đợi thấy: `RTDE connected: 192.168.1.1`, `Control loop: 100 Hz`, `SYSTEM READY TO CONTROL`.
- Ban đầu báo `WAITING DATA: Gripper=False, Joy=False` — bình thường, chờ origin set (giai đoạn D).

**T5 — 2 Realsense:**
```bash
./launch_realsense_all.sh
```
Đợi thấy: `CẢ 2 CAMERA ĐANG CHẠY`.
- ⚠️ Nếu chỉ 1 camera lên → kiểm tra USB 3.0 (`rs-enumerate-devices | grep Usb`), tuột cáp front.

### GIAI ĐOẠN C — Gripper + DIGIT (T6, T7)

**T6 — Gripper Robstride:**
```bash
python3.10 control_robstride_ros_without_calip.py --channel can0
```
Sau khi node lên, gõ lần lượt (Enter mỗi lệnh):
```
c   → auto-calip hành trình tay kẹp (slave quay tìm giới hạn)
m   → bật MIMIC (tay kẹp bám theo vô lăng)
o   → (mở vô lăng hết cỡ rồi gõ) chốt mốc pos_norm = 0.0
p   → (kẹp vô lăng hết cỡ rồi gõ) chốt mốc pos_norm = 1.0
```
- ⚠️ Nếu lỗi `Transmit buffer full` → reset CAN (xem Troubleshooting 13).

**T7 — DIGIT publisher (60Hz):**
```bash
python3 digit_publisher_ros2.py
```
Đợi thấy: `LEFT: ~60fps | RIGHT: ~60fps`.
- ⚠️ Nếu `Cannot open video` → `sudo chmod 666 /dev/video*`, rút/cắm lại DIGIT.
- Xem ảnh DIGIT (tùy chọn): `python3 digit_publisher_ros2.py --gui` → ảnh phải DỌC (portrait).

### GIAI ĐOẠN D — Recorder (T8)

**T8 — Recorder:**
```bash
python3 record_all.py --task pick_cube --fps 20
```
GUI hiện lưới 2×2 (FRONT | WRIST | DIGIT_L | DIGIT_R) + status bar 7 chấm.
- Trước khi thu: status bar phải đủ 7 chấm `*` (sáng). Nếu `actual ○` → chưa set origin (giai đoạn E).

### GIAI ĐOẠN E — Set origin + thu

1. Cầm tracker tư thế thoải mái.
2. Bấm phím **HOME** (hoặc **numpad 7**) → T3 in `📍 [Home] Origin set + Robot moving`, robot di chuyển đến tracker.
3. Lúc này status bar recorder đủ 7 chấm `*` → sẵn sàng thu.
4. Xem mục 7 để thu demo.

⚠️ **Bấm Home chỉ set origin khi T1-T4 đang chạy + tracker được track** (pose_callback cần tracker pose).

```
1. Đặt bánh răng ở vị trí mới
2. Cầm tracker (đảm bảo lighthouse track — đèn tracker xanh)
3. Bấm phím HOME (hoặc numpad 7) → chốt origin, robot di chuyển đến tracker
4. SPACE (ở T8) → robot ON + bắt đầu record
5. Di chuyển tracker → kẹp bánh răng → đưa đến cột → thả vào
6. S (thành công) hoặc F (thất bại) → lưu + robot OFF + tự về home
7. Đợi robot về home (~2s), lặp lại từ bước 1
```

**Phím tắt recorder (T8):** `SPACE` = rec/stop, `S` = save success, `F` = save fail, `Q` = quit.

GUI recorder hiện lưới 2×2 (FRONT | WRIST trên, DIGIT_L | DIGIT_R dưới) + status bar 7 chấm: `front wrist digL digR actual target grip`.

## 8. Kiểm tra dataset

```bash
python3 check_hdf5.py                  # kiểm tra schema, dual-rate, portrait, timestamp
python3 check_hdf5.py --demo 0 --save  # lưu ảnh demo_0 ra /tmp/
```

Output cần thấy:
```
✅ obs/digit_left: (T60, 320, 240, 3)
✅ Dual-rate OK: ~3× (DIGIT 60Hz)
✅ Portrait (320,240) — đúng cho V-JEPA
✅ timestamp: digit (60Hz), tick (20Hz)
```

## 9. Convert sang LeRobot + push HuggingFace

```bash
python3.10 Convert_hdf5_to_lerobot.py \
  --src dataset/pick_cube.hdf5 \
  --task "put the gear onto the peg" \
  --fps 20 --skip-failed --overwrite

python3.10 push_to_huggingface.py --repo-id <user>/ur3_pick_cube
```

Sau convert, LeRobot có: `observation.image`, `observation.wrist_image`, `observation.digit_left`, `observation.digit_right`, `observation.state`, `action`. DIGIT 60Hz được map về grid 20Hz theo timestamp (nearest neighbor).

## 10. Files

| File | Mô tả |
|---|---|
| `vive_tf_and_joy_ros2.py` | Đọc Vive tracker, publish TF + teleop enable |
| `frame_as_posestamped_ros2.py` | Convert TF → PoseStamped @60Hz |
| `vive_ur5_teleop_params.py` | Teleop logic, world alignment, set origin (evdev Home key) |
| `ur_follow_using_class_ros2.py` | Điều khiển UR3 qua RTDE, auto home |
| `control_robstride_ros_without_calip.py` | Gripper master/slave qua CAN |
| `digit_publisher_ros2.py` | Publish 2 DIGIT @60Hz |
| `record_all.py` | Recorder HDF5 dual-rate |
| `Convert_hdf5_to_lerobot.py` | Convert HDF5 → LeRobot v2 |
| `check_hdf5.py` | Kiểm tra dataset |
| `launch_realsense_all.sh` | Khởi động 2 Realsense |
| `view_ur5.launch.py` | Xem robot UR3 trong RViz |

---

## 11. CHANGELOG — Các lỗi đã gặp & cách fix (chi tiết)

Phần này ghi lại toàn bộ hành trình debug, để sau này gặp lại biết cách xử lý.

### 11.1. DIGIT thu sai tần số (20Hz thay vì 60Hz) → DUAL-RATE

**Vấn đề:** Ban đầu `record_all.py` lấy mọi tín hiệu trong `_tick()` chạy 20Hz, kể cả DIGIT — chỉ append frame DIGIT mới nhất mỗi 50ms → mất 2/3 số frame. V-JEPA cần đủ 60fps để dựng cửa sổ 100ms, thu 20Hz là hỏng nhánh slip.

**Fix:** Kiến trúc dual-rate:
- DIGIT thu ở **callback** (`_cb_digit_l/r`) khi đang recording → giữ đủ 60Hz vào buffer riêng + timestamp.
- Camera/state/action thu ở **tick** 20Hz + timestamp riêng.
- Dùng chung ROS clock cho mọi timestamp.
- HDF5 lưu 2 nhịp khác độ dài (T60 ≈ 3×T20). Converter map 60→20Hz bằng nearest timestamp.

**Verify:** `check_hdf5.py` báo `Dual-rate OK: 1903/643 = 2.96×`.

### 11.2. DIGIT sai chiều (landscape vs portrait)

**Vấn đề ban đầu (giả định sai):** Tưởng DIGIT v1 trả landscape (240,320), nên thêm `cv2.rotate(frame, ROTATE_90_CLOCKWISE)` trong publisher để xoay thành portrait.

**Phát hiện thực tế:** Test trực tiếp cho thấy DIGIT của máy này trả frame **ĐÃ portrait sẵn (320,240)**. Lệnh xoay 90° lại làm nó thành landscape (240,320) — ngược lại điều mong muốn! Đó là lý do 11 demo đầu bị landscape.

```
Frame GỐC từ DIGIT: (320, 240, 3)  ← đã portrait
Sau cv2.rotate 90°: (240, 320, 3)  ← thành landscape (SAI)
```

**Fix:** Đặt `ROTATE_DIR = None` trong `digit_publisher_ros2.py` → giữ nguyên frame portrait gốc. Loop chỉ xoay khi `ROTATE_DIR is not None`.

**Bài học:** Luôn kiểm tra shape thật bằng `d.get_frame().shape` trước khi quyết định xoay, đừng tin GUI (GUI resize làm méo nhìn nhầm).

### 11.3. Action convention (backward → forward delta)

**Vấn đề:** Code cũ tính `action[t] = actual[t] - actual[t-1]` (backward delta — robot đã đi tới đâu). Convention BC chuẩn là **forward**: `action[t] = pose(t+1) - pose(t)` (từ state hiện tại đi tiếp). Sai → policy học dự đoán motion quá khứ.

**Fix:** Lưu pose thô mỗi tick, tính forward delta khi save_episode. Frame cuối không có t+1 → delta xyz/rpy = 0, gripper giữ nguyên. Ghi attr `action_convention`.

### 11.4. Bỏ tactile_state (torque thô)

**Vấn đề:** HDF5 lưu `tactile_state` = torque gripper (1D). Pi0.5 không dùng field này (tín hiệu tactile thật từ ảnh DIGIT).

**Fix:** Bỏ ghi `tactile_state` vào HDF5. Torque vẫn đọc trong `control_robstride` để gripper kẹp vật (detect contact), chỉ không lưu file. Gripper [0,1] vẫn ở state[7] + action[6].

### 11.5. Lỗi DIGIT "Cannot open video device"

**Vấn đề:** D21384 báo `Cannot open /dev/video18`. DIGIT đổi `/dev/video` node mỗi lần cắm, đôi khi nhảy số cao (18) không mở được.

**Fix:**
```bash
sudo chmod 666 /dev/video*
# Nếu vẫn lỗi → rút/cắm lại cáp DIGIT, chmod lại
python3 -c "from digit_interface import DigitHandler; print(DigitHandler.list_digits())"
```

### 11.6. Gripper "Transmit buffer full" (CAN)

**Vấn đề:** `can.exceptions.CanOperationError: Transmit buffer full` — CAN bus nghẽn, gửi lệnh nhanh hơn bus xử lý (thường do motor mất kết nối, lệnh dồn lại).

**Fix:** Reset CAN + tăng buffer:
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

### 11.7. Recorder thấy `actual ○` / `target ○` (không nhận pose robot)

**Vấn đề:** Status bar recorder báo `actual ○` — không nhận `/ur_actual_pose`. ur_follow (T4) báo `WAITING DATA: Gripper=False, Joy=False` → không publish pose.

**Nguyên nhân chuỗi:** ur_follow chờ đủ input (Joy + Gripper) trước khi chạy control loop. Joy đến từ tracker → cần origin được set → cần pose_callback chạy → cần tracker được track.

**Fix:** Chạy đủ T1-T4, tracker track, set origin (Home) → ur_follow đủ data → publish pose.

### 11.8. Lỗi ROS "Failed to find a free participant index"

**Vấn đề:** Mở quá nhiều node + lệnh `ros2 topic echo` cùng lúc → cạn participant index domain 0 → node mới không tạo được.

**Fix:** Đóng bớt lệnh `ros2 topic echo`/`ros2 node list` thừa. Hoặc tăng giới hạn qua CycloneDDS config (`MaxAutoParticipantIndex`). Debug bằng cách nhìn trực tiếp terminal thay vì echo.

### 11.9. bashrc lỗi "not found" workspace cũ

**Vấn đề:** Mở terminal báo:
```
not found: /home/khanh/ur3_vive_ws/install/local_setup.bash
not found: /home/khanh/ros2_ws/install/local_setup.bash
```

**Nguyên nhân:** `~/.bashrc` còn dòng source 2 workspace cũ đã xóa. Thêm nữa, file `ur5_teleop_vive/install/setup.bash` (chain underlay) nhớ lại 2 workspace này lúc build.

**Fix nhanh (tạo workspace rỗng):**
```bash
mkdir -p ~/ur3_vive_ws/install ~/ros2_ws/install
touch ~/ur3_vive_ws/install/local_setup.bash ~/ros2_ws/install/local_setup.bash
```
Hoặc xóa dòng source cũ trong bashrc:
```bash
sed -i '/ur3_vive_ws\/install\/setup.bash/d; /ros2_ws\/install\/setup.bash/d' ~/.bashrc
```

### 11.10. Phím Home không bấm được (Wayland + pynput) — fix khó nhất

**Vấn đề:** Bấm Home để set origin không có tác dụng. Terminal hiện `^[[H` (escape sequence của Home đi vào terminal, pynput không bắt được).

**Quá trình debug:**
1. **Nghi Wayland:** `echo $XDG_SESSION_TYPE` → `wayland`. pynput notoriously không bắt phím trên Wayland.
2. **Thử đổi X11:** Đăng nhập "Ubuntu on Xorg" → desktop không vào được. Bỏ.
3. **Thử evdev + group input:** `pip install evdev` + `usermod -a -G input` + reboot → vào được group input, nhưng pynput vẫn fail.
4. **Test evdev trực tiếp:** liệt kê `/dev/input/event*` — phát hiện pynput đọc nhầm device chuột "Logitech G102 Mouse Keyboard", không phải bàn phím thật.
5. **Tìm bàn phím thật:** `/dev/input/event4` → "AT Translated Set 2 keyboard".
6. **Phát hiện mã phím:** Test evdev event4, bấm Home → in `KEY_KP7` (numpad 7), KHÔNG phải `KEY_HOME`! Phím Home laptop = numpad 7 khi NumLock off. pynput map `Key.home` sang mã khác → không khớp.

**Fix triệt để:** Thay listener pynput bằng **evdev** đọc thẳng device bàn phím:
- Tự tìm đúng bàn phím (có phím chữ A + Home/KP7, bỏ qua device tên "mouse").
- Bắt **cả KEY_HOME lẫn KEY_KP7** → phím Home hoạt động.
- Chạy trên Wayland (evdev đọc thẳng kernel, không qua Wayland compositor).

**Lưu ý:** `^[[H` vẫn lọt vào terminal là bình thường (evdev đọc thêm, không chặn terminal). Quan trọng là evdev set `home_pressed=True`. Phím Home chỉ set origin khi T1-T4 chạy + tracker được track (pose_callback xử lý).

### 11.11. Converter chưa có DIGIT

**Vấn đề:** `Convert_hdf5_to_lerobot.py` ban đầu không đưa DIGIT vào LeRobot — train sẽ thiếu tactile.

**Fix:** Thêm đọc digit_left/right + timestamp, map 60Hz→20Hz bằng nearest timestamp, encode 2 video MP4, thêm vào parquet + info.json features. Cờ `dataset_has_digit` tự bật khi có DIGIT, shape lấy thật từ data.

---

## 12. Ghi chú kỹ thuật

- **Xem RViz:** `ros2 launch view_ur5.launch.py` (cần T4 chạy publish `/joint_states`). RViz hoạt động với RTDE — KHÔNG cần URBasic. RViz hiển thị robot dựa vào `/joint_states`, nguồn nào (URBasic hay RTDE) cũng được.
- **RTDE vs URBasic:** RTDE (500Hz real-time, servoL mượt) đúng cho teleop. URBasic (~125Hz, socket URScript, dễ I/O nhưng chậm) phù hợp tác vụ đơn giản không real-time.
- **Phím Home = KEY_KP7:** phím Home laptop là numpad 7 khi NumLock off. evdev bắt cả KEY_HOME + KEY_KP7.
- **World alignment:** `world_alignment_matrix.txt` (góc xoay ~-30.49°, std 0.043°). Không recalib trừ khi lighthouse/robot dời.
- **Folder "ur5":** robot thật là UR3, tên folder để "ur5" cho tiện (không đổi tên).
- **DIGIT portrait:** DIGIT v1 (máy này) trả frame (320,240) portrait sẵn → KHÔNG xoay (`ROTATE_DIR = None`).
- **Video codec:** converter dùng mp4v — train được nhưng KHÔNG xem trên web (cần h264/avc1 nếu muốn xem LeRobot visualizer).

## 13. Troubleshooting nhanh

| Lỗi | Fix |
|---|---|
| DIGIT "Cannot open video" | `sudo chmod 666 /dev/video*`, rút/cắm lại |
| Gripper "Transmit buffer full" | Reset CAN (slcand) + `txqueuelen 1000` — xem dưới |
| Home không set origin | Cần T1-T4 chạy + tracker track (pose_callback cần tracker pose) |
| `^[[H` khi bấm Home | Bình thường với evdev — kiểm tra T3 có in "Origin set" khi chạy đủ pipeline |
| `actual ○` ở recorder | T4 ur_follow chưa publish, hoặc tracker chưa track |
| Participant index full | Đóng bớt `ros2 topic echo` thừa |
| bashrc "not found" | Xóa dòng source workspace cũ, hoặc tạo workspace rỗng |
| DIGIT landscape | DIGIT này portrait sẵn → đặt `ROTATE_DIR = None` |
| SteamVR "Headset Not Detected" | Set `requireHmd: false` + null driver `enable: true` (mục 5b) |
| Tracker không track (T1 không thấy) | Bật SteamVR headless trước; kiểm tra đèn tracker xanh + 2 lighthouse |

**Reset CAN khi lỗi (CANable2 / slcand):**
```bash
sudo killall slcand 2>/dev/null          # kill daemon cũ nếu device busy
sudo ip link set can0 down 2>/dev/null
ls /dev/ttyACM*                          # tìm lại port
sudo slcand -o -c -s8 /dev/ttyACM0 can0  # -s8 = 1Mbps
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000
ip link show can0                        # verify "state UP"
candump can0                             # (tùy chọn) xem motor có phản hồi không
```

---

## 14. Trạng thái hiện tại

- ✅ Pipeline thu data hoàn chỉnh (8 terminal, dual-rate 60Hz DIGIT)
- ✅ HDF5 schema đúng (portrait, forward delta, timestamp)
- ✅ Converter đưa DIGIT vào LeRobot
- ✅ Phím Home fix qua evdev (KEY_KP7)
- ⏳ Thu bộ data thật (portrait, demo ngắn 5-15s, 100-200 demo)
- ⏳ Train Pi0.5
