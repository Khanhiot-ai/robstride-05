

# robstride-05
control robstride 05
control robstride 0.5

Hướng dẫn Setup Gripper Robstride + CANable V2.0 (ros2 humble)

# Hướng dẫn Setup Gripper Robstride + CANable V2.0

Bước 1 — Cài đặt phần mềm
**Mục tiêu**: Điều khiển 2 motor Robstride 05 qua CAN bus để làm gripper cho UR3,  
với tính năng tự dừng khi chạm vật (contact detection).

**Phần cứng cần có**:
- 2× Robstride 05 motor
- 1× MakerBase CANable V2.0
- PC chạy Ubuntu 22.04 (ROS 2 Humble)
- Nguồn DC 24V cho motor

---

## Bước 1 — Cài đặt phần mềm

```bash
# Cài python-can và robstride SDK
pip install python-can robstride

# Cài can-utils (candump, cansend)

sudo apt install can-utils
```

---

## Bước 2 — Kiểm tra CANable V2.0

Bước 2 — Kiểm tra CANable V2.0
Cắm CANable vào USB, kiểm tra firmware:

bashlsusb | grep -i -E "OpenMoko|MCS|canable"
```bash
lsusb | grep -i -E "OpenMoko|MCS|canable"
```

**Kết quả mong đợi:**

| Firmware | USB ID | Trạng thái |
|---|---|---|
| slcan (mặc định) | `16d0:117e MCS CANable2` | Dùng được, cần setup thêm |
| candleLight | `1d50:606f OpenMoko` | Tốt nhất, dùng luôn |

Nếu thấy `16d0:117e` (slcan) → tiếp tục Bước 3A.  
Nếu thấy `1d50:606f` (candleLight) → nhảy sang Bước 3B.

Kết quả mong đợi:
FirmwareUSB IDTrạng tháislcan (mặc định)16d0:117e MCS CANable2Dùng được, cần setup thêmcandleLight1d50:606f OpenMokoTốt nhất, dùng luôn
Nếu thấy 16d0:117e (slcan) → tiếp tục Bước 3A.
Nếu thấy 1d50:606f (candleLight) → nhảy sang Bước 3B.
---

## Bước 3A — Setup CAN interface (slcan firmware)

Bước 3A — Setup CAN interface (slcan firmware)
Chạy mỗi lần reboot:
bash# Tìm port

```bash
# Tìm port
ls /dev/ttyACM*
# Thường là /dev/ttyACM0
sudo killall slcand 2>/dev/null
sudo ip link set can0 down 2>/dev/null
ls /dev/ttyACM*
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000
ip link show can0          # verify "state UP"



```

Lưu ý: -s8 = 1 Mbps (Robstride mặc định). Phải chạy lại sau mỗi lần reboot.
> **Lưu ý**: `-s8` = 1 Mbps (Robstride mặc định). Phải chạy lại sau mỗi lần reboot.

---

Bước 3B — Setup CAN interface (candleLight firmware)
bashsudo ip link set can0 type can bitrate 1000000
## Bước 3B — Setup CAN interface (candleLight firmware)

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip link show can0
```

---

## Bước 3C — Tạo script tự động (chạy 1 lần)

Bước 3C — Tạo script tự động (chạy 1 lần)
Để không phải gõ lại mỗi lần reboot:
bash# Tạo file

```bash
# Tạo file
cat > ~/start_can.sh << 'EOF'
#!/bin/bash
# Cho slcan firmware:
 +x ~/start_can.sh

# Dùng:
~/start_can.sh
```

---

## Bước 4 — Kiểm tra CAN bus hoạt động

Bước 4 — Kiểm tra CAN bus hoạt động
Mở 2 terminal:
Terminal 1 — lắng nghe:
bashcandump can0
Terminal 2 — gửi test frame:
bashcansend can0 123#DEADBEEF

**Terminal 1 — lắng nghe:**
```bash
candump can0
```

**Terminal 2 — gửi test frame:**
```bash
cansend can0 123#DEADBEEF
```

Terminal 1 phải in ra:
```
can0  123   [4]  DE AD BE EF
```

Nếu thấy → CAN bus OK. Nếu không thấy → kiểm tra lại Bước 3.

Bước 5 — Tìm ID của motor
Cắm từng motor một vào CAN, bật nguồn motor (24V), rồi chạy:
---

## Bước 5 — Tìm ID của motor


python3 << 'EOF'
import can, time
bus = can.Bus(interface='socketcan', channel='can0')
found = []
for motor_id in range(256):
    arb = (2 << 24) | (0xFD << 8) | motor_id
    try:
        bus.send(can.Message(arbitration_id=arb, data=[0]*8, is_extended_id=True))
    except can.CanError:
        continue
    deadline = time.time() + 0.01
    while time.time() < deadline:
        r = bus.recv(timeout=0.005)
        if r is None:
            break
        resp_id = (r.arbitration_id >> 8) & 0xFF
        if resp_id == motor_id and resp_id not in found:
            found.append(resp_id)
print(f"Ket qua: {sorted(set(found))}")
bus.shutdown()
EOF


Bước 6 : chạy file
---

## Bước 6 — Đổi motor ID (nếu cần)

Motor mới xuất xưởng có ID = 127. Cần đổi để 2 motor không xung đột:

```bash
# Cắm CHỈ motor đầu tiên, đổi 127 → 7
python3 - << 'EOF'
import can, struct, time

bus = can.Bus(interface='socketcan', channel='can0')
OLD_ID, NEW_ID = 127, 7

arb  = (7 << 24) | (0xFD << 8) | OLD_ID
data = [0x70, 0x05, 0x00, 0x00, NEW_ID, 0x00, 0x00, 0x00]
bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=True))
print(f"Doi ID {OLD_ID} -> {NEW_ID}")
r = bus.recv(timeout=1.0)
if r: print(f"Response: 0x{r.arbitration_id:06X}")
bus.shutdown()
EOF

# Tháo motor đầu, cắm motor thứ hai, đổi 127 → 6
# (đổi NEW_ID = 6 trong script trên)
```

---

## Bước 7 — Verify motor hoạt động

Cắm cả 2 motor, kiểm tra đọc được state không:

```bash
python3 - << 'EOF'
import can, struct, math, time

bus = can.Bus(interface='socketcan', channel='can0')
P_MIN, P_MAX = -4*math.pi, 4*math.pi

for mid in (7, 6):
    bus.send(can.Message(
        arbitration_id=(2 << 24) | (0xFD << 8) | mid,
        data=[0]*8, is_extended_id=True))
    r = bus.recv(timeout=0.5)
    if r:
        d = bytes(r.data)
        rp = struct.unpack('>H', d[0:2])[0]
        pos = rp / 65535.0 * (P_MAX - P_MIN) + P_MIN
        print(f"Motor {mid}: pos={pos:+.3f} rad  OK")
    else:
        print(f"Motor {mid}: KHONG PHAN HOI - kiem tra CAN wire va nguon")

bus.shutdown()
EOF
```

Phải thấy cả 2 motor trả về vị trí. Nếu không → kiểm tra:
- Dây CAN_H, CAN_L, GND nối đúng chưa
- Nguồn 24V đã bật chưa
- Termination resistor 120Ω ở 2 đầu bus chưa

---

## Bước 8 — Chạy gripper controller

Copy file `control_robstride.py` vào thư mục làm việc, rồi:

```bash
# Chạy mặc định (speed=0.5, threshold=0.15)
python3 control_robstride.py

@@ -113,6 +237,137 @@ python3 control_robstride.py --id1 7 --id2 6 --speed 0.5 --threshold 0.15

# Nếu cơ cấu gripper 2 motor quay ngược chiều nhau
python3 control_robstride.py --reverse
```

---

## Phím tắt khi đang chạy

| Gõ + Enter | Tác dụng |
|---|---|
| `Enter` | Bắt đầu / Dừng |
| `+` | Tăng tốc độ 0.5 rad/s |
| `-` | Giảm tốc độ 0.5 rad/s |
| `r` | Đổi chiều quay (mở gripper) |
| `0` | Về 0 rad/s |
| `1.5` | Set tốc độ = 1.5 rad/s |
| `t0.2` | Đổi threshold = 0.2 N·m |
| `q` | Thoát |

---

## Bước 9 — Tune threshold

Threshold là giá trị torque (N·m) để phát hiện chạm vật.  
Cần đo nhiễu nền trước khi đặt threshold:

```bash
# Chạy với threshold=99 (không bao giờ dừng) để quan sát
python3 control_robstride.py --threshold 99 --speed 0.5
```

Nhìn vào cột `avg` khi motor đang quay tự do (không chạm gì):

| Nhiễu nền avg | Threshold khuyến nghị |
|---|---|
| 0.02 - 0.04 | 0.08 |
| 0.04 - 0.06 | 0.10 |
| 0.06 - 0.08 | 0.12 |
| 0.08 - 0.10 | 0.15 |

Sau đó tác động lực vào motor, xem avg lên đến bao nhiêu khi kẹp.  
Threshold = (nhiễu nền + giá trị kẹp) / 2.

---

## Xử lý lỗi thường gặp

**Lỗi: `No such device can0`**
```bash
# CAN interface chưa được bring up, chạy lại:
~/start_can.sh
```

**Lỗi: `t1=None t2=None`**
```bash
# Motor không trả lời — kiểm tra:
# 1. Nguồn motor đã bật chưa?
# 2. Dây CAN đã cắm đúng CAN_H/CAN_L/GND chưa?
# 3. Motor ID có đúng không? (chạy lại Bước 5)
```

**Motor quay không dừng sau Ctrl+C**
```bash
# Disable motor thủ công:
python3 - << 'EOF'
import can, time
bus = can.Bus(interface='socketcan', channel='can0')
for mid in (7, 6):
    bus.send(can.Message(
        arbitration_id=(4 << 24) | (0xFD << 8) | mid,
        data=[0]*8, is_extended_id=True))
    time.sleep(0.1)
    print(f"Motor {mid} disabled")
bus.shutdown()
EOF
```

**Contact detection trigger sai (dừng khi chưa chạm vật)**
```bash
# Tăng threshold trong lúc chạy: gõ t0.2 rồi Enter
# Hoặc restart với threshold cao hơn:
python3 control_robstride.py --threshold 0.20
```

**Contact detection không trigger (không dừng khi chạm)**
```bash
# Giảm threshold trong lúc chạy: gõ t0.08 rồi Enter
# Hoặc restart với threshold thấp hơn:
python3 control_robstride.py --threshold 0.08
```

---

## Thông số kỹ thuật Robstride 05

| Thông số | Giá trị |
|---|---|
| Điện áp | 15 - 60V DC |
| Peak torque | 5.5 N·m |
| Max speed | 44 rad/s (~420 rpm sau hộp số) |
| Tỷ số giảm tốc | 7.75:1 |
| Encoder | Dual 14-bit magnetic |
| Giao tiếp | CAN 2.0B, 1 Mbps |
| Motor ID mặc định | 127 |
| Khối lượng | 191g |

---

## Cấu trúc CAN frame

**Gửi lệnh tốc độ (Speed mode):**
```
Arbitration ID: (18 << 24) | (0xFD << 8) | motor_id
Data: [0x70, 0x0A, 0x00, 0x00, <float32 tốc độ little-endian>]
```

**Đọc feedback:**
```
Arbitration ID: (2 << 24) | (0xFD << 8) | motor_id
Response ID:    (2 << 24) | (motor_id << 8) | 0xFD
Response data:  [pos_hi, pos_lo, vel_hi, vel_lo, tor_hi, tor_lo, mode, err]
```

**Decode response:**
```python
pos = raw_pos / 65535.0 * (P_MAX - P_MIN) + P_MIN   # rad, P_MIN=-4π, P_MAX=+4π
vel = raw_vel / 65535.0 * 88.0 - 44.0               # rad/s
tor = raw_tor / 65535.0 * 11.0 - 5.5                # N·m
```

---

*Được tạo cho dự án UR3 + Robstride gripper → train π0.5*

# Bring up lại can0
sudo slcand -o -c -s8 /dev/ttyACM0 can0
