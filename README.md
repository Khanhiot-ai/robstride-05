# robstride-05
control robstride 05

Hướng dẫn Setup Gripper Robstride + CANable V2.0 (ros2 humble)


Bước 1 — Cài đặt phần mềm

pip install python-can robstride

# Cài can-utils (candump, cansend)

sudo apt install can-utils

Bước 2 — Kiểm tra CANable V2.0
Cắm CANable vào USB, kiểm tra firmware:

bashlsusb | grep -i -E "OpenMoko|MCS|canable"

Kết quả mong đợi:
FirmwareUSB IDTrạng tháislcan (mặc định)16d0:117e MCS CANable2Dùng được, cần setup thêmcandleLight1d50:606f OpenMokoTốt nhất, dùng luôn
Nếu thấy 16d0:117e (slcan) → tiếp tục Bước 3A.
Nếu thấy 1d50:606f (candleLight) → nhảy sang Bước 3B.

Bước 3A — Setup CAN interface (slcan firmware)
Chạy mỗi lần reboot:
bash# Tìm port
ls /dev/ttyACM*
# Thường là /dev/ttyACM0

# Bring up CAN interface
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

# Kiểm tra
ip link show can0
# Phải thấy: state UP

Lưu ý: -s8 = 1 Mbps (Robstride mặc định). Phải chạy lại sau mỗi lần reboot.


Bước 3B — Setup CAN interface (candleLight firmware)
bashsudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip link show can0

Bước 3C — Tạo script tự động (chạy 1 lần)
Để không phải gõ lại mỗi lần reboot:
bash# Tạo file
cat > ~/start_can.sh << 'EOF'
#!/bin/bash
# Cho slcan firmware:
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000
echo "CAN up: $(ip link show can0 | grep state)"
EOF

chmod +x ~/start_can.sh

# Dùng:
~/start_can.sh

Bước 4 — Kiểm tra CAN bus hoạt động
Mở 2 terminal:
Terminal 1 — lắng nghe:
bashcandump can0
Terminal 2 — gửi test frame:
bashcansend can0 123#DEADBEEF
Terminal 1 phải in ra:
can0  123   [4]  DE AD BE EF
Nếu thấy → CAN bus OK. Nếu không thấy → kiểm tra lại Bước 3.

Bước 5 — Tìm ID của motor
Cắm từng motor một vào CAN, bật nguồn motor (24V), rồi chạy:

python3 - << 'EOF'
import can, time

bus = can.Bus(interface='socketcan', channel='can0')
print("Scanning motor ID 0-255...")
found = []

for motor_id in range(256):
    for comm_type in [0, 1, 3]:
        arb = (comm_type << 24) | (0xFD << 8) | motor_id
        bus.send(can.Message(arbitration_id=arb, data=[0]*8, is_extended_id=True))

    deadline = time.time() + 0.02
    while time.time() < deadline:
        resp = bus.recv(timeout=0.005)
        if resp and resp.arbitration_id != 0x7FE:
            print(f"FOUND! motor_id={motor_id}  resp=0x{resp.arbitration_id:06X}")
            found.append(motor_id)
    if motor_id % 64 == 0:
        print(f"  {motor_id}/255...")

print(f"Ket qua: {list(set(found))}")
bus.shutdown()
EOF

Ghi lại ID của 2 motor (thường là 6 và 7, hoặc 127 nếu chưa đổi).

Bước 6 : chạy file

# Chạy mặc định (speed=0.5, threshold=0.15)
python3 control_robstride.py

# Tùy chỉnh
python3 control_robstride.py --id1 7 --id2 6 --speed 0.5 --threshold 0.15

# Nếu cơ cấu gripper 2 motor quay ngược chiều nhau
python3 control_robstride.py --reverse

# Bring up lại can0
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

# Kiểm tra
ip link show can0

