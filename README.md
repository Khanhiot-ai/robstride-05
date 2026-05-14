# robstride-05
control robstride 05

# Bring up lại can0
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

# Kiểm tra
ip link show can0

