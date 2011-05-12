stty -F /dev/ttyUSB0 1000000 line 0 cs8 -onlcr -echo raw min 100 time 2
echo -e -n "C\rF\rS8\rO\r" > /dev/ttyUSB0
sudo ~/socketcan/can-utils/slcand ttyUSB0 can0
sudo ifconfig can0 up

#peak can usb set speed echo "i 0x0014 e" > /dev/pcan32
