# AMR - STM32
# udevadm info --query=property --name=/dev/ttyACM1
# ID_VENDOR_ID=1a86
# ID_MODEL_ID=55d4
# ID_SERIAL=WCH.CN_USB_Single_Serial_0002
# ID_SERIAL_SHORT=0002
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="avalue_controller"' >/etc/udev/rules.d/avalue_controller.rules
# RPLIDAR
# udevadm info --query=property --name=/dev/ttyUSB0
# ID_VENDOR_ID=10c4
# ID_MODEL_ID=ea60
# ID_SERIAL=Silicon_Labs_CP2102_USB_to_UART_Bridge_86cc903f6a30a747bcaa2ba29fb234c1
# ID_SERIAL_SHORT=86cc903f6a30a747bcaa2ba29fb234c1
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="86cc903f6a30a747bcaa2ba29fb234c1", MODE:="0777", GROUP:="dialout", SYMLINK+="avalue_rplidar"' >/etc/udev/rules.d/avalue_rplidar.rules
# IMU
# udevadm info --query=property --name=/dev/ttyACM0
# ID_VENDOR_ID=1a86
# ID_MODEL_ID=55d4
# ID_SERIAL=WCH.CN_USB_Single_Serial_0003
# ID_SERIAL_SHORT=0003
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="avalue_imu"' >/etc/udev/rules.d/avalue_imu.rules
# Intel® RealSense™ D435
# udevadm info --query=property --name=/dev/video4
# ID_VENDOR_ID=8086
# ID_MODEL_ID=0b07
echo  'SUBSYSTEM=="video4linux", KERNEL=="video4", ATTR{name}=="Intel(R) RealSense(TM) Depth Ca", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", SYMLINK+="avalue_RgbCam"' >/etc/udev/rules.d/avalue_RgbCam.rules

service udev reload
sleep 2
service udev restart
