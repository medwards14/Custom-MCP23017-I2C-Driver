#README.md

build driver module:
make clean
make

verify device tree overlay is in the boot folder:
ls /boot/overlays/mcp_overlay.dtbo
	if not:
	sudo cp mcp_overlay.dtbo /boot/overlays/

insert the compiled module into the kernel:
sudo insmod /home/i2c_driver/mcp_i2c_driver.ko

play the game:
echo start | sudo tee /sys/bus/i2c/devices/1-0027/command
echo guess | sudo tee /sys/bus/i2c/devices/1-0027/command
echo reset | sudo tee /sys/bus/i2c/devices/1-0027/command

remove module from the kernel:
sudo rmmod /home/i2c_driver/mcp_i2c_driver.ko
