obj-m += mcp_i2c_driver.o

all: module dt
	@echo "Built MCP I2C driver kernel module and overlay"

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

dt: mcp_overlay.dts
	dtc -@ -I dts -O dtb -o mcp_overlay.dtbo mcp_overlay.dts

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f mcp_overlay.dtbo
