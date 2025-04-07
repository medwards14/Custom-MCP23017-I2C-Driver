#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/device.h>

// defining the MCP23017 registers I'll use to define IO direction, enable interrupts, designate pullup resistor config, etc
#define MCP23x17_IODIRA   0x00
#define MCP23x17_IODIRB   0x01
#define MCP23x17_GPINTENA 0x04
#define MCP23x17_GPINTENB 0x05
#define MCP23x17_DEFVALA  0x06
#define MCP23x17_DEFVALB  0x07
#define MCP23x17_INTCONA  0x08
#define MCP23x17_INTCONB  0x09
#define MCP23x17_IOCONA   0x0A
#define MCP23x17_IOCONB   0x0B
#define MCP23x17_GPPUA    0x0C
#define MCP23x17_GPPUB    0x0D
#define MCP23x17_INTFA    0x0E
#define MCP23x17_INTFB    0x0F
#define MCP23x17_INTCAPA  0x10
#define MCP23x17_INTCAPB  0x11
#define MCP23x17_GPIOA    0x12
#define MCP23x17_GPIOB    0x13

#define NUM_LEDS 4

struct led_lottery_i2c {
	// client points to the I2C client structure, and represents the MCP23017 chip on the I2C bus
	struct i2c_client *client;
	// this is an array used to store which 4 of the MCP expander's 16 pins are used for the LEDs
    	unsigned int led_pins[NUM_LEDS];
	// defines which MCP pin is used for the button (I wired it to PA4)
    	unsigned int button_pin;
	// defines which MCP pin is used for the buzzer (PA5)
    	unsigned int buzzer_pin;
	// GPIO pin on the raspberry Pi 5 used for interrupts (I connected INTA to GPIO23)
    	int irq_gpio;
    	int irq_number;
	// pointer to the struct device, used for logging and referencing sysfs
    	struct device *dev;

    	// variables to track the game state with
    	int target_led;
    	int current_led;
    	int game_active;
};

// reads an 8 bit register (reg) from the MCP expander
// modifies the specified bit in that register depending on value
// then writes the updated byte back to the same register
static int mcp23017_set_bit(struct led_lottery_i2c *priv, u8 reg, u8 bit, bool value) {
	s32 oldval = i2c_smbus_read_byte_data(priv->client, reg);

    	if (oldval < 0) {
        	return oldval;
	}

    	u8 newval = oldval;

	if (value) {
        	newval |= (1 << bit);
    	} else {
        	newval &= ~(1 << bit);
	}

    return i2c_smbus_write_byte_data(priv->client, reg, newval);
}

// used to read a single bit from a register, is marked maybe unused so that the compiler doesnt throw an error
static __maybe_unused int mcp23017_get_bit(struct led_lottery_i2c *priv, u8 reg, u8 bit) {
	s32 val = i2c_smbus_read_byte_data(priv->client, reg);

    	if (val < 0) {
        	return val;
	}

    	return !!(val & (1 << bit));
}

// since the MCP has two ports, each with a set of 8 pins, this function is used to determine which port and
// which bit corresponds to any one of the 16 MCP pins
static void pin_to_portbit(unsigned int pin, u8 *port, u8 *bit) {
	if (pin < 8) {
        	*port = 0;
        	*bit = pin;
    	} else {
        	*port = 1;
        	*bit = pin - 8;
    	}
}

// function used for writing a single pin's value to 1 (high) or 0 (low)
static int mcp23017_write_pin(struct led_lottery_i2c *priv, unsigned int pin, bool value) {
    u8 port, bit;
    pin_to_portbit(pin, &port, &bit);

    u8 gpioReg = (port == 0) ? MCP23x17_GPIOA : MCP23x17_GPIOB;

    return mcp23017_set_bit(priv, gpioReg, bit, value);
}

// top half of the threaded interrupt - just returns a message telling the kernel to schedule the threaded handler
static irqreturn_t button_irq_top(int irq, void *dev_id) {
	return IRQ_WAKE_THREAD;
}

// threaded interrupt implementation - runs in a kernel thread and not atomically, so its safe to call
// i2c-specific functions like i2c_smbus
static irqreturn_t button_irq_thread(int irq, void *dev_id) {
	struct led_lottery_i2c *priv = dev_id;

	// dmesg log indicating the interrupt has been detected
    	pr_info("led_lottery_i2c: Button pressed (interrupt, threaded)\n");

	// if the game is inactive - give the user an indication to start one
    	if (!priv->game_active) {
        	pr_info("led_lottery_i2c: No active game. Press 'start' first.\n");
        	goto clear;
    	}

    	// turns off the current led
    	mcp23017_write_pin(priv, priv->led_pins[priv->current_led], false);
	// incrementing to the next led
    	priv->current_led = (priv->current_led + 1) % NUM_LEDS;
	// turning on the next led
    	mcp23017_write_pin(priv, priv->led_pins[priv->current_led], true);
	// showing the user the current led that is lit in dmesg
    	pr_info("led_lottery_i2c: Current LED is %d\n", priv->current_led);

clear:
	// clearing the interrupt by reading INTCAPA or INTCAPB
    	if (priv->button_pin < 8) {
        	i2c_smbus_read_byte_data(priv->client, MCP23x17_INTCAPA);
	} else {
        	i2c_smbus_read_byte_data(priv->client, MCP23x17_INTCAPB);
	}

    	return IRQ_HANDLED;
}

// this and my other command_show function define a read-write attribute command in sysfs
static ssize_t command_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	// identifies the MCP23017 as a device in the i2c bus
	struct led_lottery_i2c *priv = dev_get_drvdata(dev);
	// copies the user's string into command buffer
    	char command[16];

    	if (count > 15) {
        	return -EINVAL;
	}

    	strncpy(command, buf, count);
    	command[count] = '\0';

    	pr_info("led_lottery_i2c: Command=%s\n", command);

	// if the user entered "start"
    	if (!strncmp(command, "start", 5)) {
        	if (priv->game_active) {
            		pr_info("Game already active.\n");
        	} else {
			// randomly pick a target led
            		priv->target_led = get_random_u32() % NUM_LEDS;

			// set the current led to the first (red) led
            		priv->current_led = 0;
			// start the game
            		priv->game_active = 1;

			// turn on the first led
            		mcp23017_write_pin(priv, priv->led_pins[0], true);
            		pr_info("Started game. Target LED=%d\n", priv->target_led);
        	}
    	}
	// if the user entered "guess"
    	else if (!strncmp(command, "guess", 5)) {
        	if (!priv->game_active) {
            		pr_info("No active game.\n");
            		return count;
        	}
		// if the user guessed the led that is the "winning" led
        	if (priv->current_led == priv->target_led) {
			// output a win message
            		pr_info("Correct guess! You win!\n");
			// beep the buzzer to celebrate
            		mcp23017_write_pin(priv, priv->buzzer_pin, true);

			// only beep for half a second
            		msleep(500);
            		mcp23017_write_pin(priv, priv->buzzer_pin, false);
        	} else {
            		pr_info("Incorrect guess. Try again!\n");
        	}

        	// reset the game
        	priv->game_active = 0;

		// turn off all leds
        	for (int i=0; i<NUM_LEDS; i++) {
            		mcp23017_write_pin(priv, priv->led_pins[i], false);
		}

        	mcp23017_write_pin(priv, priv->buzzer_pin, false);
        	pr_info("Game reset.\n");
    	}
	// if the user entered "reset"
    	else if (!strncmp(command, "reset", 5)) {
        	pr_info("Resetting game.\n");
		// set game to inactive
        	priv->game_active = 0;

		// turn off all leds
        	for (int i=0; i<NUM_LEDS; i++) {
            		mcp23017_write_pin(priv, priv->led_pins[i], false);
		}

        	mcp23017_write_pin(priv, priv->buzzer_pin, false);
        	pr_info("Game reset.\n");
    	}
    	else {
        	pr_warn("Unknown command: %s\n", command);
    	}

    	return count;
}

// this function allows the user to receive game status logs from sysfs
// the start, guess, a reset commands all follow the format "echo (command) | sudo tee /sys/bus/i2c/devices/1-0027/command",
// which draw information from sysfs to be displayed in the dmesg window for game status updates
static ssize_t command_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct led_lottery_i2c *priv = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE,
                     "Game active: %d\nCurrent LED: %d\nTarget LED: [hidden]\n",
                     priv->game_active, priv->current_led);
}

// bounds the command functions together by creating a struct device_attribute used in my probe function
static DEVICE_ATTR_RW(command);

// sets the direction and of led and buzzer pin to output - 0 in IODIR - then write these values to the MCP23017 registers
static int mcp23017_config_pins(struct led_lottery_i2c *priv) {
	int ret;
    	u8 iodira = 0xFF;
    	u8 iodirb = 0xFF;

    	// iterating through each led and setting it as output
    	for (int i = 0; i < NUM_LEDS; i++) {
        	unsigned pin = priv->led_pins[i];

        	if (pin < 8) {
            		iodira &= ~(1 << pin);
        	} else {
            		iodirb &= ~(1 << (pin-8));
		}
    	}

    	// setting the buzzer as output
    	if (priv->buzzer_pin < 8) {
        	iodira &= ~(1 << priv->buzzer_pin);
    	} else {
        	iodirb &= ~(1 << (priv->buzzer_pin - 8));
	}

	// writing the updated register values for half of the MCP's pins
    	ret = i2c_smbus_write_byte_data(priv->client, MCP23x17_IODIRA, iodira);

    	if (ret < 0) {
        	return ret;
	}

	// writing the updated register values for the other half of the MCP's pins
    	ret = i2c_smbus_write_byte_data(priv->client, MCP23x17_IODIRB, iodirb);

    	if (ret < 0) {
        	return ret;
	}

	return 0;
}

// allowing interrupt functionality by configuring the MCP so that whenever the button pin is pressed, and its register
// value goes from high to low, the chip sets INTA to low
static int mcp23017_enable_button_irq(struct led_lottery_i2c *priv) {
	if (priv->button_pin > 15) {
        	dev_warn(priv->dev, "Invalid button pin %u\n", priv->button_pin);
        	return 0;
    	}

    	u8 port = (priv->button_pin < 8) ? 0 : 1;
    	u8 bit  = (port == 0) ? priv->button_pin : (priv->button_pin - 8);

    	u8 gpinten_reg = (port == 0) ? MCP23x17_GPINTENA : MCP23x17_GPINTENB;
   	u8 defval_reg  = (port == 0) ? MCP23x17_DEFVALA  : MCP23x17_DEFVALB;
    	u8 intcon_reg  = (port == 0) ? MCP23x17_INTCONA  : MCP23x17_INTCONB;
    	u8 intcap_reg  = (port == 0) ? MCP23x17_INTCAPA  : MCP23x17_INTCAPB;

    	// using GPINTEN to enable interrupt on the given bit
    	s32 oldval = i2c_smbus_read_byte_data(priv->client, gpinten_reg);

    	if (oldval < 0) return oldval;
    	u8 newval = oldval;
    	newval |= (1 << bit);

    	int ret = i2c_smbus_write_byte_data(priv->client, gpinten_reg, newval);
    	if (ret < 0) return ret;

    	// using DEFVAL to set pin value to 1, indicating falling edge
    	oldval = i2c_smbus_read_byte_data(priv->client, defval_reg);
    	if (oldval < 0) return oldval;

    	newval = oldval;
    	newval |= (1 << bit);
    	ret = i2c_smbus_write_byte_data(priv->client, defval_reg, newval);

    	if (ret < 0) return ret;

    	// using INTCON to compare pin's current value to DEFVAL
    	oldval = i2c_smbus_read_byte_data(priv->client, intcon_reg);
    	if (oldval < 0) return oldval;

    	newval = oldval;
    	newval |= (1 << bit);
    	ret = i2c_smbus_write_byte_data(priv->client, intcon_reg, newval);

    	if (ret < 0) return ret;

    	// clearing any pending interrupt
    	i2c_smbus_read_byte_data(priv->client, intcap_reg);

    	dev_info(priv->dev, "Enabled HW IRQ on pin %u (port=%c)\n", priv->button_pin, (port==0?'A':'B'));

    	return 0;
}

// this is the probe function for my MCP23017 driver - the kernel calls this when it detectsan I2C device that matches
// my "myvendor,led-lottery" compatible string I setup in the device tree file
static int led_lottery_i2c_probe_2(struct i2c_client *client, const struct i2c_device_id *id) {
	// dev here is a pointer to the kernel's generic device structure for this I2C client, so we'll use it for logging
	// and referencing device properties
	struct device *dev = &client->dev;
	// np points to the deice tree node for this device
    	struct device_node *np = dev->of_node;
	// priv is where Ill store thi driver's specific data
    	struct led_lottery_i2c *priv;
    	int ret;

	// logging a message in dmesg with the device's address
    	dev_info(dev, "led_lottery_i2c: probe_2 at 0x%x\n", client->addr);
	// allocating zero-initialized priv structure to store values for and track game logic variabes I previously defined,
	// as well as led pin assignments, the i2c_client pointer, ...
    	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);

    	if (!priv) {
        	return -ENOMEM;
	}

	// storing references to the I2C client and device structures inside priv
   	priv->client = client;
    	priv->dev = dev;
	// attaching priv data structure to this device so it can be retrieved later in my remove function and in sysfs via
	// dev_get_drvdata()
    	dev_set_drvdata(dev, priv);

	// reading arrays/properties from the device tree node np
    	of_property_read_u32_array(np, "led-lottery,led-pins", priv->led_pins, NUM_LEDS);
    	of_property_read_u32(np, "led-lottery,button-pin", &priv->button_pin);
    	of_property_read_u32(np, "led-lottery,buzzer-pin", &priv->buzzer_pin);

    	priv->irq_gpio = of_get_named_gpio(np, "interrupt-gpio", 0);

	// looking for a GPIO that represents the interrupt line on the Raspberry Pi 5 - what INTA from the MCP expander connects to
	// (i wired INTA to GPIO23)
    	if (priv->irq_gpio < 0) {
        	dev_warn(dev, "No interrupt-gpio found, will poll.\n");
    	} else {
		// requesting ownership of the Pi's GPIO line
        	gpio_request(priv->irq_gpio, "mcp23017-irq");
		// making sure it is set as input
        	gpio_direction_input(priv->irq_gpio);

		// now converting the GPIO number into a system-wide IRQ number that allows the kernel to track interrupts
        	priv->irq_number = gpio_to_irq(priv->irq_gpio);
        	if (priv->irq_number < 0) {
            		dev_err(dev, "Failed to get irq for INTA\n");
        	} else {
            		// here I actually request the threaded interrupt request for the found GPIO line
			// button_irq_top is the top-half / fast interrupt handler, while button_irq_thread is the threaded handler that
			// can safely sleep or do I2C operations
            		ret = request_threaded_irq(priv->irq_number, button_irq_top, button_irq_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "mcp_lott_irq", priv);

			// is the threaded interrupt request fails, log an error
            		if (ret) {
                		dev_err(dev, "Failed to request irq: %d\n", ret);
            		}
        	}
    	}

	// configuring which MCP23017 pins are inputs vs outputs
    	ret = mcp23017_config_pins(priv);

    	if (ret < 0) {
        	dev_err(dev, "Failed config pins: %d\n", ret);
        	return ret;
    	}

	// enabling the hardware interrupt on the button pin (PA4) within the MCP itself
    	ret = mcp23017_enable_button_irq(priv);
    	if (ret < 0)
        	dev_warn(dev, "button irq config failed: %d\n", ret);

    	// creating a sysfs attribute file named command so the commands "start", "guess", and "reset"  can be written from user space
	// this calls either my command_store() or command_show() function from the driver
    	ret = device_create_file(dev, &dev_attr_command);
    	if (ret) {
        	dev_err(dev, "Failed to create sysfs: %d\n", ret);
        	return ret;
    	}

    	dev_info(dev, "led_lottery_i2c: driver init OK\n");
    	return 0;
}

// this remove function is called when the driver kernel module gets unloaded from the kernel
// basically undoes everything I did in the probe function
static void led_lottery_i2c_remove_2(struct i2c_client *client) {
	// fetches the private data structure for the driver from this MCP's I2C client
	// same priv i stored via dev_set_drvdata() in the probe function
	struct led_lottery_i2c *priv = i2c_get_clientdata(client);

	// removing the command sysfs file I created in probe
    	device_remove_file(&client->dev, &dev_attr_command);
	// freeing the requested interrupt
    	if (priv->irq_number > 0) {
        	free_irq(priv->irq_number, priv);
	}
	// freeing the GPIO line used for the interrupt
    	if (priv->irq_gpio >= 0) {
        	gpio_free(priv->irq_gpio);
	}

    	dev_info(&client->dev, "led_lottery_i2c: removed\n");
}

// wrapper function to satisfy the kernel's desire to to have a certain probe signature
static int led_lottery_i2c_probe_1(struct i2c_client *client) {
	return led_lottery_i2c_probe_2(client, NULL);
}

// similar wrapper function for remove
static void led_lottery_i2c_remove_1(struct i2c_client *client) {
	led_lottery_i2c_remove_2(client);
}

// craeting an ID table for I2C device matching
static const struct i2c_device_id mcp_lott_id[] = {
	{ "mcp_lott", 0 },
    	{ }
};

MODULE_DEVICE_TABLE(i2c, mcp_lott_id);

// table for matching devices via device tree
static const struct of_device_id mcp_lott_of_match[] = {
	{ .compatible = "myvendor,led-lottery" },
    	{ }
};

MODULE_DEVICE_TABLE(of, mcp_lott_of_match);

// main i2c_driver structure, tells the kernel how to handle devices taht match my created driver ID or driver strings
static struct i2c_driver mcp_lott_i2c_driver = {
	.driver = {
        	.name = "mcp_lott",
        	.of_match_table = mcp_lott_of_match,
    	},
    	.probe = led_lottery_i2c_probe_1,
    	.remove = led_lottery_i2c_remove_1,
    	.id_table = mcp_lott_id,
};

// sets up the driver to be automatically registered with the I2C core when module is loaded into the kernel
// and unregistered when it is unloaded
module_i2c_driver(mcp_lott_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mason Edwards + ChatGPT");
MODULE_DESCRIPTION("MCP23017 LED Lottery I2C driver with hardware interrupt (threaded)");
