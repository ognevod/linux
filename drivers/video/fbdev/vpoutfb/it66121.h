
#ifndef IT66121_H
#define IT66121_H

#include <linux/fb.h>
#include <linux/of.h>
#include <linux/gpio.h>

struct it66121_device_data {
	struct i2c_client	*client;
	struct gpio_desc	*gpio_reset;
};

void it66121_probe(struct it66121_device_data *devdata,
		   struct device_node *output_node);
int it66121_init(struct it66121_device_data *devdata, struct fb_info *info);
void it66121_remove(struct it66121_device_data *devdata);
void it66121_reset(struct it66121_device_data *devdata);
#endif /* IT66121_H */
