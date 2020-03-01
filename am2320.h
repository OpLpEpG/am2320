/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AM2320_AM2320_H_
#define ZEPHYR_DRIVERS_SENSOR_AM2320_AM2320_H_


struct am2320_data {
	struct device *i2c;
	u16_t humidity;
	u16_t temperature;
};

struct am2320_dev_config {
	u16_t i2c_addr;
};

#endif /*  ZEPHYR_DRIVERS_SENSOR_AM2320_AM2320_H_ */
