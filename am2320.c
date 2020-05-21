/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/i2c.h>
#include "stm32f1xx_ll_i2c.h"
#include <drivers/sensor.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>

#include "am2320.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(AM2320);

#define DT_INST_0_TI_AM2320_BUS_NAME "I2C_1"
#define DT_INST_0_TI_AM2320_LABEL "AM2320"

static int AM2320_reg_read(struct device *dev, u8_t reg, u8_t count, u16_t *val)
{
	struct am2320_data *drv_data = dev->driver_data;
	const struct am2320_dev_config *cfg = dev->config->config_info;
    
	u8_t res = 0;
	u8_t b[3], read[32];
	b[0] = 0; 
    i2c_write(drv_data->i2c, b, 1, cfg->i2c_addr);
    k_sleep(10); 
	b[0] = 3;
	b[1] = reg; 
	b[2]= count;
    if (i2c_write(drv_data->i2c, b, 3, cfg->i2c_addr) < 0) 
    {
		res = -EIO;
    }
    k_sleep(2);
    if (i2c_read(drv_data->i2c, read, 1+1+count+2, cfg->i2c_addr) < 0) 
    {
		res = -EIO;
    }
    else LOG_HEXDUMP_DBG(read, 1+1+count+2, "[AM] am2320");

	for(u8_t i = 0; i < count/2; i++)
	{
     *val++ = sys_be16_to_cpu(*(u16_t*) &read[2+i*2]);
	}
	return res;
}

/**
 * @brief Check the Device ID
 *
 * @param[in]   dev     Pointer to the device structure
 *
 * @retval 0 On success
 * @retval -EIO Otherwise
 */
static inline int AM2320_device_id_check(struct device *dev)
{
	u16_t value[2];

	if (AM2320_reg_read(dev, 0, 4, (u16_t*) &value[0]) != 0) {
		LOG_ERR("%s: Failed to get Device ID register!",
			DT_INST_0_TI_AM2320_LABEL);
		return -EIO;
	}

	// if (value != AM2320_DEVICE_ID)  {
		// LOG_ERR("%s: Failed to match the device IDs!",
			// DT_INST_0_TI_AM2320_LABEL);
		// return -EINVAL;
	// }

	return 0;
}

static int AM2320_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct am2320_data *drv_data = dev->driver_data;
	u16_t value[2];
	int rc;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_AMBIENT_TEMP);

	/* clear sensor values */
	drv_data->humidity = 0U;
	drv_data->temperature = 0U;

	/* Get the most recent temperature measurement */
	rc = AM2320_reg_read(dev, 0, 4, (u16_t*) &value[0]);
	if (rc < 0) {
		LOG_ERR("%s: Failed to read from TEMP register!",
			DT_INST_0_TI_AM2320_LABEL);
		return rc;
	}

	/* store measurements to the driver */
	drv_data->humidity = value[0];
	drv_data->temperature = value[1];

	return 0;
}

static int AM2320_channel_get(struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct am2320_data *drv_data = dev->driver_data;
	s32_t tmp;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP)	
	{
		tmp = (s16_t)drv_data->temperature;
		val->val1 = tmp / 10; /* uCelsius */
		val->val2 = tmp % 10 * 100000;
		return 0;
	}
	else if (chan == SENSOR_CHAN_HUMIDITY) 
	{
		tmp = (s16_t)drv_data->humidity;
		val->val1 = tmp / 10; 
		val->val2 = tmp % 10 * 100000;
		return 0;
	}
	else return -ENOTSUP; 
	/*
	 * See datasheet "Temperature Results and Limits" section for more
	 * details on processing sample data.
	 */
}

static const struct sensor_driver_api AM2320_driver_api = {
	.sample_fetch = AM2320_sample_fetch,
	.channel_get = AM2320_channel_get
};

#define __HAL_RCC_I2C1_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST))
#define __HAL_RCC_I2C1_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST))

static int AM2320_init(struct device *dev)
{
	struct am2320_data *drv_data = dev->driver_data;

	/* Bind to the I2C bus that the sensor is connected */
	drv_data->i2c = device_get_binding(DT_INST_0_TI_AM2320_BUS_NAME);
	if (!drv_data->i2c) {
		LOG_ERR("Cannot bind to %s device!", DT_INST_0_TI_AM2320_BUS_NAME);
		return -EINVAL;
	}
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        LL_I2C_Disable(I2C1);
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        LL_I2C_Enable(I2C1);
        LOG_ERR("LL_I2C_IsActiveFlag_BUSY");
        k_sleep(1000);
	}
    if (i2c_configure(drv_data->i2c, I2C_SPEED_FAST | I2C_MODE_MASTER) < 0)
    {
        LOG_ERR("i2c_configure");
    }//*/

	/* Check the Device ID */
	// rc = AM2320_device_id_check(dev);
	// if (rc < 0) {
		// return rc;
	// }

	return 0;
}

static struct am2320_data AM2320_data;

static const struct am2320_dev_config AM2320_config = {
	.i2c_addr = 0xB8>>1
};

DEVICE_AND_API_INIT(am2320, DT_INST_0_TI_AM2320_LABEL, AM2320_init,
		    &AM2320_data, &AM2320_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &AM2320_driver_api);
