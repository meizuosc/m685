
#include "sensor.h"
#include "sensor-i2c.h"


int sensor_i2c_read_16bits(struct i2c_client *client,
	uint8_t addr, uint16_t *val)
{
	int ret = 0;
	uint8_t buf[2];

	if (client == NULL || val == NULL) {
		pr_err("client or val is NULL, client: 0x%p, val: 0x%p\n",
			client, val);
		ret = -EINVAL;
		goto error;
	}

	buf[0]= addr;

	ret = i2c_master_send(client, buf, 0x1);
	if(ret <= 0) {
		dev_err(&client->dev, "%s i2c_master_send failed\n", __func__);
		goto error;
	}

	ret = i2c_master_recv(client, (char *)val, 0x2);
	if(ret <= 0)
	{
		dev_err(&client->dev, "%s i2c_master_recv failed\n", __func__);
		goto error;
	}

	return 0;

error:
	dev_err(&client->dev, "%s failed, ret: %d\n", __func__, ret);
	return ret;
}

int sensor_i2c_read_8bit(struct i2c_client *client, uint8_t addr, uint8_t *val)
{
	int ret = 0;
	uint8_t buf[2];

	if (client == NULL || val == NULL) {
		pr_err("client or val is NULL, client: 0x%p, val: 0x%p\n",
			client, val);
		ret = -EINVAL;
		goto error;
	}

	buf[0]= addr;

	ret = i2c_master_send(client, buf, 0x1);
	if(ret <= 0) {
		dev_err(&client->dev, "i2c_master_send failed\n");
		goto error;
	}

	ret = i2c_master_recv(client, val, 0x1);
	if(ret <= 0)
	{
		dev_err(&client->dev, "i2c_master_recv failed\n");
		goto error;
	}

	return 0;

error:
	dev_err(&client->dev, "sensor_i2c_read_8bit failed, ret: %d\n", ret);
	return ret;
}

int sensor_i2c_write_8bit(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	u8 buf[2];
	int ret = 0;

	buf[0] = addr;
	buf[1] = val;

	if (client == NULL) {
		pr_err("client is NULL\n");
		ret = -EINVAL;
		goto error;
	}

	ret = i2c_master_send(client, buf, 0x2);
	if (ret < 0){
		dev_err(&client->dev, "i2c_master_send failed\n");
		goto error;
	}

	return 0;

error:
	dev_err(&client->dev, "sensor_i2c_write_8bit failed, ret: %d\n", ret);
	return ret;
}

/*
1. write 8bit value to register
	// set reg 236 to 0x02
	echo "236 > 0x00" > debug

2. write 8bit value to register
	// set reg 0x00 to 0x02
	echo "0x02 > 0x00" > debug

3. write specific bits to register
	// set reg 0x01 bits [6:4] to 0x110
	echo "0b110 > 0x01[6:4]" > debug

 */
int sensor_i2c_set_debug_8bit(struct i2c_client *client,
	const char *buf, size_t count)
{
	int i;
	uint32_t reg;
	uint32_t val;
	uint32_t val_bit = 0;
	uint32_t start_bit;
	uint32_t stop_bit;
	uint8_t tmp;
	char str_bit[50];
	int  str_len;
	memset(str_bit, 0, sizeof(str_bit));

	if (sscanf(buf, "0d%d > 0x%x", &val, &reg) == 2) {

		dev_info(&client->dev,
			"write reg 0x%02X value %d [0x%02X]\n", reg, val, val);
		sensor_i2c_write_8bit(client, reg, val);

	} else if (sscanf(buf, "0x%x > 0x%x", &val, &reg) == 2) {

		dev_info(&client->dev,
			"write reg 0x%02X value 0x%02X [%d]\n", reg, val, val);
		sensor_i2c_write_8bit(client, reg, val);

	} else if (sscanf(buf, "0b%s > 0x%x[%d:%d]",
		str_bit, &reg, &start_bit, &stop_bit) == 4) {

		sensor_i2c_read_8bit(client, reg, &tmp);

		str_len = strlen(str_bit);
		if ((start_bit < stop_bit)
		 || (str_len != (start_bit-stop_bit+1))) {
			dev_err(&client->dev, "Invalid Argument of 0bxxx\n");
			return -EINVAL;
		} else {

			/* clear bits */
			for (i = start_bit; i >= stop_bit; i--) {
				tmp &= ~((uint8_t)(1 << i));
			}

			/* get bit value */
			for (i = 0; i < str_len; i++) {
				val_bit |= (str_bit[i] - '0')
					<< (str_len - i - 1);
			}
		}

		tmp |= val_bit << stop_bit;

		dev_info(&client->dev,
			"write reg 0x%02X[%d:%d] value 0x%02X[0x%x]\n",
			reg, start_bit, stop_bit, tmp, val_bit);
		sensor_i2c_write_8bit(client, reg, tmp);
	} else {
		dev_err(&client->dev, "Invalid Argument\n");
	}

	return 0;
}