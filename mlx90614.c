#include <math.h>
#include <assert.h>
#include "mlx90614.h"

typedef enum {
	MLX90614_COMMAND_RAM_RAW_IR_CHANNEL_1 = 0x04,
	MLX90614_COMMAND_RAM_RAW_IR_CHANNEL_2 = 0x05,
	MLX90614_COMMAND_RAM_TEMPERATURE_AMBIENT = 0x06,
	MLX90614_COMMAND_RAM_TEMPERATURE_OBJECT_1 = 0x07,
	MLX90614_COMMAND_RAM_TEMPERATURE_OBJECT_2 = 0x08,
	MLX90614_COMMAND_EEPROM_TO_MAX = 0x20,
	MLX90614_COMMAND_EEPROM_TO_MIN = 0x21,
	MLX90614_COMMAND_EEPROM_PWMCTRL = 0x22,
	MLX90614_COMMAND_EEPROM_TA_RANGE = 0x23,
	MLX90614_COMMAND_EEPROM_EMISSIVITY = 0x24,
	MLX90614_COMMAND_EEPROM_CONFIG_REGISTER_1 = 0x25,
	MLX90614_COMMAND_EEPROM_SMBUS_ADDRESS = 0x2e,
	MLX90614_COMMAND_EEPROM_ID_NUMBER_1 = 0x3c,
	MLX90614_COMMAND_EEPROM_ID_NUMBER_2 = 0x3d,
	MLX90614_COMMAND_EEPROM_ID_NUMBER_3 = 0x3e,
	MLX90614_COMMAND_EEPROM_ID_NUMBER_4 = 0x3f,
	MLX90614_COMMAND_READ_FLAGS = 0xf0,
	MLX90614_COMMAND_SLEEP = 0xff,
} mlx90614_command_t;

typedef enum {
	MLX90614_FLAG_DEVICE_READY = 0x04,
	MLX90614_FLAG_EEPROM_DEAD = 0x05,
	MLX90614_FLAG_EEPROM_BUSY = 0x07
} mlx90614_flag_t;

static uint8_t crc8(uint8_t *data, size_t length)
{
	uint8_t crc = 0x00;

	while (length--) {
		crc ^= *data++;
		for (uint8_t i = 0; i < 8; i++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ MLX90614_CRC8POLY;
			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

static bool mlx90614_read_uint16(mlx90614_handle_t *handle, mlx90614_command_t command, uint16_t *value)
{
	uint8_t buffer[3] = {0};

	if (HAL_I2C_Mem_Read(handle->i2c_handle, (handle->device_address << 1u), command, 1, buffer, sizeof(buffer),
					  MLX90614_I2C_TIMEOUT) != HAL_OK) return false;

	*value = buffer[0] | (buffer[1] << 8);

	uint8_t crc_buffer[5] = {
			(handle->device_address << 1), command,
			(handle->device_address << 1) + 1, buffer[0], buffer[1]
	};
	uint8_t crc = crc8(crc_buffer, sizeof(crc_buffer));

	if (crc != buffer[2]) return false;

	return true;
}

static bool mlx90614_write_uint16(mlx90614_handle_t *handle, mlx90614_command_t command, uint16_t value)
{
	uint8_t buffer[3] = {0};
	buffer[0] = value & 0xff;
	buffer[1] = (value >> 8) & 0xff;

	uint8_t crc_buffer[4] = {
		(handle->device_address << 1), command, buffer[0], buffer[1]
	};
	buffer[2] = crc8(crc_buffer, sizeof(crc_buffer));

	return HAL_I2C_Mem_Write(handle->i2c_handle, (handle->device_address << 1u), command, 1, buffer, sizeof(buffer),
	                     MLX90614_I2C_TIMEOUT) == HAL_OK;
}

static bool mlx90614_write_eeprom(mlx90614_handle_t *handle, mlx90614_command_t command, uint16_t value)
{
	uint16_t current_value = 0;

	if (!mlx90614_read_uint16(handle, command, &current_value)) return false;

	// Only update value if it differs from the currently stored value
	if (value != current_value) {

		// Clear to 0x00
		if (!mlx90614_write_uint16(handle, command, 0x00)) return false;
		HAL_Delay(10);

		if (!mlx90614_write_uint16(handle, command, value)) return false;
		HAL_Delay(10);
	}

	return true;
}

static float mlx90614_convert_value_to_temperature(uint16_t value)
{
	return (float)value * 0.02f - 273.15f;
}

bool mlx90614_configure_emissivity(mlx90614_handle_t *handle, float emissivity)
{
	assert(handle != NULL);
	assert(emissivity >= 0.0);
	assert(emissivity <= 1.0);

	uint16_t emissivity_value = (uint16_t)roundf(65535 * emissivity);
	return mlx90614_write_eeprom(handle, MLX90614_COMMAND_EEPROM_EMISSIVITY, emissivity_value);
}

bool mlx90614_read_ambient_temperature(mlx90614_handle_t *handle, float *temperature)
{
	assert(handle != NULL);
	assert(temperature != NULL);

	uint16_t value;
	if (!mlx90614_read_uint16(handle, MLX90614_COMMAND_RAM_TEMPERATURE_AMBIENT, &value)) return false;
	*temperature = mlx90614_convert_value_to_temperature(value);
	return true;
}

bool mlx90614_read_object_temperature(mlx90614_handle_t *handle, float *temperature)
{
	assert(handle != NULL);
	assert(temperature != NULL);

	uint16_t value;
	if (!mlx90614_read_uint16(handle, MLX90614_COMMAND_RAM_TEMPERATURE_OBJECT_1, &value)) return false;
	*temperature = mlx90614_convert_value_to_temperature(value);
	return true;
}

bool mlx90614_sleep(mlx90614_handle_t *handle)
{
	uint8_t crc_buffer[2] = {
		(handle->device_address << 1), MLX90614_COMMAND_SLEEP
	};
	uint8_t crc = crc8(crc_buffer, sizeof(crc_buffer));

	return HAL_I2C_Mem_Write(handle->i2c_handle, (handle->device_address << 1u), MLX90614_COMMAND_SLEEP, 1, &crc, 1,
	                         MLX90614_I2C_TIMEOUT) == HAL_OK;
}
