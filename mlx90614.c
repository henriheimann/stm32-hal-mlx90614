#include "mlx90614.h"

typedef enum {
	MLX90614_RAM_ADDRESS_TEMPERATURE_AMBIENT = 0x06,
	MLX90614_RAM_ADDRESS_TEMPERATURE_OBJECT_1 = 0x07,
	MLX90614_RAM_ADDRESS_TEMPERATURE_OBJECT_2 = 0x08
} mlx90614_ram_address_t;

uint8_t crc8(uint8_t *data, size_t length)
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

bool mlx90614_read_uint16(mlx90614_handle_t *handle, uint8_t command, uint16_t *value)
{
	uint8_t buffer[3] = {0};

	if (HAL_I2C_Mem_Read(handle->i2c_handle, (handle->device_address << 1u), command, 1, buffer, 3,
					  MLX90614_I2C_TIMEOUT) != HAL_OK) return false;

	*value = buffer[0] | (buffer[1] << 8);

	uint8_t crc_buffer[5] = {
			(handle->device_address << 1), command,
			(handle->device_address << 1) + 1, buffer[0], buffer[1]
	};
	uint8_t crc = crc8(crc_buffer, 5);

	if (crc != buffer[2]) return false;

	return true;
}

float mlx90614_convert_value_to_temperature(uint16_t value)
{
	return (float)value * 0.02f - 273.15f;
}

bool mlx90614_read_ambient_temperature(mlx90614_handle_t *handle, float *temperature)
{
	uint16_t value;
	if (!mlx90614_read_uint16(handle, 0b00000000 | MLX90614_RAM_ADDRESS_TEMPERATURE_AMBIENT, &value)) return false;
	*temperature = mlx90614_convert_value_to_temperature(value);
	return true;
}

bool mlx90614_read_object_temperature(mlx90614_handle_t *handle, float *temperature)
{
	uint16_t value;

	if (!mlx90614_read_uint16(handle, 0b00000000 | MLX90614_RAM_ADDRESS_TEMPERATURE_OBJECT_1, &value)) return false;
	float t1 = mlx90614_convert_value_to_temperature(value);

	if (!mlx90614_read_uint16(handle, 0b00000000 | MLX90614_RAM_ADDRESS_TEMPERATURE_OBJECT_2, &value)) return false;
	float t2 = mlx90614_convert_value_to_temperature(value);

	*temperature = (t1 + t2) / 2;

	return true;
}