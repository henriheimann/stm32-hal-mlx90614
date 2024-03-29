#pragma once

#if (defined STM32L011xx) || (defined STM32L021xx) || \
	(defined STM32L031xx) || (defined STM32L041xx) || \
	(defined STM32L051xx) || (defined STM32L052xx) || (defined STM32L053xx) || \
	(defined STM32L061xx) || (defined STM32L062xx) || (defined STM32L063xx) || \
	(defined STM32L071xx) || (defined STM32L072xx) || (defined STM32L073xx) || \
	(defined STM32L081xx) || (defined STM32L082xx) || (defined STM32L083xx)
#include "stm32l0xx_hal.h"
#elif defined (STM32L412xx) || defined (STM32L422xx) || \
	defined (STM32L431xx) || (defined STM32L432xx) || defined (STM32L433xx) || defined (STM32L442xx) || defined (STM32L443xx) || \
	defined (STM32L451xx) || defined (STM32L452xx) || defined (STM32L462xx) || \
	defined (STM32L471xx) || defined (STM32L475xx) || defined (STM32L476xx) || defined (STM32L485xx) || defined (STM32L486xx) || \
    defined (STM32L496xx) || defined (STM32L4A6xx) || \
    defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#include "stm32l4xx_hal.h"
#else
#error Platform not implemented
#endif

#include <stdbool.h>
#include <stdint.h>

#define MLX90614_DEFAULT_ADDRESS 0x5a

#define MLX90614_CRC8POLY 0x07

/**
 * The timeout used in all I2C transmits.
 */
#ifndef MLX90614_I2C_TIMEOUT
#define MLX90614_I2C_TIMEOUT 100
#endif

/**
 * Structure defining a handle describing an MLX90614 sensor.
 */
typedef struct {

	/**
	 * The handle to the I2C bus used.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The device address of the MLX90614 sensor.
	 */
	uint8_t device_address;

} mlx90614_handle_t;

/**
 * Configures the emissivity to be used during the calculation of the object temperature. Must be in the range between
 * 0.0 and 1.0. The MLX90614 comes with a default configuration of 1.0.
 *
 * @param handle The handle to the MLX90614.
 * @param emissivity The emissivity to configure.
 * @return True on success, false otherwise.
 */
bool mlx90614_configure_emissivity(mlx90614_handle_t *handle, float emissivity);

/**
 * Reads the current ambient temperature.
 *
 * @param handle The handle to the MLX90614.
 * @param temperature Pointer to a memory location to store the temperature.
 * @return True on success, false otherwise.
 */
bool mlx90614_read_ambient_temperature(mlx90614_handle_t *handle, float *temperature);

/**
 * Reads the current object temperature.
 *
 * @param handle The handle to the MLX90614.
 * @param temperature Pointer to a memory location to store the temperature.
 * @return True on success, false otherwise.
 */
bool mlx90614_read_object_temperature(mlx90614_handle_t *handle, float *temperature);

/**
 * Send the MLX90614 to it's sleep mode. Attention! The device cannot be used afterwards before a power on reset was
 * performed.
 *
 * @param handle The handle to the MLX90614.
 * @return True on success, false otherwise.
 */
bool mlx90614_sleep(mlx90614_handle_t *handle);
