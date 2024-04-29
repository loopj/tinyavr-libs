/*
 * Basic I2C target device library for byte registers.
 * 
 * - Provides a simple interface for reading and writing byte registers
 * - Supports 7-bit I2C addresses
 * - Supports auto-incrementing register reads and writes
 * - No support for I2C general call addresses
 * - No support for matching on multiple addresses
*/

#pragma once

#include <stdint.h>

// Callback for reading the value of a single byte register from the specified address
typedef int (*read_register_fn)(uint8_t reg_addr, uint8_t *value);

// Callback for writing a value to a single byte register at the specified address
typedef int (*write_register_fn)(uint8_t reg_addr, uint8_t value);

// Initialize as an I2C target device, with the provided access functions
void i2c_target_init(uint8_t dev_addr, read_register_fn read_fn, write_register_fn write_fn);