#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "i2c_target.h"

// The state of an I2C transaction
static enum { IDLE, NEW_TRANSACTION, RECEIVED_ADDRESS, RECEIVED_DATA, SENT_DATA };
static uint8_t twi_state = IDLE;
static bool first_read   = true;

// I2C register access
static uint8_t reg_index              = 0;
static read_register_fn reg_read_fn   = NULL;
static write_register_fn reg_write_fn = NULL;

static void i2c_target_handle_address_match()
{
    // Reads are only allowed after we've received a register address
    if ((TWI0.SSTATUS & TWI_DIR_bm) && twi_state != RECEIVED_ADDRESS) {
        // Send NACK and complete the transaction
        TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
        return;
    }

    // Update internal state
    twi_state = NEW_TRANSACTION;

    // Send ACK and wait for data
    TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
}

static void i2c_target_handle_stop()
{
    // Reset internal state
    twi_state  = IDLE;
    first_read = true;

    // Clear the interrupt flag
    TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
}

static void i2c_target_handle_data()
{
    if (TWI0.SSTATUS & TWI_DIR_bm) {
        // Handle reads from a master
        if ((TWI0.SSTATUS & TWI_RXACK_bm) && !first_read) {
            // Send NACK and complete the transaction
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
        } else {
            // Fetch the data from the requested register address
            reg_read_fn(reg_index, &TWI0.SDATA);

            // Update internal state, send ACK and wait for more data
            twi_state   = SENT_DATA;
            first_read  = false;
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;

            // Support auto-incrementing register reads
            reg_index++;
        }
    } else {
        // Handle writes from a master
        if (twi_state == NEW_TRANSACTION) {
            // The first write of a transaction is the register index
            reg_index = TWI0.SDATA;

            // Update internal state, send ACK and wait for more data
            twi_state   = RECEIVED_ADDRESS;
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
        } else {
            // Subsequent writes are the register data
            // Write the data to the requested register address
            reg_write_fn(reg_index, TWI0.SDATA);

            // Update internal state, send ACK and wait for more data
            twi_state   = RECEIVED_DATA;
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;

            // Support auto-incrementing register writes
            reg_index++;
        }
    }
}

// I2C target mode interrupt handler
ISR(TWI0_TWIS_vect)
{
    if (TWI0.SSTATUS & (TWI_COLL_bm | TWI_BUSERR_bm)) {
        // Handle collisions and bus errors
        TWI0.SSTATUS |= (TWI_COLL_bm | TWI_BUSERR_bm);
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    } else if (TWI0.SSTATUS & TWI_APIF_bm) {
        // Handle address match and stop condition interrupts
        if (TWI0.SSTATUS & TWI_AP_bm) {
            i2c_target_handle_address_match();
        } else {
            i2c_target_handle_stop();
        }
    } else if (TWI0.SSTATUS & TWI_DIF_bm) {
        // Handle data interrupts
        i2c_target_handle_data();
    }
}

void i2c_target_init(uint8_t addr, read_register_fn read_fn, write_register_fn write_fn)
{
    // Store the register access functions
    reg_read_fn  = read_fn;
    reg_write_fn = write_fn;

    // Set the I2C target address
    TWI0.SADDR = (addr << 1);

    // Enable I2C target mode, stop interrupt, address match interrupt, and data interrupt
    TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;
}