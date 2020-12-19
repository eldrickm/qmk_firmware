/* Matrix scanning for Phase using a PCA9555 over I2C.
 *
 * With 100kHz I2C clock rate and 8 I2C commands per scan, this matrix scanning routine should run
 * at about 260Hz.
 */

#include QMK_KEYBOARD_H
#include "i2c_master.h"

#define I2C_TIMEOUT 1000

#define I2C_ADDR 0x40  // 8-bit address

#define REG_INPUT  0  // input register (2 bytes)
#define REG_OUTPUT 2  // output register (2 bytes); powers up to high
#define REG_CONFIG 6  // direction register; 0 is output, 1 is input

static const pin_t row_pins_mcu[MATRIX_ROWS_PER_SIDE] = MATRIX_ROW_PINS_MCU;
static const pin_t col_pins_mcu[MATRIX_COLS_PER_SIDE] = MATRIX_COL_PINS_MCU;
static const uint8_t row_pins_mcp[MATRIX_ROWS_PER_SIDE] = MATRIX_ROW_PINS_MCP;
static const uint8_t col_pins_mcp[MATRIX_COLS_PER_SIDE] = MATRIX_COL_PINS_MCP;

/* Write two bytes to a register */
static void i2c_write_uint16(uint8_t reg, uint16_t data) {
    uint8_t writeptr[2] = { data & 0xFF, data >> 8 };
    i2c_writeReg(I2C_ADDR, reg, writeptr, 2, I2C_TIMEOUT);
}

/* Select a row on both the right and left sides by making it low. */
static void select_row(uint8_t row) {
    uint16_t index = 1 << row_pins_mcp[row];  // bit mask for config register
    uint16_t data = 0xFFFF ^ index;  // set as output in config reg
    i2c_write_uint16(REG_CONFIG, data);

    setPinOutput(row_pins_mcu[row]);
    writePinLow(row_pins_mcu[row]);
}

/* Unselect a row on the MCU. Right hand row will be unselected when the next row is selected. */
static void unselect_row(uint16_t row) {
    setPinInputHigh(row_pins_mcu[row]);
}

/* Unselect all rows and columns on the MCU and PCA9555 */
static void unselect_all(void) {
    i2c_write_uint16(REG_CONFIG, 0xFFFF);  // set all pins as inputs
    i2c_write_uint16(REG_OUTPUT, 0x0000);  // set to logic low when the pins are written as outputs

    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinInputHigh(row_pins_mcu[x]);
    }
    for (uint8_t x = 0; x < MATRIX_COLS_PER_SIDE; x++) {
        setPinInputHigh(col_pins_mcu[x]);
    }
}

/* Read the INPUT register on the PCA9555 into a uint16_t. */
static uint16_t get_mcp_pin_states(void) {
    uint8_t readptr[2];
    i2c_readReg(I2C_ADDR, REG_INPUT, readptr, 2, I2C_TIMEOUT);
    return readptr[0] | (readptr[1] << 8);
}

/* Read all the columns with one row held low. */
static bool read_cols_on_row(matrix_row_t current_matrix[], uint8_t current_row) {
    matrix_row_t last_row_value = current_matrix[current_row];
    current_matrix[current_row] = 0;

    select_row(current_row);

    // Get pin states on right side
    uint16_t mcp_pin_states = get_mcp_pin_states();

    for (uint8_t col_index = 0; col_index < MATRIX_COLS_PER_SIDE; col_index++) {
        uint8_t pin_state = readPin(col_pins_mcu[col_index]);
        current_matrix[current_row] |= pin_state ? 0 : (MATRIX_ROW_SHIFTER << col_index);

        uint16_t mcp_pin_state = mcp_pin_states & (1 << col_pins_mcp[col_index]);
        current_matrix[current_row] |= mcp_pin_state ? 0 : (MATRIX_ROW_SHIFTER
                                                            << (col_index + MATRIX_COLS_PER_SIDE));
    }

    unselect_row(current_row);

    return (last_row_value != current_matrix[current_row]);
}

/* Init i2c and set all rows and columns high with pullups to VDD. */
void matrix_init_custom(void) {
    i2c_init();
    unselect_all();
}

/* Main matrix scanning function. Modifies current_matrix to the reflect which keys are pressed
 * and returns whether or not the the matrix state has changed. */
bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    bool changed = false;

    // Set row, read cols
    for (uint8_t current_row = 0; current_row < MATRIX_ROWS; current_row++) {
        changed |= read_cols_on_row(current_matrix, current_row);
    }

    return changed;
}
