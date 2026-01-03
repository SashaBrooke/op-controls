/**
 * @file uart_comm.h
 * @brief UART communication driver for non-blocking serial transmission.
 * 
 * Provides non-blocking UART write operations using an internal ring buffer
 * to avoid blocking the FreeRTOS scheduler during UART transmissions.
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize UART communication with specified baud rate.
 * 
 * @param uart_id The UART peripheral ID (uart0 or uart1)
 * @param baudrate The baud rate for communication (e.g., 115200)
 * @param tx_pin The GPIO pin for UART TX
 * @param rx_pin The GPIO pin for UART RX
 */
void uart_init_custom(uint8_t uart_id, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin);

/**
 * @brief Non-blocking UART write that uses internal ring buffer.
 * 
 * Data is written to the ring buffer and transmitted via interrupt handler.
 * This function never blocks the calling task.
 * 
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write
 * @return Number of bytes successfully written to buffer, may be less than len if buffer is full
 */
size_t uart_write_nonblocking(const uint8_t *data, size_t len);

/**
 * @brief Get the number of bytes available in the TX buffer.
 * 
 * @return Number of free bytes available in the TX buffer
 */
size_t uart_tx_buffer_available(void);

#endif  // UART_COMM_H
