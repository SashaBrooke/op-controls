/**
 * @file uart_comm.c
 * @brief UART communication driver for non-blocking serial transmission.
 * 
 * Uses a ring buffer with atomic operations to avoid blocking the FreeRTOS
 * scheduler. Transmission happens entirely in the UART TX interrupt handler.
 */

#include "uart_comm.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

// Ring buffer for outgoing data
#define UART_TX_BUFFER_SIZE 2048

typedef struct {
    uint8_t buffer[UART_TX_BUFFER_SIZE];
    volatile uint16_t write_pos;  // Only modified by user task
    volatile uint16_t read_pos;   // Only modified by interrupt handler
} uart_ring_buffer_t;

static uart_ring_buffer_t tx_buffer;
static uart_inst_t *uart_inst = NULL;
static uart_hw_t *uart_hw_inst = NULL;
static uint8_t uart_irq_num = 0;

/**
 * @brief Get number of bytes in the ring buffer.
 * 
 * Safe to call from any context since it only reads volatile variables.
 */
static inline uint16_t ring_buffer_count(void) {
    uint16_t w = tx_buffer.write_pos;
    uint16_t r = tx_buffer.read_pos;
    if (w >= r) {
        return w - r;
    } else {
        return UART_TX_BUFFER_SIZE - (r - w);
    }
}

/**
 * @brief UART interrupt handler for TX ready events.
 * 
 * Only accessed from ISR context, no synchronization needed.
 */
static void uart_irq_handler(void) {
    // Check if TX FIFO has space and we have data to send
    while (uart_is_writable(uart_inst) && tx_buffer.read_pos != tx_buffer.write_pos) {
        uart_hw_inst->dr = tx_buffer.buffer[tx_buffer.read_pos];
        tx_buffer.read_pos = (tx_buffer.read_pos + 1) % UART_TX_BUFFER_SIZE;
    }
    
    // Disable TX interrupt if buffer is empty
    if (tx_buffer.read_pos == tx_buffer.write_pos) {
        hw_clear_bits(&uart_hw_inst->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

void uart_init_custom(uint8_t uart_id, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin) {
    // Initialize ring buffer
    tx_buffer.write_pos = 0;
    tx_buffer.read_pos = 0;
    
    // Get UART hardware instance
    uart_inst = uart_id == 0 ? uart0 : uart1;
    uart_hw_inst = uart_get_hw(uart_inst);
    
    // Initialize UART
    uart_init(uart_inst, baudrate);
    
    // Set GPIO pins
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // Setup TX interrupt
    uart_irq_num = uart_id == 0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq_num, uart_irq_handler);
    irq_set_enabled(uart_irq_num, true);
    
    // Enable UART TX interrupt
    hw_set_bits(&uart_hw_inst->imsc, UART_UARTIMSC_TXIM_BITS);
}

size_t uart_write_nonblocking(const uint8_t *data, size_t len) {
    if (!uart_hw_inst || data == NULL || len == 0) {
        return 0;
    }
    
    size_t bytes_written = 0;
    
    for (size_t i = 0; i < len; i++) {
        uint16_t next_write = (tx_buffer.write_pos + 1) % UART_TX_BUFFER_SIZE;
        
        // Stop if buffer is full
        if (next_write == tx_buffer.read_pos) {
            break;
        }
        
        tx_buffer.buffer[tx_buffer.write_pos] = data[i];
        
        // Use atomic write to ensure write_pos update is visible to interrupt handler
        tx_buffer.write_pos = next_write;
        bytes_written++;
    }
    
    // Enable TX interrupt if buffer has data
    if (tx_buffer.write_pos != tx_buffer.read_pos) {
        hw_set_bits(&uart_hw_inst->imsc, UART_UARTIMSC_TXIM_BITS);
    }
    
    return bytes_written;
}

size_t uart_tx_buffer_available(void) {
    if (!uart_hw_inst) {
        return 0;
    }
    
    uint16_t count = ring_buffer_count();
    return UART_TX_BUFFER_SIZE - count - 1;
}
