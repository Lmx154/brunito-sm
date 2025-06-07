/**
 * FC_FreeRTOS.h - Flight Controller FreeRTOS Organization
 * 
 * This header file documents the FreeRTOS task structure and
 * inter-task communication mechanisms used in the Flight Controller.
 */

#ifndef FC_FREERTOS_H
#define FC_FREERTOS_H

#include <STM32FreeRTOS.h>

/**
 * FreeRTOS Task Architecture:
 * 
 * 1. Command Task (Priority: 3, Stack: 2048)
 *    - Processes commands from USB-CDC serial
 *    - Processes commands from command queue (LoRa commands)
 *    - Sends acknowledgments
 *    - Rate: 100Hz
 * 
 * 2. Status Task (Priority: 1, Stack: 1024)
 *    - Sends periodic status updates
 *    - Updates state manager
 *    - Rate: 0.1Hz (every 10 seconds)
 * 
 * 3. UART Task (Priority: 4, Stack: 2048)
 *    - Reads binary packets from NAVC
 *    - Controls telemetry streaming
 *    - Sends packets to telemetry queue
 *    - Rate: 1kHz
 * 
 * 4. LoRa Task (Priority: 3, Stack: 2048)
 *    - Manages LoRa transmit/receive
 *    - Processes packet queue
 *    - Sends periodic reports
 *    - Rate: 200Hz
 * 
 * 5. Telemetry Task (Priority: 2, Stack: 1536)
 *    - Formats telemetry data
 *    - Implements adaptive rate control
 *    - Manages congestion control
 *    - Rate: Variable (1-20Hz)
 * 
 * 6. Heartbeat Task (Priority: 1, Stack: 512)
 *    - Updates LED pattern based on state
 *    - Rate: 100Hz
 * 
 * 7. Background Task (Priority: 2, Stack: 1024)
 *    - State-specific background processes
 *    - Manual ARM button monitoring (IDLE)
 *    - Buzzer beeping (ARMED)
 *    - Rate: 100Hz
 */

/**
 * Inter-Task Communication:
 * 
 * Queues:
 * - commandQueue: Commands from LoRa → Command Task
 * - telemetryQueue: Sensor packets from UART → Telemetry Task
 * - loraRxQueue: (Reserved for future use)
 * 
 * Mutexes:
 * - stateMutex: Protects StateManager access
 * - uartMutex: Protects UART hardware and UartManager
 * - loraMutex: Protects LoRa hardware and LoraManager
 */

/**
 * Task Synchronization Rules:
 * 
 * 1. State Access:
 *    - Always acquire stateMutex before accessing StateManager
 *    - Hold mutex for minimum time necessary
 *    - Maximum wait time: 100ms for critical operations
 * 
 * 2. UART Access:
 *    - UART Task has primary ownership
 *    - Other tasks must acquire uartMutex for:
 *      - Sending commands to NAVC
 *      - Reading packet statistics
 * 
 * 3. LoRa Access:
 *    - LoRa Task has primary ownership
 *    - Other tasks must acquire loraMutex for:
 *      - Sending packets
 *      - Reading statistics
 *      - Checking connection status
 * 
 * 4. Queue Usage:
 *    - Use xQueueSend with timeout 0 (non-blocking)
 *    - Drop packets if queue is full
 *    - Check return value for queue operations
 */

/**
 * Memory Management:
 * 
 * - All tasks use static stack allocation
 * - No dynamic memory allocation after initialization
 * - Stack sizes chosen based on worst-case usage
 * - Monitor stack high water marks during development
 */

/**
 * Error Handling:
 * 
 * - Each task implements local error recovery
 * - Critical errors logged via Serial
 * - System continues operation even if individual tasks fail
 * - Watchdog timer monitors overall system health
 */

#endif // FC_FREERTOS_H