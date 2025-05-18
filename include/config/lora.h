/**
 * lora.h - LoRa configuration for Brunito Project
 * 
 * This file contains the LoRa settings and constants used by
 * both FC and GS modules.
 */

#ifndef LORA_H
#define LORA_H

// LoRa configuration
#define LORA_FREQUENCY        915.0
#define LORA_BANDWIDTH        250.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODING_RATE      5
#define LORA_SYNC_WORD        0xAB
#define LORA_TX_POWER         20
#define LORA_PREAMBLE_LEN     8

// Node addresses
#define LORA_FC_ADDR          0xA2
#define LORA_GS_ADDR          0xA1

// Pin definitions for RFM95W (same for both FC and GS)
#define LORA_NSS              PA4
#define LORA_DIO0             PA8
#define LORA_NRST             PA9
#define LORA_DIO1             PA10

// Timeouts and retries
#define LORA_ACK_TIMEOUT_MS   1200  // Increased to allow more time for ACKs to arrive
#define LORA_MAX_RETRIES      5    // Increased for better reliability 
#define LORA_RETRY_BASE_MS    500  // Increased base backoff time for retries
#define LORA_MAX_PACKET_SIZE  128

// Message types
#define LORA_TYPE_CMD         0x01
#define LORA_TYPE_ACK         0x02
#define LORA_TYPE_TELEM       0x03
#define LORA_TYPE_SETTINGS    0x04
#define LORA_TYPE_STATUS      0x05   // fire-and-forget
#define LORA_TYPE_PING        0x06   // ping request
#define LORA_TYPE_PONG        0x07   // ping response

// Settings for handshake
typedef struct {
    uint8_t spreadingFactor;
    float bandwidth;
    uint8_t codingRate;
    uint8_t syncWord;
    uint8_t txPower;
} LoraSettings;

#endif // LORA_H
