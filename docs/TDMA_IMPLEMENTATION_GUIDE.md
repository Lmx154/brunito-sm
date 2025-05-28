# TDMA Implementation Guide for 25Hz Telemetry
## Solving LoRa Half-Duplex Blocking with Time Division Multiple Access

### Problem Statement
At 25Hz telemetry rates (40ms cycles), continuous transmission occupies the half-duplex LoRa channel, preventing commands from finding receive windows. The SX127x transceivers cannot receive while transmitting, creating a communication deadlock.

### Solution Overview
Implement a Time Division Multiple Access (TDMA) approach that reserves guaranteed micro-RX windows in every 40ms cycle, ensuring commands can always reach the Flight Controller while maintaining high-rate telemetry.

---

## Phase 1: Radio Parameter Optimization (Baseline)

### Objective
Optimize LoRa settings to minimize Time-on-Air (TOA) and create larger RX windows.

### Current State Analysis
- **Current Settings**: SF7, 500kHz BW, CR5, 20dBm power, 6-byte preamble
- **Current TOA**: ~6ms for 30-byte payload at SF7/500kHz
- **Target**: Reduce TOA to ≤10ms (25% of 40ms cycle)

### Implementation Steps

#### Step 1.1: Calculate Optimal Radio Parameters
1. **File to modify**: `include/config/lora.h`
2. **Changes needed**:
   - Add TOA calculation macros
   - Define TDMA timing constants
   - Add adaptive rate thresholds

```cpp
// TDMA Timing Configuration
#define TDMA_CYCLE_MS           40      // 25Hz = 40ms cycle
#define TDMA_TX_WINDOW_MS       10      // Max TX time
#define TDMA_RX_WINDOW_MS       10      // Guaranteed RX window
#define TDMA_TURNAROUND_MS      2       // Radio switching time
#define TDMA_GUARD_MS           18      // Processing/guard time

// Optimized radio settings for fast TOA
#define LORA_SF_FAST            6       // SF6 for faster TOA
#define LORA_BW_FAST            500.0   // 500kHz bandwidth
#define LORA_PREAMBLE_FAST      6       // Minimum preamble
```

#### Step 1.2: Add TOA Calculation Functions
1. **File to modify**: `src/utils/LoraManager.cpp`
2. **Add method**: `calculateTimeOnAir(size_t payloadLen)`
3. **Purpose**: Validate packet timing fits within TDMA windows

#### Step 1.3: Implement Adaptive Rate Switching
1. **Enhancement**: Modify telemetry rate based on link quality
2. **Logic**: Drop to 10Hz if RSSI < -110dBm or if TOA exceeds budget
3. **Files affected**: `src/fc/FC.cpp`, `src/utils/LoraManager.cpp`

---

## Phase 2: FreeRTOS Task Architecture Redesign

### Objective
Restructure the FC's task system to implement strict TDMA scheduling with dedicated radio service task.

### Current State Analysis
- **Current Tasks**: sensorTask (P3), sdTask (P2), commandTask (P1)
- **Current Radio Handling**: Polled every 5ms in main loop
- **Target**: Dedicated high-priority radio service task with precise timing

### Implementation Steps

#### Step 2.1: Create TDMA Radio Service Task
1. **File to modify**: `src/navc/NAVC.cpp`
2. **New task**: `vTdmaRadioService` (Priority 4 - highest)
3. **Responsibilities**:
   - Own the radio exclusively
   - Enforce TDMA timing
   - Manage TX/RX windows
   - Handle all radio interrupts

```cpp
// Task signature to implement
void vTdmaRadioService(void *pvParameters);

// Timing variables
TickType_t tdmaCyclePeriod = pdMS_TO_TICKS(TDMA_CYCLE_MS);
TickType_t lastWakeTime;
```

#### Step 2.2: Implement Telemetry Producer Task
1. **File to modify**: `src/navc/NAVC.cpp`
2. **New task**: `vTelemetryProducer` (Priority 3)
3. **Function**: Generate telemetry at 25Hz, queue for transmission
4. **Queue**: Ring buffer for telemetry packets

#### Step 2.3: Enhance Command Processor Task
1. **File to modify**: `src/navc/NAVC.cpp`
2. **Modify existing**: `commandTask` becomes `vCommandProcessor`
3. **Priority**: Maintain Priority 1
4. **Enhancement**: Process commands from priority queue

#### Step 2.4: Add Inter-Task Communication
1. **New queues to create**:
   - `qTelemetryOut`: Ring buffer for outgoing telemetry
   - `qCommandsIn`: Priority queue for incoming commands
   - `qAckStatus`: Status feedback for reliability

2. **Semaphores**:
   - `xRadioMutex`: Protect radio hardware access
   - `xTimingSync`: Synchronize TDMA phases

---

## Phase 3: TDMA Timing Implementation

### Objective
Implement precise TDMA timing with guaranteed RX windows and strict cycle enforcement.

### Implementation Steps

#### Step 3.1: Core TDMA State Machine
1. **File to modify**: `src/utils/LoraManager.cpp`
2. **Add enum**: TDMA states (TX, TURNAROUND, RX, GUARD)
3. **Add method**: `executeTdmacycle()`

```cpp
typedef enum {
    TDMA_STATE_TX,          // Transmitting telemetry
    TDMA_STATE_TURNAROUND,  // Radio switching
    TDMA_STATE_RX,          // Receiving commands
    TDMA_STATE_GUARD        // Processing/idle
} tdma_state_t;
```

#### Step 3.2: Precision Timing Control
1. **Implementation**: Use `vTaskDelayUntil()` for jitter-free timing
2. **Cycle enforcement**: Start each 40ms cycle at exact intervals
3. **Overrun protection**: Handle TX overruns gracefully

#### Step 3.3: RX Window Management
1. **Guaranteed windows**: Always allocate 10ms for RX
2. **Command detection**: Fast interrupt-driven packet detection
3. **Priority handling**: Process commands immediately in RX window

#### Step 3.4: Adaptive TDMA
1. **TOA monitoring**: Track actual transmission times
2. **Window adjustment**: Adjust RX window based on measured TOA
3. **Fallback**: Drop to 10Hz if timing budget exceeded

---

## Phase 4: Ring Buffer and Queue Implementation

### Objective
Implement efficient data structures for high-throughput telemetry and low-latency commands.

### Implementation Steps

#### Step 4.1: Telemetry Ring Buffer
1. **File to create**: `include/utils/RingBuffer.h`
2. **Implementation**: Lock-free ring buffer for telemetry
3. **Features**:
   - Producer can overwrite old data
   - Consumer gets latest available packet
   - Thread-safe for single producer/consumer

```cpp
typedef struct {
    uint8_t data[RING_BUFFER_SIZE][LORA_MAX_PACKET_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile bool full;
} telemetry_ring_t;
```

#### Step 4.2: Command Priority Queue
1. **File to create**: `include/utils/PriorityQueue.h`
2. **Implementation**: Small priority queue for commands
3. **Features**:
   - EMERGENCY > CONTROL > STATUS commands
   - Sequence number tracking
   - Duplicate detection

#### Step 4.3: Reliability Tracking
1. **ACK embedding**: Embed ACK status in telemetry packets
2. **Sequence tracking**: Track command sequence numbers
3. **Retry logic**: Implement exponential backoff for GS

---

## Phase 5: Ground Station Synchronization

### Objective
Modify Ground Station to synchronize with FC's TDMA timing for optimal command injection.

### Implementation Steps

#### Step 5.1: Telemetry Listener Enhancement
1. **File to modify**: `src/gs/GS.cpp`
2. **Add timing detection**: Learn FC's cycle timing from telemetry
3. **Window prediction**: Predict next RX window opening

#### Step 5.2: Command Injection Timing
1. **Strategy**: Send commands immediately after receiving telemetry
2. **Timing**: Inject during FC's known RX window (8-18ms after telemetry)
3. **Collision avoidance**: Use CAD (Channel Activity Detection) if available

#### Step 5.3: Adaptive Command Scheduling
1. **Priority handling**: Send emergency commands immediately
2. **Batching**: Combine multiple low-priority commands
3. **Rate limiting**: Respect FC's processing capacity

---

## Phase 6: Packet Format and Protocol Updates

### Objective
Optimize packet formats for TDMA efficiency and add reliability features.

### Implementation Steps

#### Step 6.1: Compact Telemetry Format
1. **File to modify**: `include/utils/FrameCodec.h`
2. **Optimization**: Reduce telemetry packet size
3. **ACK embedding**: Include command ACK status in telemetry

```cpp
typedef struct {
    uint8_t type;
    uint8_t seq;           // Sequence number
    uint8_t ackSeq;        // Last command ACK'd
    uint8_t flags;         // Status flags
    uint8_t payload[26];   // Compressed telemetry
    uint16_t crc;
} __attribute__((packed)) tdma_packet_t;
```

#### Step 6.2: Command Format Enhancement
1. **Sequence numbers**: Add reliable delivery tracking
2. **Priority flags**: Embed urgency levels
3. **Batch commands**: Support multiple commands per packet

#### Step 6.3: Error Recovery Protocol
1. **Missing ACKs**: Exponential backoff retry
2. **Sequence gaps**: Request retransmission
3. **Link monitoring**: Continuous quality assessment

---

## Phase 7: Integration and Testing

### Objective
Integrate all components and validate TDMA performance under various conditions.

### Implementation Steps

#### Step 7.1: Unit Testing
1. **Ring buffer tests**: Verify thread safety and performance
2. **Timing tests**: Validate TDMA cycle precision
3. **Protocol tests**: Verify packet encoding/decoding

#### Step 7.2: Integration Testing
1. **FC-GS loop**: Test complete communication cycle
2. **Load testing**: Verify 25Hz sustained operation
3. **Latency testing**: Measure command response times

#### Step 7.3: Real-world Validation
1. **Range testing**: Verify performance at various distances
2. **Interference testing**: Test with 2.4GHz/WiFi interference
3. **Battery impact**: Measure power consumption changes

#### Step 7.4: Performance Monitoring
1. **Metrics to track**:
   - Telemetry throughput (packets/second)
   - Command latency (ms)
   - Packet loss rate (%)
   - TDMA timing jitter (μs)

---

## Phase 8: Optimization and Tuning

### Objective
Fine-tune TDMA parameters for optimal performance across different scenarios.

### Implementation Steps

#### Step 8.1: Adaptive Parameters
1. **Dynamic window sizing**: Adjust RX window based on traffic
2. **Rate adaptation**: Auto-switch between 25Hz/10Hz/5Hz
3. **Power optimization**: Adjust TX power based on link quality

#### Step 8.2: Advanced Features
1. **Command piggybacking**: Embed commands in ACK packets
2. **Burst telemetry**: Send multiple packets in TX window
3. **Predictive scheduling**: Anticipate command needs

#### Step 8.3: Diagnostics and Monitoring
1. **TDMA health metrics**: Monitor cycle timing accuracy
2. **Link quality tracking**: Continuous RSSI/SNR monitoring  
3. **Performance dashboards**: Real-time system status

---

## Implementation Timeline and Dependencies

### Phase Dependencies
1. **Phase 1** → **Phase 2**: Radio optimization before task redesign
2. **Phase 2** → **Phase 3**: Task architecture before TDMA implementation
3. **Phase 3** → **Phase 4**: TDMA timing before data structures
4. **Phase 4** → **Phase 5**: FC implementation before GS synchronization
5. **Phase 5** → **Phase 6**: Basic protocol before optimizations
6. **Phases 1-6** → **Phase 7**: All components before testing
7. **Phase 7** → **Phase 8**: Testing results drive optimizations

### Estimated Timeline
- **Phase 1**: 2-3 days (radio optimization)
- **Phase 2**: 3-4 days (FreeRTOS restructure)
- **Phase 3**: 4-5 days (TDMA core implementation)
- **Phase 4**: 2-3 days (data structures)
- **Phase 5**: 3-4 days (GS synchronization)
- **Phase 6**: 2-3 days (protocol updates)
- **Phase 7**: 5-7 days (comprehensive testing)
- **Phase 8**: 3-5 days (optimization)

**Total**: 24-34 days

---

## Key Files to Modify

### Core Implementation Files
1. `include/config/lora.h` - TDMA timing constants
2. `src/utils/LoraManager.cpp` - TDMA state machine
3. `src/navc/NAVC.cpp` - FreeRTOS task restructure
4. `src/fc/FC.cpp` - FC integration
5. `src/gs/GS.cpp` - GS synchronization

### New Files to Create
1. `include/utils/RingBuffer.h` - Telemetry ring buffer
2. `include/utils/PriorityQueue.h` - Command priority queue
3. `include/utils/TdmaScheduler.h` - TDMA timing control
4. `src/utils/TdmaScheduler.cpp` - TDMA implementation

### Testing Files
1. `test/test_tdma_timing.cpp` - TDMA precision tests
2. `test/test_ring_buffer.cpp` - Data structure tests
3. `test/test_protocol.cpp` - Protocol validation

---

## Success Criteria

### Performance Targets
- **Telemetry Rate**: Sustained 25Hz (40ms cycles)
- **Command Latency**: <100ms average, <200ms maximum
- **Packet Loss**: <1% under normal conditions
- **Timing Jitter**: <5ms TDMA cycle variation

### Reliability Requirements
- **Command ACK**: 99.9% delivery confirmation
- **Link Recovery**: <5 seconds after temporary outage
- **Graceful Degradation**: Auto-fallback to 10Hz if needed

### Compatibility
- **Existing Hardware**: No hardware changes required
- **Protocol Backward Compatibility**: Support legacy 10Hz mode
- **Power Consumption**: <10% increase over current implementation

---

## Risk Mitigation

### Technical Risks
1. **Timing Precision**: Use hardware timers if FreeRTOS insufficient
2. **Radio Switching Speed**: Buffer extra time for TX/RX transitions
3. **Interrupt Latency**: Minimize ISR processing time

### Operational Risks
1. **Testing Complexity**: Implement comprehensive simulation
2. **Regression Issues**: Maintain fallback to current implementation
3. **Field Validation**: Extensive real-world testing before deployment

### Contingency Plans
1. **Performance Issues**: Auto-fallback to 10Hz operation
2. **Reliability Problems**: Revert to current queue-based system
3. **Integration Conflicts**: Maintain separate TDMA branch until stable

This implementation guide provides a comprehensive roadmap for implementing TDMA-based 25Hz telemetry while maintaining reliable command communication in the Brunito LoRa system.
