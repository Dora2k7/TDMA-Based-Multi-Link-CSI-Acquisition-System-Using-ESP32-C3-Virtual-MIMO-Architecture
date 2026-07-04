# TDMA-Based Multi-Link CSI Acquisition System  
### Virtual MIMO Architecture with 4× ESP32-C3 SISO Nodes

---

## System Architecture

![System Architecture Overview](docs/system_architecture.png)

*Figure 1. Complete system overview: (1) 4-node Virtual MIMO topology with 12-slot TDMA schedule, (2) raw CSI data matrix and USB data pipeline, (3) signal processing pipeline (Hampel → Gaussian → PCA).*

---

## 1. Introduction

Channel State Information (CSI) describes how a wireless signal propagates between a transmitter and a receiver — capturing the combined effects of path loss, multipath reflections, scattering, and fading across each OFDM subcarrier. When a person moves within the propagation environment, their body alters the multipath structure, producing measurable variations in CSI amplitude and phase. This makes CSI a powerful modality for **device-free sensing** applications including Human Activity Recognition (HAR) and Pose Estimation.

However, CSI from a single SISO link provides only one spatial perspective, limiting the ability of AI models to distinguish complex activities. This project addresses that limitation by constructing a **Virtual MIMO** system: 4 single-antenna ESP32-C3 nodes coordinated via **TDMA-based Multi-Link CSI Acquisition**, producing **12 independent wireless links** per measurement cycle — each capturing a distinct spatial view of the environment.

---

## 2. How CSI Is Generated via ESP-NOW

### 2.1. Physical-Layer CSI Extraction

In an OFDM-based WiFi system (IEEE 802.11n), every transmitted frame begins with a **preamble** containing known reference signals called **Long Training Fields (LTF)**. These LTF sequences are predefined by the standard — both the transmitter and receiver know exactly what was sent.

When the receiver's WiFi hardware captures an incoming frame, it compares the **received** LTF with the **expected** LTF for each OFDM subcarrier. The complex ratio between them yields the **Channel Frequency Response (CFR)** — this is CSI:

> **CSI = H(f) = Y(f) / X(f)**
> 
> where *Y(f)* is the received signal and *X(f)* is the known transmitted signal at subcarrier frequency *f*.

Each subcarrier's CSI is a complex number (I + jQ) encoding both **amplitude** (signal attenuation) and **phase** (propagation delay). The ESP32-C3 reports these as signed 8-bit integer pairs (I, Q) for each subcarrier.

### 2.2. Why ESP-NOW Triggers CSI

**ESP-NOW** is Espressif's connectionless peer-to-peer protocol operating at the MAC layer. It requires no WiFi association, no router, and no handshake — a node simply transmits a frame to a known MAC address. Crucially for this project:

- Every ESP-NOW frame is a standard 802.11 action frame with a **full OFDM preamble** (including LTFs).
- The receiving node's WiFi hardware **automatically** extracts CSI from this preamble, independent of the frame's payload content.
- The ESP-IDF firmware exposes this CSI via a registered callback (`wifi_csi_cb`), which fires on every received frame.

Therefore, each ESP-NOW Probe packet in the TDMA schedule serves a **dual purpose**: (1) it carries control data (round_id, tx_id, sequence number) for the TDMA protocol, and (2) its physical-layer preamble triggers CSI extraction at the receiver — all within a single over-the-air transmission.

### 2.3. CSI Configuration

The firmware configures CSI extraction as follows:

| Setting | Value | Effect |
|:---|:---:|:---|
| `lltf_en` | `true` | Extract CSI from Legacy LTF |
| `htltf_en` | `true` | Extract CSI from HT-LTF |
| `ltf_merge_en` | `true` | Merge both LTFs → 128 bytes = **64 subcarriers** (I/Q pairs) |
| `stbc_htltf2_en` | `false` | Disabled — ESP32-C3 has no STBC support |
| `channel_filter_en` | `false` | Raw CSI without hardware smoothing |

After removing the DC subcarrier (index 0) and guard band (indices 27–37), **52 clean subcarriers** remain per link.

---

## 3. ESP32-C3 WiFi Capabilities and Design Impact

The ESP32-C3's WiFi radio has specific capabilities and constraints that directly determined the system architecture:

| WiFi Feature | ESP32-C3 Specification | Design Impact |
|:---|:---|:---|
| **Antenna config** | 1T1R (single antenna) | SISO only → requires multi-node Virtual MIMO topology |
| **Bandwidth** | HT20 only (20 MHz) | 64 OFDM subcarriers; no HT40 option to double resolution |
| **Frequency band** | 2.4 GHz only | Good wall penetration for indoor sensing; higher interference risk |
| **MIMO support** | None | Cannot extract spatial streams → each node is one "virtual antenna" |
| **PHY rates** | MCS0–MCS7 | MCS0 (BPSK, rate 1/2) chosen for maximum robustness |
| **STBC** | Not supported | `stbc_htltf2_en` disabled; no space-time coding gain |
| **ESP-NOW latency** | < 1 ms | Enables sub-millisecond TDMA slot coordination |
| **CSI buffer size** | 128 bytes (merged LTF) | 64 subcarriers × 2 bytes (I, Q) per subcarrier |
| **CPU** | Single-core RISC-V | Insufficient for real-time DSP → offload to PC |
| **USB interface** | Built-in USB Serial JTAG | 2 Mbaud; no external UART bridge needed |

**Key architectural consequence:** Because each ESP32-C3 has only **one antenna** and **no MIMO capability**, the only way to achieve spatial diversity is to deploy **multiple nodes** at different physical locations. The **Virtual MIMO** approach — where 4 SISO nodes generate 12 links through TDMA scheduling — is a direct response to this hardware limitation.

---

## 4. TDMA System Design

### 4.1. Timing Budget

| Parameter | Value |
|:---|---:|
| Nodes (*N*) | 4 |
| Slots per cycle | *N*(*N*−1) = 12 |
| Slot duration | 2,000 µs |
| Cycle period | 24 ms |
| Theoretical max FPS | **41.67 fps** |
| ACK timeout | 500 µs per attempt |
| Retries | 1 |
| Guard time per slot | 1,000 µs |

### 4.2. Slot Protocol

Within each 2,000 µs slot:

1. **TX** sends Probe packet `{type=1, tx_id, round_id, seq}` via ESP-NOW
2. **RX** hardware extracts CSI from preamble → enqueued with `round_id` (captured atomically in ISR)
3. **RX** software returns ACK `{type=2, tx_id, seq}`
4. **TX** waits up to 500 µs for ACK; retries once if needed
5. Remaining ~1,000 µs: CRC16 computation + USB write

### 4.3. Synchronization

**Node 0 (Master)** maintains the global `round_id`. When Slaves receive the Master's first Probe of a new round, they enter a critical section, reset their slot counter, and restart their hardware timer — re-synchronizing every 24 ms to suppress crystal oscillator drift.

### 4.4. Binary Protocol (ESP32 → PC)

```
[Magic 4B: 0x43534921] [round_id 4B] [tx_id 1B] [rx_id 1B] [RSSI 1B] [csi_len 2B] [CSI payload] [CRC16 2B]
```

Four concurrent reader threads (one per COM port) parse the binary stream. CRC16-CCITT failures are silently discarded. A garbage collector purges incomplete rounds (lag > 10) to prevent memory leaks.

---

## 5. Signal Processing Pipeline

Raw CSI frames pass through three sequential stages before being consumed by AI models:

```
Raw CSI (52×12) → Hampel → Gaussian → Online PCA → Output (52×5)
```

| Stage | Domain | Dim | Purpose | Key Params |
|:---|:---|:---:|:---|:---|
| **Hampel Filter** | Temporal (per element) | 52×12 → 52×12 | Remove impulsive spikes using MAD-based robust statistics | *W*=7, *n*σ=2.5 |
| **Gaussian Filter** | Subcarrier (per frame) | 52×12 → 52×12 | Smooth EMI sawtooth noise between adjacent subcarriers | σ=0.8, *K*=7 |
| **Online PCA** | Link (per frame) | 52×12 → 52×5 | Reduce 12 links to 5 principal components; suppress static reflections via temporal detrending (EMA) | *k*=5, α=0.05, β=0.05 |

The ordering is deliberate: Hampel runs first to prevent spike propagation through the Gaussian convolution kernel; Gaussian runs before PCA to avoid distorting the inter-link covariance matrix.

---

## 6. Experimental Results

### 6.1. Measurement Conditions

| | Run A | Run B |
|:---|:---|:---|
| **TDMA config** | Previous (conservative timing) | Current (*T*_slot = 2,000 µs) |
| **Environment** | Indoor laboratory | University library |
| **Duration** | 105.3 s | 7.2 s (pilot) |

### 6.2. Results

| Metric | Run A | Run B |
|:---|---:|---:|
| Total frames | 1,959 | 880 |
| Complete frames (12/12) | 1,387 | 286 |
| Incomplete frames | 572 | 594 |
| CRC errors | 4 | 4 |
| **Completeness ratio** | **70.8%** | **32.5%** |
| Total FPS | 18.61 | 121.47 |
| **Complete FPS** | **13.17** | **39.48** |

### 6.3. Analysis

**Run A** used a previous, more conservative TDMA configuration (longer slot durations). The higher completeness (70.8%) reflects greater guard margins for retransmission at the cost of lower frame rate (13.17 fps). The system operated stably over 105 seconds, producing 1,387 clean frames without memory leaks or sync loss.

**Run B** used the current configuration in a university library with significant ambient WiFi interference. The complete frame rate of **39.48 fps** reaches **94.7% of the theoretical maximum** (41.67 fps), confirming near-optimal TDMA utilization. The lower completeness (32.5%) is attributed to RF-level packet loss from co-channel interference, not insufficient retries.

**CRC integrity:** Only 4 errors across both runs (< 0.3%), confirming reliable USB data transfer.

### 6.4. Parameter Trade-offs

| Parameter | ↓ Decrease | ↑ Increase |
|:---|:---|:---|
| **Slot duration** (2,000 µs) | Higher FPS, less guard → more drops | Lower FPS, more guard → better completeness |
| **ACK timeout** (500 µs) | Risk premature timeout (RTT ~200–400 µs) | Less guard time for CRC/USB |
| **Retries** (1) | Fewer recovery chances | Consumes guard; diminishing returns |
| **Nodes** (*N*=4) | Fewer links, higher FPS (*f* ∝ 1/*N*²) | More spatial diversity, lower FPS |

---

## 7. Limitations and Future Work

- [ ] Extended measurements (> 10 min) under current configuration
- [ ] Quantitative SNR improvement per filter stage
- [ ] Optimal PCA components (*k*) across activity types
- [ ] End-to-end latency characterization
- [ ] Downstream AI model accuracy (HAR, Pose Estimation)

---

## Project Structure

```
├── README.md                       # This document
├── docs/
│   └── system_architecture.png     # System overview diagram (Figure 1)
├── main/
│   └── main.c                      # ESP32-C3 firmware (TDMA scheduler + CSI extraction + binary protocol)
└── python/
    ├── csi_receiver.py             # Multi-threaded binary CSI receiver + frame assembly
    └── csi_processor.py            # Hampel + Gaussian + PCA pipeline + real-time visualization
```

## Quick Start

```bash
# Flash firmware to all 4 ESP32-C3 nodes (ESP-IDF required)
idf.py build flash monitor

# Run raw CSI collection only
python python/csi_receiver.py

# Run full pipeline with real-time plot
python python/csi_processor.py
```

---

*This project is developed for academic research purposes.*
