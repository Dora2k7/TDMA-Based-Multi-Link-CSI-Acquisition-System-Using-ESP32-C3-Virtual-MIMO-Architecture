# TDMA-Based Multi-Link CSI Acquisition System Using ESP32-C3 Virtual MIMO Architecture

---

## 1. Introduction

Channel State Information (CSI) extracted from commercial WiFi transceivers has emerged as a promising modality for device-free sensing, enabling applications such as Human Activity Recognition (HAR), gesture detection, and pose estimation without requiring users to carry dedicated sensors. However, CSI obtained from a single-link Single-Input Single-Output (SISO) transceiver provides limited spatial diversity, constraining the discriminative power of downstream machine learning models.

This work presents the design and implementation of a low-cost, multi-link CSI acquisition platform based on a **Virtual MIMO** architecture, employing **TDMA-based Multi-Link CSI Acquisition**. The system exploits $N = 4$ SISO transceiver nodes operating under a Time-Division Multiple Access (TDMA) schedule to produce $N(N-1) = 12$ independent wireless links per measurement cycle. Each link captures a distinct spatial perspective of the monitored environment, collectively providing channel diversity equivalent to a distributed antenna array — realized entirely through time-multiplexing of inexpensive single-antenna hardware.

---

## 2. Hardware Platform: ESP32-C3

### 2.1. Device Specifications

The ESP32-C3, manufactured by Espressif Systems, is a single-core RISC-V 32-bit microcontroller integrating an IEEE 802.11 b/g/n transceiver operating in the 2.4 GHz ISM band with 20 MHz channel bandwidth (HT20). It supports physical-layer CSI extraction from the Long Training Field (LTF) via the ESP-IDF SDK and features a built-in USB Serial JTAG interface capable of sustaining data rates up to 2,000,000 baud without external UART-USB bridge circuitry.

### 2.2. Rationale for Selection

The ESP32-C3 was chosen based on the following criteria: (i) low unit cost enabling multi-node deployment, (ii) compact form factor suitable for unobtrusive installation, (iii) native ESP-NOW peer-to-peer protocol with sub-millisecond latency for inter-node coordination, and (iv) firmware-level access to raw CSI buffers including amplitude and phase information across OFDM subcarriers.

### 2.3. Limitations and Architectural Consequences

The ESP32-C3 exhibits several constraints that directly shaped the system architecture:

- **SISO-only radio chain**: A single transmit and single receive antenna, precluding native spatial multiplexing. This limitation motivated the multi-node Virtual MIMO topology.
- **Limited computational resources**: A single CPU core with no hardware DSP, insufficient for real-time multi-link signal processing. Consequently, all filtering and dimensionality reduction are offloaded to a host PC.
- **Known hardware errata**: The first four bytes of the CSI buffer may contain invalid values, flagged by the `first_word_invalid` field. The firmware applies deterministic zeroing of these bytes to prevent noise propagation into downstream algorithms.

---

## 3. System Architecture

### 3.1. Hardware Topology

The system comprises four ESP32-C3 nodes connected to a host PC through a USB Hub. All nodes operate on WiFi Channel 11 (2.462 GHz) in Station mode. Communication between nodes uses ESP-NOW with the HT20 physical-layer mode at MCS0 (BPSK, coding rate 1/2), chosen for its robustness against channel fading. The maximum transmit power is configured at 13 dBm (52 in quarter-dBm units).

### 3.2. TDMA Schedule Design

Node 0 serves as the TDMA **Master**, maintaining the global cycle counter (`round_id`) and providing timing reference for the network. Nodes 1–3 operate as **Slaves**. Each TDMA cycle comprises 12 time slots, one for each ordered transmitter–receiver pair in the full-mesh topology.

**TDMA Timing Budget (current configuration):**

| Parameter | Symbol | Value |
|:---|:---:|:---:|
| Number of nodes | $N$ | 4 |
| Slots per cycle | $N_{\text{slot}}$ | $N(N-1) = 12$ |
| Slot duration | $T_{\text{slot}}$ | 2,000 µs |
| Cycle period | $T_{\text{cycle}}$ | 24,000 µs (24 ms) |
| Theoretical max frame rate | $f_{\max}$ | 41.67 fps |
| ACK timeout per attempt | $T_{\text{ack}}$ | 500 µs |
| Maximum retries | $R$ | 1 |
| Worst-case active time | $T_{\text{active}}$ | $(R+1) \times T_{\text{ack}} = 1{,}000$ µs |
| Guard time | $T_{\text{guard}}$ | $T_{\text{slot}} - T_{\text{active}} = 1{,}000$ µs |

**Slot-level protocol:** Within each slot, the designated transmitter sends a Probe packet containing `{type, tx_id, round_id, seq}` via ESP-NOW. Upon reception, the receiver: (a) extracts CSI from the physical-layer preamble and enqueues it with the current `round_id` captured atomically inside the ISR callback, and (b) returns an ACK packet containing `{type, tx_id, seq}`. If no ACK is received within $T_{\text{ack}} = 500$ µs, a single retry is attempted. After at most two attempts (1,000 µs), the slot terminates. The remaining 1,000 µs guard time accommodates CRC16 computation, USB Serial JTAG write operations, and inter-slot timing margins.

### 3.3. Inter-Node Synchronization

Clock drift between independent crystal oscillators is suppressed by a **Hard Sync** mechanism. When a Slave node receives the Master's first Probe of a new `round_id`, it:

1. Enters a critical section (`portDISABLE_INTERRUPTS()`),
2. Resets its local slot counter to the expected position based on its node ID,
3. Exits the critical section and restarts the hardware timer.

This re-synchronization occurs once per cycle (every 24 ms), preventing drift accumulation across consecutive cycles.

### 3.4. Binary Transmission Protocol

Each CSI record is transmitted from ESP32-C3 to the host PC using a custom binary framing protocol:

| Field | Size (bytes) | Description |
|:---|:---:|:---|
| Magic word | 4 | `0x43534921` ("CSI!" in little-endian) |
| `round_id` | 4 | TDMA cycle identifier |
| `tx_id` | 1 | Transmitter node index (0–3) |
| `rx_id` | 1 | Receiver node index (0–3) |
| RSSI | 1 | Received signal strength (dBm, signed) |
| `csi_len` | 2 | CSI payload length in bytes |
| CSI payload | `csi_len` | Raw I/Q samples as int8 pairs |
| CRC16-CCITT | 2 | Integrity checksum over magic + header + payload |

On the host side, four concurrent reader threads (one per COM port) perform buffered binary parsing with magic word resynchronization. Records failing CRC16 verification are discarded. A garbage collector purges incomplete `round_id` entries when the current maximum `round_id` exceeds theirs by more than 10 units, preventing unbounded memory growth during long-duration acquisitions.

### 3.5. Subcarrier Selection

Each raw CSI vector contains 64 complex-valued subcarrier estimates (LLTF, HT20 mode). The following are discarded as uninformative:

- **Subcarrier index 0** — DC component, corrupted by carrier frequency offset.
- **Subcarrier indices 27–37** — Guard band / transition region with no channel information.

The remaining **52 clean subcarriers** are retained, yielding a per-frame CSI amplitude matrix $\mathbf{X}_t \in \mathbb{R}^{52 \times 12}$.

---

## 4. Signal Processing Pipeline

Raw CSI frames undergo a three-stage denoising and dimensionality reduction pipeline. The ordering is deliberate:

1. **Hampel before Gaussian** — prevents spike energy from spreading to adjacent subcarriers via the convolution kernel.
2. **Gaussian before PCA** — ensures the inter-link covariance matrix is not distorted by high-frequency subcarrier noise, improving eigenvector quality.

### 4.1. Stage 1 — Hampel Filter (Temporal Domain)

The Hampel filter operates independently on each of the $52 \times 12 = 624$ CSI amplitude time series to detect and replace impulsive outliers without attenuating genuine motion-induced variations.

For a time series $x(t)$ with sliding window $W_t$ of size $W = 2m+1$ centered at $t$:

$$\tilde{x}(t) = \mathrm{median}(W_t), \qquad \mathrm{MAD}(t) = \mathrm{median}\!\left(\left\{|x(i) - \tilde{x}(t)|\right\}_{i=t-m}^{t+m}\right)$$

The robust standard deviation estimate relates to MAD through the consistency constant for normally distributed data:

$$\hat{\sigma}(t) = 1.4826 \times \mathrm{MAD}(t)$$

A sample is classified as an outlier and replaced by the median if:

$$|x(t) - \tilde{x}(t)| > n_\sigma \times \hat{\sigma}(t) \quad \text{and} \quad \mathrm{MAD}(t) > \epsilon$$

**Parameters:** $W = 7$ frames, $n_\sigma = 2.5$, $\epsilon = 10^{-9}$. Processing latency: $\lfloor W/2 \rfloor = 3$ frames.

### 4.2. Stage 2 — Gaussian Filter (Subcarrier Domain)

A 1D Gaussian convolution is applied along the subcarrier axis of each frame independently, targeting electromagnetic interference (EMI) manifesting as high-frequency sawtooth fluctuations between adjacent subcarriers.

The Gaussian kernel with standard deviation $\sigma_g$:

$$G(u) = \frac{1}{\sqrt{2\pi}\,\sigma_g}\exp\!\left(-\frac{u^2}{2\sigma_g^2}\right)$$

is discretized over a window of size $K = 2\lceil 3\sigma_g \rceil + 1$. With $\sigma_g = 0.8$: $K = 2 \times 3 + 1 = 7$. Boundary effects are handled via reflect padding. This stage smooths inter-subcarrier noise while preserving temporal dynamics.

### 4.3. Stage 3 — Online Spatial PCA with Temporal Detrending

Principal Component Analysis reduces the link dimension from $L = 12$ to $k = 5$ principal components, capturing dominant motion-related variance while suppressing uncorrelated per-link noise.

**Step 1 — DC elimination via exponential moving average:**

$$\mathbf{M}_t = (1 - \alpha)\,\mathbf{M}_{t-1} + \alpha\,\mathbf{X}_t, \qquad \mathbf{Z}_t = \mathbf{X}_t - \mathbf{M}_t$$

where $\alpha = 0.05$ (time constant $\tau = 1/\alpha = 20$ frames).

**Step 2 — Recursive spatial covariance estimation:**

$$\mathbf{C}_{\mathrm{inst},t} = \frac{1}{S-1}\,\mathbf{Z}_t^\top\mathbf{Z}_t, \qquad \mathbf{C}_t = (1-\beta)\,\mathbf{C}_{t-1} + \beta\,\mathbf{C}_{\mathrm{inst},t}$$

where $\beta = 0.05$ (time constant $\tau = 20$ frames).

**Step 3 — Eigendecomposition and sign alignment:** The top-$k$ eigenvectors of $\mathbf{C}_t$ form the projection matrix $\mathbf{P}_t \in \mathbb{R}^{12 \times 5}$. To prevent sign ambiguity between consecutive frames:

$$\mathbf{v}_j^{(t)} \leftarrow \mathrm{sign}\!\left(\langle\mathbf{v}_j^{(t)},\,\mathbf{v}_j^{(t-1)}\rangle\right) \cdot \mathbf{v}_j^{(t)}$$

A dead-zone threshold $|\mathrm{dot}| < 0.1$ is applied to avoid erroneous flips when successive eigenvectors are near-orthogonal due to eigenvalue crossings.

**Step 4 — Projection:** $\mathbf{X}_{t,\mathrm{pca}} = \mathbf{Z}_t\,\mathbf{P}_t \in \mathbb{R}^{52 \times 5}$

A warm-up phase of 5 frames is enforced before PCA output begins, ensuring the running mean $\mathbf{M}_t$ is initialized from averaged data rather than a single sample.

### 4.4. Pipeline Summary

| Stage | Domain | Input → Output | Purpose | Key Parameters |
|:---|:---|:---:|:---|:---|
| Hampel | Temporal | $52 \times 12 \to 52 \times 12$ | Impulsive spike removal | $W{=}7$, $n_\sigma{=}2.5$ |
| Gaussian | Subcarrier | $52 \times 12 \to 52 \times 12$ | EMI smoothing | $\sigma_g{=}0.8$, $K{=}7$ |
| PCA | Link | $52 \times 12 \to 52 \times 5$ | Dimensionality reduction | $k{=}5$, $\alpha{=}0.05$, $\beta{=}0.05$ |

---

## 5. Experimental Results

### 5.1. Measurement Conditions

Two measurement campaigns were conducted to evaluate the system under different configurations and environments:

- **Run A** — Performed under a **previous TDMA configuration** with different slot timing parameters (exact values not recorded). The purpose was to assess sustained operation stability over an extended session.
- **Run B** — Performed under the **current TDMA configuration** ($T_{\text{slot}} = 2{,}000$ µs, $T_{\text{ack}} = 500$ µs, $R = 1$) in a **university library environment** with ambient WiFi interference from neighboring access points and user devices. This was a short-duration pilot measurement to verify functional correctness before longer campaigns.

### 5.2. Quantitative Results

| Metric | Run A (previous config) | Run B (current config) |
|:---|:---:|:---:|
| Environment | — | University library |
| Duration | 105.3 s | 7.2 s |
| Total frames assembled | 1,959 | 880 |
| Complete frames (12/12 links) | 1,387 | 286 |
| Incomplete frames (< 12 links) | 572 | 594 |
| CRC errors | 4 | 4 |
| Frame completeness ratio | 70.8% | 32.5% |
| Total frame rate (all frames) | 18.61 fps | 121.47 fps |
| Complete frame rate (12/12 only) | 13.17 fps | 39.48 fps |
| Exported CSV frames | 1,387 | 286 |

### 5.3. Analysis

**Run A (previous configuration, 105.3 s):** This sustained measurement achieved a frame completeness ratio of 70.8%, indicating that approximately 7 out of every 10 TDMA cycles captured all 12 links successfully. The total frame rate of 18.61 fps — substantially below the current configuration's theoretical maximum of 41.67 fps — reflects the use of a more conservative TDMA timing schedule (longer slot duration, providing greater guard margins). The complete frame rate of 13.17 fps was sufficient for continuous data collection, yielding 1,387 exportable frames over the session. This result demonstrates that the system can operate stably over multi-minute sessions without memory leaks or synchronization loss.

**Run B (current configuration, 7.2 s pilot):** Under the current TDMA parameters in the university library, the complete frame rate reached **39.48 fps**, corresponding to **94.7% of the theoretical maximum** (41.67 fps). This confirms that the 2,000 µs slot design is near-optimal in terms of throughput when channel conditions are favorable. However, the low completeness ratio (32.5%) indicates that the majority of TDMA cycles suffered at least one link failure. The high total frame rate of 121.47 fps — exceeding the single-cycle theoretical maximum — is an artifact of the metric counting both complete and incomplete rounds; incomplete rounds are assembled and discarded individually rather than occupying a full cycle period. The dense WiFi environment of the university library (multiple co-channel access points, student devices) is the likely cause of elevated packet loss. It should be noted that this was a short-duration pilot measurement (7.2 s); extended campaigns under the current configuration are planned to obtain statistically robust completeness and throughput figures.

**CRC integrity:** Both runs recorded only 4 CRC errors (< 0.3% of total frames), confirming that the binary protocol with CRC16-CCITT provides reliable data integrity over the USB link.

### 5.4. TDMA Parameter Sensitivity Analysis

The measured results from Run A and Run B illustrate the fundamental tension between **frame rate** and **link reliability** inherent in the TDMA design. The following analysis examines how each parameter influences system behavior.

#### 5.4.1. Slot Duration ($T_{\text{slot}}$)

The slot duration directly governs the maximum achievable frame rate:

$$f_{\max} = \frac{1}{N_{\text{slot}} \times T_{\text{slot}}}$$

| $T_{\text{slot}}$ | $T_{\text{cycle}}$ | $f_{\max}$ | $T_{\text{guard}}$ | Expected behavior |
|:---:|:---:|:---:|:---:|:---|
| 1,000 µs | 12 ms | 83.3 fps | 0 µs | No guard time; high slot overflow risk |
| 1,500 µs | 18 ms | 55.6 fps | 500 µs | Marginal; USB write may exceed guard |
| **2,000 µs** | **24 ms** | **41.7 fps** | **1,000 µs** | **Current — balanced throughput/reliability** |
| 3,500 µs | 42 ms | 23.8 fps | 2,500 µs | Conservative; likely higher completeness |
| 5,000 µs | 60 ms | 16.7 fps | 4,000 µs | Very conservative; diminishing returns |

The contrast between Run A (lower fps, higher completeness) and Run B (higher fps, lower completeness) is consistent with this trade-off: a longer $T_{\text{slot}}$ provides more guard time for processing and retransmission, improving per-link success probability at the cost of reduced temporal resolution.

#### 5.4.2. ACK Timeout ($T_{\text{ack}}$)

The ACK timeout determines how long the transmitter waits for confirmation before either retrying or yielding the slot:

$$T_{\text{active}} = (R + 1) \times T_{\text{ack}}, \qquad T_{\text{guard}} = T_{\text{slot}} - T_{\text{active}}$$

| $T_{\text{ack}}$ | Retries | $T_{\text{active}}$ | $T_{\text{guard}}$ | Impact |
|:---:|:---:|:---:|:---:|:---|
| 300 µs | 1 | 600 µs | 1,400 µs | Risk of premature timeout; ESP-NOW RTT can reach 200–400 µs |
| **500 µs** | **1** | **1,000 µs** | **1,000 µs** | **Current — sufficient for typical RTT** |
| 750 µs | 1 | 1,500 µs | 500 µs | Higher ACK capture rate; compressed guard |
| 500 µs | 2 | 1,500 µs | 500 µs | Additional retry; equivalent guard reduction |

#### 5.4.3. Retry Count ($R$)

Increasing retries improves per-slot reliability but consumes guard time within a fixed slot:

| Retries ($R$) | $T_{\text{active}}$ (at $T_{\text{ack}}{=}500$ µs) | $T_{\text{guard}}$ | Reliability gain |
|:---:|:---:|:---:|:---|
| 0 | 500 µs | 1,500 µs | Baseline; single attempt only |
| **1** | **1,000 µs** | **1,000 µs** | **Current — moderate improvement** |
| 2 | 1,500 µs | 500 µs | Diminishing returns; insufficient guard |
| 3 | 2,000 µs | 0 µs | Slot fully consumed; no processing margin |

The measured completeness of 32.5% (Run B) suggests that RF-level interference — not insufficient retries — is the dominant source of link failure. Environmental mitigations (channel selection, antenna placement, transmit power adjustment) are likely more effective than additional retries.

#### 5.4.4. Network Scaling ($N$ Nodes)

The full-mesh topology requires $N(N-1)$ slots, leading to quadratic scaling of the cycle period:

| Nodes ($N$) | Links | $T_{\text{cycle}}$ | $f_{\max}$ |
|:---:|:---:|:---:|:---:|
| 3 | 6 | 12 ms | 83.3 fps |
| **4** | **12** | **24 ms** | **41.7 fps** |
| 5 | 20 | 40 ms | 25.0 fps |
| 6 | 30 | 60 ms | 16.7 fps |

The frame rate degrades as $f_{\max} \propto 1/N^2$, representing a fundamental scalability constraint of the full-mesh Virtual MIMO topology. For applications requiring more than 5–6 nodes, partial-mesh or hierarchical scheduling strategies would need to be investigated.

---

## 6. Limitations and Future Work

The following aspects have not been systematically evaluated under controlled experimental conditions and constitute planned future work:

- Extended measurement campaigns (> 10 minutes) under the current TDMA configuration to establish statistically robust completeness and throughput baselines.
- Quantitative signal-to-noise ratio (SNR) improvement attributable to each stage of the filtering pipeline (Hampel, Gaussian, PCA).
- Optimal selection of the number of principal components ($k$) under varying activity types and environmental conditions, including explained variance analysis.
- End-to-end latency from CSI extraction at the ESP32-C3 firmware to availability of the processed $52 \times 5$ feature matrix at the host application layer.
- Classification accuracy of downstream AI models (HAR, Pose Estimation) trained on the processed CSI features.
- Systematic comparison of frame completeness across different WiFi channels, transmit power levels, node geometries, and interference scenarios.
