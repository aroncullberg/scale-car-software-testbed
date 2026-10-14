# Saleae Logic 2 Bidirectional DShot (BDshot) Analyzer Extension

A custom **High-Level Analyzer (HLA)** for Saleae Logic 2 to decode and analyze **Bidirectional DShot** (BDshot) and standard DShot protocols (DShot150, DShot300, DShot600, DShot1200).

---

## Features

- **Forward Command Decoding (Flight Controller $\rightarrow$ ESC)**:
  - 16-bit DShot command & throttle decoding (0 to 2047 steps, 0.0% to 100.0%).
  - Special command identification (Disarm, Beep, 3D Mode, Spin Direction, ESC Info, etc.).
  - Telemetry Request flag detection.
  - Inverted / Non-inverted 4-bit CRC verification.
- **Reverse Telemetry Decoding (ESC $\rightarrow$ Flight Controller)**:
  - 20-bit / 21-bit GCR (Group Code Recording) 5b/4b hardware table decoding.
  - Electrical RPM (eRPM) period extraction ($T_{\text{period}} = \text{mantissa} \times 2^{\text{exponent}}$).
  - True mechanical RPM calculation based on configurable motor poles (e.g. 14 poles / 7 pole-pairs).
  - Extended DShot Telemetry (EDT) parsing (Temperature °C, Voltage V, Current A, Debug).
  - ESC CRC verification and telemetry signal health checking.

---

## How to Install in Saleae Logic 2

1. Open **Saleae Logic 2**.
2. Click the **Extensions** icon in the right-hand panel (puzzle piece).
3. Click the three dots menu (**...**) in the Extensions panel and select **Load Existing Extension...**.
4. Browse to this folder:
   ```
   tools/saleae_bidirectional_dshot
   ```
5. Select the `extension.json` file. Logic 2 will load the **Bidirectional DShot Analyzer**.

---

## How to Configure in Logic 2

High-Level Analyzers (HLAs) in Logic 2 attach to a low-level analyzer. Follow these 2 quick steps:

### Step 1: Add the "Async Serial" Low-Level Analyzer
In Logic 2, click **Analyzers (+)** $\rightarrow$ **Async Serial**:

| Setting | Value for DShot600 | Value for DShot300 | Value for DShot150 |
| :--- | :--- | :--- | :--- |
| **Input Channel** | Your DShot Pin | Your DShot Pin | Your DShot Pin |
| **Bit Rate (Baud)** | `600000` | `300000` | `150000` |
| **Bits per Transfer** | `8 bits` | `8 bits` | `8 bits` |
| **Stop Bits** | `1 Stop Bit` | `1 Stop Bit` | `1 Stop Bit` |
| **Parity Bit** | `No Parity` | `No Parity` | `No Parity` |
| **Significant Bit** | `MSB first` | `MSB first` | `MSB first` |
| **Signal Inversion** | **Non-Inverted** (Standard) | **Non-Inverted** (Standard) | **Non-Inverted** (Standard) |

*(Note: For Bidirectional DShot, keep "Inverted" OFF / Normal, because the line idles HIGH and active-low pulses act as the UART start transition).*

### Step 2: Add the "Bidirectional DShot" HLA
Click **Analyzers (+)** $\rightarrow$ **Bidirectional DShot**:
- **Input Analyzer**: Select the **Async Serial** analyzer created in Step 1.
- **DShot Speed**: `DShot600` (or your target speed).
- **Mode / Polarity**: `Bidirectional (Inverted / Idle HIGH)`.
- **Motor Poles**: `14` (set to your motor magnet count for accurate RPM).

---

## Decoded Output Frames

- **`FC: Throttle=1048 (50.0%) [ReqTelem=0] CRC: OK`**
- **`FC CMD: ESC INFO (6) [ReqTelem=1] CRC: OK`**
- **`ESC: 12450 RPM (eRPM=87150, Period=688µs) CRC: OK`**
- **`ESC EDT: Voltage = 15.20 V CRC: OK`**
- **`ESC: Motor Stopped (eRPM=0) CRC: OK`**
