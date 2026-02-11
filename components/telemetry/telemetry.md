# Telemetry Component

## Architecture

Central MPMC (Multi-Producer, Multi-Consumer) dispatcher built on FreeRTOS queues. Each telemetry type gets a **Channel** with two queues:

- **Mailbox** (depth 1, `xQueueOverwrite`) — always holds the most recent sample, old data silently replaced
- **Ring** (depth 32, `xQueueSend`) — short FIFO history for logging/recording; drops newest if full

All queue operations use 0-tick timeout so producers never block.

## Data Types

Defined in `include/telemetry_types.h`:

| Struct | Fields | Typical Producer |
|---|---|---|
| `BatteryTelemetry` | voltage_mv, current_ma, capacity_mah, remaining_pct | voltage_sensor (planned) |
| `GpsTelemetry` | lat/lon (1e7), speed_kmh, heading_deg, altitude_m, satellites | proto_nmea NmeaDriver |
| `AttitudeTelemetry` | roll/pitch/yaw (float deg) | imu (commented out) |
| `RpmTelemetry` | source_id, rpm_values[4], count | vehicle::Controller |
| `AccelGyroTelemetry` | sample_time_us, 3-axis gyro (rad/s) + accel (m/s²) | imu (commented out) |
| `LinkStatsTelemetry` | rssi, link_quality, snr, good/bad frames | — |
| `AirspeedTelemetry` | speed_kmh_x10 | — |
| `FlightModeTelemetry` | mode[32] | — |
| `TempTelemetry` | source_id, temps[8], count | — |

## Data Flow

```
  Producers                    Dispatcher                  Consumers
  ─────────                    ──────────                  ─────────
  NmeaDriver ──publish()──►  ┌─ mailbox ─┐  ◄──poll()──  ELRS backend (~10Hz)
  vehicle    ──publish()──►  │            │               → CRSF packets → RC TX
  (imu)      ──publish()──►  ├─ ring ─────┤  ◄──drain()── telem_log task (10Hz)
                             └────────────┘               → ESP_LOGD
```

## Consumers

### ExpressLRS RC Backend (`elrs.cpp`)
Every 10th valid RC frame (~100ms / ~10Hz), polls `telemetry::poll_latest()` for all types. Converts to CRSF wire format and transmits over UART back to the RC transmitter. Unit conversions happen here (e.g. degrees → 100µrad for attitude, rad/s → CRSF gyro LSB).

### Logger Task (`telemetry.cpp`)
Runs at 10Hz, drains ring queues in batches of 8 and logs via `ESP_LOGD`. Intended for future SD card / flash logging.

## Thread Safety

- FreeRTOS queues provide all synchronization between tasks/cores
- Producers (vehicle controller on Core 1, NMEA driver on Core 1) and consumers (ELRS on Core 1, logger on any core) are fully decoupled
- Non-blocking publishes guarantee control loops are never stalled by telemetry
