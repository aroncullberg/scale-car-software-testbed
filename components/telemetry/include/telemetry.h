#pragma once
#include "telemetry_types.h"

namespace telemetry {

void init();

void publish(const AttitudeTelemetry& data);
void publish(const RpmTelemetry& data);
void publish(const GpsTelemetry& data);
void publish(const BatteryTelemetry& data);
void publish(const AirspeedTelemetry& data);
void publish(const FlightModeTelemetry& data);
void publish(const TempTelemetry& data);
void publish(const AccelGyroTelemetry& data);

bool poll_latest(AttitudeTelemetry& out);
bool poll_latest(RpmTelemetry& out);
bool poll_latest(GpsTelemetry& out);
bool poll_latest(BatteryTelemetry& out);
bool poll_latest(AirspeedTelemetry& out);
bool poll_latest(FlightModeTelemetry& out);
bool poll_latest(TempTelemetry& out);
bool poll_latest(AccelGyroTelemetry& out);

}
