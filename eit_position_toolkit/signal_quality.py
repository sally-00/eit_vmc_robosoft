"""Gain-dependent ADC range estimates; no hardware access or gain changes."""
import math
import numpy as np
from eit_reader import gain_exponent

# Supplied v1.1.1 app.c: signed ADC codes -2048..2047, 1 mV per count.
# Use the smaller positive rail, conservatively, for a zero-offset sinusoid.
ADC_PEAK_LIMIT_V = 2.047


def receiver_limits(device, channels, adc_peak_limit_v=ADC_PEAK_LIMIT_V):
    if not math.isfinite(adc_peak_limit_v) or adc_peak_limit_v <= 0:
        raise ValueError("ADC peak limit must be positive")
    unknown = np.full(channels, np.nan)
    device = device or {}
    rows = device.get("gain_dump")
    if device.get("demo") or not rows:
        return unknown.copy(), unknown.copy()
    if len(rows) != channels:
        raise ValueError("Gain dump and capture channel counts differ")
    protocol = device.get("configuration_order_txp_txn_rxp_rxn")
    if protocol is not None and len(protocol) != channels:
        raise ValueError("Gain dump and electrode protocol counts differ")
    gains = []
    for i, row in enumerate(rows):
        fields = row.split()
        if len(fields) != 6:
            raise ValueError("Malformed saved gain row")
        pins = tuple(map(int, fields[:4]))
        if protocol is not None and pins != tuple(protocol[i]):
            raise ValueError("Gain row does not match channel electrode order")
        gains.append(2. ** gain_exponent(fields[5]))
    gains = np.asarray(gains)
    # Reported pre-RX RMS * RX gain * sqrt(2) estimates ADC sine peak.
    return gains, adc_peak_limit_v / (math.sqrt(2) * gains)


def quality_flags(summary, warning_percent=80., low_percent=5.):
    flags = []
    for i, value in enumerate(summary['usage_percent']):
        row = []
        if summary['clipped'][i]:row.append('CLIPPED: amplitude unreliable')
        if summary['zero_channels'][i]:row.append('ZERO in capture')
        if np.isfinite(value) and not summary['clipped'][i]:
            if value >= 100:row.append('AT/ABOVE estimated sine limit')
            elif value >= warning_percent:row.append('near estimated limit')
            elif value < low_percent:row.append('low range use')
        if summary['frame_noise_percent'][i] > 2:row.append('varies >2% within capture')
        flags.append('; '.join(row) or ('range unknown' if not np.isfinite(value) else 'no flag'))
    return flags
