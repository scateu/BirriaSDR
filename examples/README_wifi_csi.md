# Export channel state information from BirriaSDR I/Q

The `birria_wifi` decoder can export software-derived SISO channel state information from the legacy long training field of each detected 20 MHz OFDM packet. The exporter writes 52 active complex subcarriers with packet timing, PHY and MAC metadata.

This guide covers CSI collection, record formats, validity, interpretation,
calibration limits and verification. The [CSI principle guide](README_wifi_csi_principle.md)
explains the estimate. The [main Wi-Fi decoder guide](README_wifi.md) covers
native USB, UDP and file input, sweep mode and frame decoding.

> [!IMPORTANT]
>
> The exported values are receiver channel estimates, not calibrated absolute
> propagation measurements. They include transmitter response, receiver
> response, gain, timing, frequency error and the current rate conversion.

## Build and test

Build the decoder from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j --target birria_wifi birria_udp
```

Run the self-test, which includes synthetic complex-channel recovery, carrier-offset correction, normalization and binary serialization:

```sh
./build/birria_wifi --self-test
```

CTest exposes the same check:

```sh
ctest --test-dir build -R wifi_decoder_self_test --output-on-failure
```

## Collect live CSI

Native USB capture is the default. This command tunes device 0 to channel 6 at
40 MSps and writes FCS-valid CSI records:

```sh
./build/birria_wifi \
    --frequency 2437000000 \
    --manual \
    --csi live.bcsi
```

The sweep mode can export CSI while visiting channels 1 through 13. Every record
contains the center frequency associated with its decoder epoch:

```sh
./build/birria_wifi \
    --sweep \
    --dwell-ms 500 \
    --csi sweep.jsonl \
    --csi-format jsonl \
    --manual
```

The experimental 80 MSps mode software-sweeps the channels inside one wide
capture. The CSI still represents one selected 20 MHz channel per dwell.

## Collect CSI through UDP

Select UDP explicitly and start the decoder before the sender:

```sh
./build/birria_wifi \
    --udp \
    --bind 127.0.0.1 \
    --port 1235 \
    --frequency 2437000000 \
    --sample-rate 40000000 \
    --csi live.bcsi

./build/birria_udp \
    --host 127.0.0.1 \
    --port 1235 \
    --frequency 2437000000 \
    --sample-rate 40000000 \
    --manual
```

Use a different port from SDR++ unless a fanout mechanism duplicates the
samples. The raw UDP protocol cannot carry sweep epochs.

## Export CSI from a recording

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --frequency 2437000000 \
    --sample-rate 40000000 \
    --csi capture.bcsi
```

The input must be continuous little-endian CS16 at the specified rate. Do not
use the same path for `--file` and `--csi`.

## CSI command line options

| Option | Default | Purpose and limits |
| --- | --- | --- |
| `--csi PATH` | Disabled | Writes CSI records, and `-` selects standard output |
| `--csi-format FORMAT` | `binary` | Selects `binary` or `jsonl` |
| `--csi-invalid` | Disabled | Includes SIGNAL-valid candidates whose MAC FCS failed |
| `--csi-normalize` | Disabled | Adds a unit root mean square magnitude vector after the raw vector |
| `--frequency HZ` | Channel 6 in native mode | Tunes native capture or labels file and UDP input |
| `--sample-rate RATE` | `40000000` | Selects the native or input sample rate |
| `--sweep` | Disabled | Exports CSI while native capture visits channels 1 through 13 |
| `--file PATH` | Disabled | Reads CSI source samples from a recorder file |
| `--udp` | Disabled | Selects UDP instead of native USB capture |
| `--bind ADDRESS` | `0.0.0.0` | Selects the UDP bind address |
| `--port PORT` | `1234` | Selects the UDP port |

The CSI-specific format and validity options require `--csi`. Frame summaries normally remain on standard output. When `--csi -` selects standard output, frame summaries move to standard error so they cannot corrupt the CSI stream.

## Estimator and subcarrier map

The decoder estimates a complex channel response from the two legacy long training symbols of each detected packet. For active subcarrier `k`, the estimate is:

```text
H[k] = (Y1[k] + Y2[k]) / (2 Xltf[k])
```

The `Y1` and `Y2` terms are carrier-offset-corrected FFT outputs, and `Xltf` is the known legacy long-training value. The export contains the 52 active legacy OFDM subcarriers in this fixed order:

```text
-26, -25, ... -2, -1, +1, +2, ... +25, +26
```

The DC subcarrier and guard subcarriers are omitted. The subcarrier spacing is 312.5 kHz for the 20 MHz waveform.

## Write binary CSI

Binary output is compact and is the default CSI format:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --frequency 2437000000 \
    --csi capture.bcsi
```

Without `--csi-invalid`, the file contains CSI only for packets whose SIGNAL field and MAC FCS both passed. This is the recommended default because a valid FCS strongly supports the packet timing, carrier correction and channel estimate associated with that frame.

Use the normalized option when an additional vector with unit root mean square magnitude is useful:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --frequency 2437000000 \
    --csi capture.bcsi \
    --csi-normalize
```

The raw vector is always retained. Normalization does not replace it.

## Write JSON Lines CSI

The JSON Lines format is larger but convenient for inspection and scripts:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --frequency 2437000000 \
    --csi capture.jsonl \
    --csi-format jsonl \
    --csi-normalize
```

Each line is one JSON object. The `subcarriers` array gives the signed subcarrier indices. The `csi` array contains `[real, imaginary]` pairs in the same order. The optional `csi_normalized` array has the same layout.

The `snr_db` field estimates signal-to-noise ratio from the two repeated
long-training symbols. It is a packet quality estimate rather than a calibrated
receiver measurement.

Use `--csi -` to write either CSI format to standard output. Frame summaries move to standard error in that mode so they do not corrupt the CSI stream:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --csi - \
    --csi-format jsonl
```

## Include packets with a failed FCS

The invalid option preserves CSI for a candidate that had a valid legacy SIGNAL field but whose decoded PSDU failed its FCS:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --csi candidates.jsonl \
    --csi-format jsonl \
    --csi-invalid
```

These records carry `fcs_valid=false` and `mac_valid=false`. Their type, subtype and MAC addresses are null in JSON and are not trustworthy. They can be useful for channel sensing experiments, but UDP loss, interference, unsupported PHY formats or a false candidate can also produce them.

## Binary record format

Every binary record is self-delimiting and little-endian. The first version uses a 96-byte header followed by 52 complex float32 values. The normalized vector, when requested, follows the raw vector.

| Byte offset | Size | Field |
| --- | --- | --- |
| `0` | 4 | The ASCII magic `BCSI` |
| `4` | 2 | Format version, currently 1 |
| `6` | 2 | Header size, currently 96 bytes |
| `8` | 4 | Total record size in bytes |
| `12` | 4 | Flags described below |
| `16` | 8 | CSI record sequence starting at zero |
| `24` | 8 | Input complex-sample index at the selected input rate |
| `32` | 8 | Center-frequency label in hertz, or zero |
| `40` | 4 | Carrier-frequency offset in hertz as float32 |
| `44` | 4 | Minimum normalized correlation of the two long-training symbols as float32 |
| `48` | 4 | Reserved float32, currently zero |
| `52` | 2 | Legacy OFDM rate in megabits per second |
| `54` | 2 | PSDU length from the SIGNAL field |
| `56` | 1 | MAC frame type when the MAC-valid flag is set |
| `57` | 1 | MAC frame subtype when the MAC-valid flag is set |
| `58` | 1 | FCS-valid byte, zero or one |
| `59` | 1 | Number of CSI subcarriers, currently 52 |
| `60` | 6 | Destination address, or all zero when unavailable |
| `66` | 6 | Source address, or all zero when unavailable |
| `72` | 6 | BSSID, or all zero when unavailable |
| `78` | 18 | Reserved bytes, currently zero |
| `96` | 416 | Raw CSI, 52 little-endian float32 real and imaginary pairs |
| `512` | 416 | Optional normalized CSI when flag bit 1 is set |

The flags word uses these bits:

| Flag bit | Meaning |
| --- | --- |
| `0` | The MAC FCS passed |
| `1` | A normalized CSI vector follows the raw vector |
| `2` | The legacy SIGNAL field passed, always set in version 1 records |
| `3` | The MAC type, subtype and address fields are valid |

A record without normalized CSI is 512 bytes. A record with normalized CSI is 928 bytes. Version 1 fixes the subcarrier order shown above, so the binary record does not repeat the index array.

This Python example reads version 1 records and returns their raw complex vectors:

```python
import struct


def read_bcsi(path):
    with open(path, "rb") as source:
        while header := source.read(96):
            if len(header) != 96 or header[:4] != b"BCSI":
                raise ValueError("invalid BCSI header")
            version, header_bytes, record_bytes, flags = struct.unpack_from(
                "<HHII", header, 4
            )
            if version != 1 or header_bytes != 96 or record_bytes not in (512, 928):
                raise ValueError("unsupported BCSI record")
            sequence, sample, center = struct.unpack_from("<QQQ", header, 16)
            cfo_hz, correlation = struct.unpack_from("<ff", header, 40)
            rate, length = struct.unpack_from("<HH", header, 52)
            payload = source.read(record_bytes - header_bytes)
            if len(payload) != record_bytes - header_bytes:
                raise ValueError("truncated BCSI record")
            pairs = struct.unpack_from("<104f", payload)
            csi = [complex(pairs[i], pairs[i + 1]) for i in range(0, 104, 2)]
            yield sequence, sample, center, cfo_hz, correlation, rate, length, flags, csi
```

## CSI interpretation and calibration

The raw estimate includes the entire measurement chain:

```text
Hmeasured[k] = Htransmitter[k] Hpropagation[k] Hreceiver[k]
```

The result therefore includes transmitter filtering, propagation, receiver filtering, gain, packet timing and the current moving-average native-rate to 20 MSps conversion. It is useful for relative packet-to-packet measurements, but it is not a calibrated absolute propagation channel.

The raw amplitude is in receiver sample units. The normalized vector divides all 52 complex values by their root mean square magnitude. This removes one packet-wide amplitude factor, but it does not remove per-subcarrier receiver response, common phase, linear phase slope or transmitter response.

The current center-frequency depression can affect active subcarriers near DC, especially indices `-1` and `+1`. Preserve raw records and capture a stable reference before applying experimental amplitude or phase calibration. A practical relative calibration divides each measured subcarrier by an averaged complex reference taken with the same center frequency, gain settings and processing version.

The single BirriaSDR stream produces SISO CSI from one receive chain. It cannot provide a multi-antenna channel matrix, angle of arrival or receive-side MIMO CSI without simultaneous synchronized capture from additional chains.


## Validity levels

The exporter distinguishes three useful validity levels:

| Level | Requirement | Export behavior |
| --- | --- | --- |
| Training valid | The short and long training detectors accepted a candidate | Internal only |
| SIGNAL valid | The legacy rate code, parity, tail and PSDU length passed | Exported only with `--csi-invalid` when the FCS later fails |
| Frame valid | The SIGNAL field passed and IEEE CRC32 matches the received FCS | Exported by default |

The default policy is deliberately conservative. A valid FCS makes it highly likely that the CSI record belongs to the decoded frame and that packet timing, carrier correction and channel estimation were coherent enough to recover the PSDU.

The invalid mode can preserve useful sensing observations, but those records can also result from UDP loss, interference, an unsupported packet format or a false candidate. Version 1 marks their MAC fields invalid and does not expose decoded addresses as trustworthy metadata.

## Phase and amplitude guidance

The raw complex phase contains several terms beyond propagation:

- A packet-wide common phase from oscillator and synchronization error.
- A linear phase slope from packet timing and sampling offset.
- Per-subcarrier transmitter and receiver response.
- Environmental multipath.

The exporter currently preserves these terms. It does not silently remove common phase or fit a linear slope. This is important because different experiments need different corrections.

The normalized vector removes only one packet-wide amplitude scale. It does not calibrate the passband, flatten the center depression or correct phase. Preserve the raw vector whenever derived processing is applied.

For relative calibration, collect many packets in a stable reference geometry with identical center frequency, gain codes and software version. Average the complex reference per subcarrier, then divide later measurements by that reference. Reject subcarriers whose reference magnitude is too small because division would amplify noise.

## Current limitations

- The estimate comes from the legacy 20 MHz L-LTF only.
- The result is one receive chain by one legacy spatial stream.
- The decoder does not export HT-LTF, VHT-LTF, HE-LTF or EHT-LTF estimates.
- The decoder does not search offset channels within the 40 MHz input.
- The adjacent-pair rate conversion is fast but is not a qualified channelizing FIR.
- The output is not calibrated in volts, dBm, delay or distance.
- The output cannot provide angle of arrival or a receive-side MIMO matrix.
- The UDP source has no sequence number, so packet loss is inferred only through decoder failures.
- The center-frequency depression under investigation can bias subcarriers near DC.

## Verification performed

The implementation has been checked in these ways:

- A synthetic frequency-selective complex channel is applied to the known long-training sequence and recovered within a small numerical tolerance.
- A synthetic carrier offset is applied and corrected before channel estimation.
- The normalized vector is checked for unit root mean square magnitude.
- Binary records are checked for magic, version, sizes, flags and subcarrier count.
- The existing recorder file produces one FCS-valid 52-subcarrier record for its decoded CTS frame.
- The invalid export mode produces one valid record and three FCS-failed SIGNAL-valid records from that file.
- A localhost UDP replay produces the same FCS-valid CSI record.
- Binary, normalized binary, JSON Lines and standard-output modes have been parsed after generation.

These checks validate the implemented signal path and serialization. They do not establish absolute channel calibration or qualify every supported legacy OFDM rate over the air.
