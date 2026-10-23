# Decode Wi-Fi frames from BirriaSDR I/Q

The `birria_wifi` example decodes legacy 20 MHz IEEE 802.11 OFDM frames. Native
USB capture is the default. The decoder can also read a recorder file or receive
little-endian CS16 over UDP.

The decoder links to `libbirriasdr` for native capture and uses no external DSP
library. It validates each decoded frame with IEEE CRC32 and prints MAC
information to standard output.

> [!IMPORTANT]
>
> Receive wireless traffic only where local law and your authorization permit
> it. Management and control frames can expose network names and device
> addresses even when data payloads are encrypted.

## Build and test the decoder

Build from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j --target birria_wifi birria_udp birria_rx
./build/birria_wifi --self-test
```

A successful test prints:

```text
Wi-Fi decoder self-test passed
```

## Capture directly from USB

The default command opens native device 0 at 40 MSps and tunes to Wi-Fi channel
6 at 2,437 MHz:

```sh
./build/birria_wifi
```

Select another center or device when needed:

```sh
./build/birria_wifi \
    --device 1 \
    --frequency 2412000000 \
    --manual
```

The manual option applies LNA 2, filter stage 8 and digital 0. Decoder work runs
on a separate worker so packet processing does not block USB delivery. A decoder
queue overflow is reported as an error rather than hidden.

## Sweep the 2.4 GHz Wi-Fi channels

The sweep mode monitors channel centers 1 through 13:

```sh
./build/birria_wifi \
    --sweep \
    --dwell-ms 500 \
    --manual
```

The default 40 MSps mode live-retunes through the thirteen centers. The decoder
discards a settling interval after each retune and starts a new configuration
epoch.

The experimental 80 MSps mode keeps one wide capture centered at 2,442 MHz. It
software-sweeps the thirteen 20 MHz channel centers without RF retuning:

```sh
./build/birria_wifi \
    --sample-rate 80000000 \
    --sweep \
    --dwell-ms 200 \
    --manual
```

The 80 MSps mode requires about 320 MB/s of CS16 payload. It decoded valid
legacy OFDM frames during native testing on October 23, 2026, but it remains
experimental and its wide passband is not calibrated.

The optional 160 MSps mode prints a warning and disables recovery. The current
CS16 USB exporter is expected to overrun, so the first overrun stops the run:

```sh
./build/birria_wifi --sample-rate 160000000 --sweep --manual
```

## Receive UDP optionally

Start the decoder before the UDP sender:

```sh
./build/birria_wifi \
    --udp \
    --bind 127.0.0.1 \
    --port 1235 \
    --frequency 2437000000 \
    --sample-rate 40000000
```

Start the matching sender in another terminal:

```sh
./build/birria_udp \
    --host 127.0.0.1 \
    --port 1235 \
    --frequency 2437000000 \
    --sample-rate 40000000 \
    --manual
```

The frequency and sample rate must match. The UDP mode cannot sweep because raw
datagrams contain no return control, frequency metadata or configuration epoch.

## Decode a recorder file

The decoder can read a little-endian CS16 file written by `birria_rx`:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --frequency 2437000000 \
    --sample-rate 40000000
```

The sample rate and center must describe the recording. The file path, UDP mode
and native USB capture are mutually exclusive.

Do not use the same path for `--file` and `--csi`. The decoder rejects an exact
path match to prevent an output file from replacing its input.

## Command line options

Run the help command for the current option summary:

```sh
./build/birria_wifi --help
```

| Option | Default | Purpose and limits |
| --- | --- | --- |
| `--udp` | Disabled | Selects UDP instead of native USB capture |
| `--bind ADDRESS` | `0.0.0.0` | Selects the local UDP address |
| `--port PORT` | `1234` | Selects the local UDP port |
| `--file PATH` | Disabled | Reads CS16 from a file instead of USB or UDP |
| `--frequency HZ` | `2437000000` in native mode | Selects native tuning or labels file and UDP input |
| `--device INDEX` | `0` | Selects the native USB device |
| `--sample-rate RATE` | `40000000` | Selects 40, 80 or 160 MSps |
| `--manual` | Disabled | Selects LNA 2, filter stage 8 and digital 0 for native capture |
| `--recovery COUNT` | `3` | Selects the native full restart budget |
| `--sweep` | Disabled | Monitors Wi-Fi channel centers 1 through 13 in native mode |
| `--dwell-ms MS` | `500` | Selects sweep dwell per channel |
| `--csi PATH` | Disabled | Writes CSI records, and `-` selects standard output |
| `--csi-format FORMAT` | `binary` | Selects `binary` or `jsonl`, and requires `--csi` |
| `--csi-invalid` | Disabled | Includes SIGNAL-valid candidates whose MAC FCS failed |
| `--csi-normalize` | Disabled | Adds a unit-RMS CSI vector |
| `--compact` | Disabled | Prints one compact line per valid frame |
| `--interactive` | Disabled | Shows a deduplicated live terminal table with count and last-seen age |
| `--verbose` | Disabled | Prints rejected PSDU diagnostics |
| `--self-test` | Disabled | Tests coding, CSI, CRC and MAC parsing |
| `--help` | Not applicable | Prints the option summary and exits |

Native-only options are rejected with UDP and file input. The sweep option is
native-only and cannot be combined with an explicit frequency.

## Output

A valid decoded frame produces one summary line. This example came from an existing 40 MSps capture:

```text
frame time=0.007555s sample=302184 center=2437000000Hz rate=6Mbps length=14 type=control subtype=cts dst=e0:3e:44:08:34:da cfo=-13670.9Hz rssi=-44.6dBFS snr=15.0dB ltf=0.705 fcs=ok
```

| Field | Meaning |
| --- | --- |
| `time` | Position of the detected long training field, derived from the selected native input sample count |
| `sample` | Input complex-sample index at the detected long training field |
| `center` | Label supplied through `--frequency`, omitted when no label was supplied |
| `rate` | Legacy OFDM PHY rate decoded from the SIGNAL field |
| `length` | PSDU length from the SIGNAL field, including the four-byte FCS |
| `type` | IEEE 802.11 MAC frame type |
| `subtype` | Decoded management or control subtype, or `data` for a data frame |
| `dst` | Destination or receiver address according to the available MAC header fields |
| `src` | Source or transmitter address when the frame format carries one |
| `bssid` | Basic service set identifier when the frame format carries one |
| `cfo` | Estimated carrier-frequency offset in hertz |
| `rssi` | Uncalibrated long-training power relative to the signed 16-bit numeric range |
| `snr` | Signal-to-noise estimate from the difference between the two long-training symbols |
| `ltf` | Minimum normalized correlation score for the two long-training symbols |
| `fcs` | The value `ok`, because frames with a failed FCS are not printed to standard output |

The decoder maps data-frame addresses according to the To DS and From DS bits. Four-address data frames use address 3 as the destination and address 4 as the source. Control frames expose only the addresses present in their shorter subtype-specific headers.

The `rssi` value is not calibrated in dBm. It is useful for comparisons within a
capture that keeps the same gain settings. The `snr` value is an estimate from
the repeated long-training symbols, and interference or channel variation can
bias it.


Detailed output remains the default. The compact option emits one line and
suppresses the second beacon or probe-response detail line:

```sh
./build/birria_wifi --compact
```

A compact beacon has this form:

```text
wifi time=0.103745s sample=4149784 center=2412000000Hz rate=6Mbps pdu=beacon len=443 src=88:9e:96:a0:b4:8b dst=ff:ff:ff:ff:ff:ff ssid="example" channel=1 rssi=-38.5dBFS snr=18.7dB fcs=ok
```

The compact record keeps timing, center, rate, frame subtype, length, available
addresses, SSID and advertised channel, signal estimates and FCS state.

The interactive mode opens an ANSI terminal screen:

```sh
./build/birria_wifi --sweep --interactive --manual
```

The screen combines repeated frames into one row. Beacon and probe-response
traffic is grouped by BSSID. Other traffic is grouped by subtype and available
source, destination and BSSID addresses. Rows are sorted by descending count,
with most recent activity as the tie breaker. The aligned columns are
`LAST SEEN`, `COUNT`, `Center/MHz`, `Rate/Mbps`, `PDU`, `RSSI/dBFS`, `SNR/dB`,
`Address` and `Details`. The table is limited to the terminal height and retains
at most 2,048 unique rows.

The compact and interactive options are mutually exclusive. Interactive mode
cannot be combined with rejected-packet diagnostics from `--verbose` and
requires standard output to be a terminal and cannot be combined with
`--csi -` because that option reserves standard output for CSI data.

### Beacon and probe response output

A valid legacy OFDM beacon or probe response produces a second line with selected information elements:

```text
beacon bssid=02:11:22:33:44:55 ssid="example" channel=6 interval=102.400ms capability=0x0431
```

The decoder extracts these values:

- The BSSID comes from address 3 in the management header.
- The SSID comes from information element 0. An absent or zero-length element prints as `<hidden>`.
- The advertised channel comes from information element 3 when present.
- The beacon interval comes from the fixed parameters and is converted from time units to milliseconds.
- The capability value is printed as a hexadecimal word without interpreting individual flags.

Malformed or truncated information elements stop element parsing for that frame, but the frame summary still prints when its FCS is valid.

## Export channel state information

The decoder can export 52 active legacy OFDM subcarriers as raw complex CSI, with optional normalization and binary or JSON Lines output. Packets with a valid FCS are exported by default, and an explicit option can retain SIGNAL-valid candidates whose FCS failed. See the [complete CSI export guide](README_wifi_csi.md) for live and recorded collection, record layouts, parsing, validity, calibration guidance, limitations and verification.

## Input and transport contract

Native capture supplies native-endian CS16 directly from `libbirriasdr`. File
and UDP input use little-endian CS16. Every source must use one of the supported
native rates:

| Property | Required value |
| --- | --- |
| Sample rate | 40 MSps by default, or experimental 80 and 160 MSps |
| Sample type | Interleaved signed 16-bit I and Q |
| Source range | Normally minus 8192 to 8191 for each BirriaSDR component |
| UDP metadata | None |
| UDP datagram alignment | A whole number of four-byte complex samples |
| Expected channel | One selected 20 MHz channel after native tuning or software translation |

The UDP protocol has no sequence number, timestamp, acknowledgement or tuning
control. A datagram loss commonly causes a SIGNAL rejection or FCS failure.
Do not bind SDR++ and `birria_wifi` to the same unicast address and port.

The payload rates are 160 MB/s at 40 MSps, 320 MB/s at 80 MSps and 640 MB/s at
160 MSps. One gigabit Ethernet cannot carry any native full-rate mode. The
160 MSps payload also exceeds the current USB transport capability.

## Decoder pipeline

The implementation in [`birria_wifi.cpp`](birria_wifi.cpp) keeps the signal path self-contained:

1. The source supplies native USB, little-endian file or little-endian UDP CS16 samples.
2. A frequency translator selects the requested channel when a wide software sweep is active, then a rate converter reduces 40, 80 or 160 MSps to 20 MSps.
3. The packet detector searches for the legacy short training field with a lag-16 autocorrelation and a rolling energy estimate.
4. The synchronizer correlates against both 64-sample long training symbols and estimates coarse and fine carrier offset.
5. The channel estimator divides the received long training subcarriers by the known training sequence.
6. The SIGNAL decoder applies a 64-point FFT, pilot phase correction, BPSK soft decisions, deinterleaving and rate-one-half Viterbi decoding.
7. The DATA decoder follows the SIGNAL rate and length, applies pilot tracking, QAM soft decisions, deinterleaving, depuncturing and Viterbi decoding. The Viterbi traceback selects the best final state because scrambled padding can follow the zero tail bits.
8. The descrambler finds the seven-bit initial scrambler state from the zero SERVICE prefix.
9. The frame checker compares the received FCS with IEEE CRC32 before exposing MAC information.
10. The MAC parser labels frame type, subtype and addresses, then parses selected beacon and probe-response fields.

The 64-point FFT is implemented in the example. The convolutional decoder uses the standard constraint-length-seven code with octal generators `133` and `171`. The supported puncturing patterns cover every legacy OFDM rate.

The current rate converter uses a moving average with factors two, four or
eight. The wide 80 MSps sweep adds frequency translation before this conversion.
The converter is fast but is not a qualified polyphase channelizer. Adjacent
channel rejection and amplitude flatness remain experimental.

## Supported PHY modes

| PHY rate | Modulation | Coding rate |
| --- | --- | --- |
| 6 Mbps | BPSK | 1/2 |
| 9 Mbps | BPSK | 3/4 |
| 12 Mbps | QPSK | 1/2 |
| 18 Mbps | QPSK | 3/4 |
| 24 Mbps | 16-QAM | 1/2 |
| 36 Mbps | 16-QAM | 3/4 |
| 48 Mbps | 64-QAM | 2/3 |
| 54 Mbps | 64-QAM | 3/4 |

The decoder accepts PSDU lengths from 4 to 4,095 bytes. It rejects invalid SIGNAL parity, nonzero SIGNAL tail bits, unknown rate codes and lengths outside that range.

## Limitations

The decoder is an experimental visibility tool rather than a complete IEEE 802.11 receiver.

- The decoder handles legacy 20 MHz OFDM frames only.
- The decoder does not handle the 1, 2, 5.5 or 11 Mbps DSSS and CCK PHY modes. Many 2.4 GHz access points transmit beacons at one of these basic rates, so those beacons do not appear.
- The decoder does not decode HT, VHT, HE or EHT payloads. Their legacy compatibility fields can still produce rejected candidates.
- The fixed-center mode decodes one 20 MHz channel. Native sweep mode searches channel centers 1 through 13 over time.
- The decoder uses one receive stream and does not perform MIMO processing, beamforming recovery or antenna combining.
- The decoder performs pilot common-phase tracking but no full per-symbol channel tracking.
- The decoder prints metadata and addresses but does not decrypt protected data frames or interpret higher-layer payloads.
- The decoder has no UDP reorder buffer, loss concealment or source timestamp. Native sweep epochs provide local frequency labels but not a hardware timestamp.
- File and UDP input rely on the supplied frequency label. Native capture controls the receiver center directly.

A missing beacon is therefore not evidence that an access point is absent. The beacon may use DSSS or CCK, lie on another channel, arrive below the decoding threshold or fail because samples were lost.

## Performance and validation

The self-test covers the constraint-length-seven convolutional decoder, a synthetic complex channel with known amplitude and phase, carrier-offset correction, CSI normalization, binary CSI serialization, the standard CRC32 check value and basic control-frame address parsing. CTest runs it as `wifi_decoder_self_test`:

```sh
ctest --test-dir build -R wifi_decoder_self_test --output-on-failure
```

The decoder was exercised with a connected generic-ID MT7961 and MT7972
dongle on October 23, 2026. A half-second channel 1 recording produced seven
valid legacy OFDM frames, including beacons, probe responses and an RTS frame.
A half-second channel 6 recording produced two valid 12 Mbps action frames. A
live localhost UDP run decoded beacons, probe responses, acknowledgements and a
12 Mbps block acknowledgment without source gaps or local UDP drops. These
observations qualify live 6 Mbps and 12 Mbps decoding on that setup. They do not
qualify every supported rate or frame subtype.

The final status line separates short-training candidates, long-training
rejections, SIGNAL rejections, valid SIGNAL headers, DATA decoding rejections,
valid frames and FCS failures. These counters show which receiver stage rejected
samples when no frame reaches standard output.

The complete project test suite includes the decoder self-test:

```sh
ctest --test-dir build --output-on-failure
```

## Troubleshooting

### No frames appear

Confirm that `birria_wifi` starts before `birria_udp`, both use the same port and the sender targets the decoder's bind address. Confirm that the receiver center matches the desired 20 MHz Wi-Fi channel.

Remember that common 2.4 GHz beacon rates use DSSS or CCK, which this decoder does not support. Generate legacy OFDM traffic on the centered channel or inspect a busy network before concluding that the decoder is not receiving samples.

### SIGNAL headers appear but valid frames do not

Run with `--verbose` and inspect FCS mismatch messages. Frequent failures can come from UDP loss, low signal-to-noise ratio, an incorrect channel center, interference, an unsupported nonlegacy payload or limitations in the current equalizer and pilot tracking.

Use localhost first. Check `birria_udp` for dropped datagrams and avoid running another unicast listener on the same port.

### The reported frequency is wrong

Pass the same whole-hertz value to both programs:

```sh
./build/birria_wifi --port 1235 --frequency 2437000000
./build/birria_udp --port 1235 --frequency 2437000000 --manual
```

The first command labels decoded records. The second command tunes the hardware.

### Recorded input produces no output

Confirm that the file is raw little-endian CS16 at the sample rate passed through
`--sample-rate`, rather than WAV, CF32 or another container. Confirm that the file starts on a complete I,Q pair and contains the centered channel of interest.

### UDP reception stops or loses many frames

The data rate is high enough to stress socket buffers and schedulers. Keep both programs on the same host during initial testing. Reduce other load and use the sender's default packet size before experimenting with larger datagrams.
