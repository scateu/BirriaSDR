# Decode Wi-Fi frames from BirriaSDR I/Q

The `birria_wifi` example is a standalone receiver for legacy 20 MHz IEEE 802.11 OFDM frames inside the 40 MSps BirriaSDR stream. It accepts raw CS16 over UDP or from a recorder file, validates each decoded frame with IEEE CRC32 and prints MAC information to standard output.

The decoder uses only the C++ standard library and operating system socket APIs. It does not link to `libbirriasdr` or an external DSP library, so it can run on a different computer from the USB receiver.

> [!IMPORTANT]
>
> Receive wireless traffic only where local law and your authorization permit it. Management and control frames can expose network names and device addresses even when data payloads are encrypted.

## Build the decoder

Build from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j --target birria_wifi birria_udp
```

The build creates `build/birria_wifi`. The decoder does not open the dongle, so it does not need libusb permissions at runtime. The project still needs libusb during configuration because the same build also defines `libbirriasdr` and the hardware examples.

Run the built-in coding and MAC parser test before using live samples:

```sh
./build/birria_wifi --self-test
```

A successful test prints:

```text
Wi-Fi decoder self-test passed
```

## Decode a live UDP stream

Start the decoder before the UDP sender. This lets the receiving socket bind before the first I/Q datagram arrives.

Use port `1235` when SDR++ already uses the default UDP port `1234`:

```sh
./build/birria_wifi \
    --bind 127.0.0.1 \
    --port 1235 \
    --frequency 2437000000
```

Start the sender in another terminal with the same destination port and center frequency:

```sh
./build/birria_udp \
    --host 127.0.0.1 \
    --port 1235 \
    --frequency 2437000000 \
    --manual
```

The `--frequency` option on `birria_wifi` labels output only. It does not tune hardware or send a command to `birria_udp`. The hardware center comes from the `--frequency` option passed to `birria_udp`.

The desired 20 MHz Wi-Fi channel must be centered in the 40 MSps capture. For example, use 2,437,000,000 Hz for Wi-Fi channel 6. The decoder does not search for channels at other offsets inside the sampled span.

Stop the decoder with Ctrl-C. It prints final counters to standard error before exiting.

## Decode a recorder file

The decoder can read a little-endian CS16 file written by `birria_rx`:

```sh
./build/birria_wifi \
    --file capture.cs16 \
    --frequency 2437000000
```

The file must contain interleaved signed 16-bit I and Q components at 40 MSps complex. The `--frequency` value remains an output label and does not alter the samples.

The file path replaces UDP input. The bind address and port have no effect when `--file` is present.

## Command line options

Run the help command for the current option summary:

```sh
./build/birria_wifi --help
```

| Option | Default | Purpose and limits |
| --- | --- | --- |
| `--bind ADDRESS` | `0.0.0.0` | Selects the local address for the UDP socket |
| `--port PORT` | `1234` | Selects the local UDP port from 1 to 65,535 |
| `--file PATH` | Disabled | Reads a continuous little-endian CS16 file instead of UDP |
| `--frequency HZ` | Not shown | Adds a center-frequency label to decoded frame output |
| `--verbose` | Disabled | Prints rejected PSDU diagnostics, including FCS mismatches, to standard error |
| `--self-test` | Disabled | Tests convolutional decoding, CRC32 and basic MAC parsing without input samples |
| `--help` | Not applicable | Prints the option summary and exits |

The decoder writes valid frame records to standard output. Startup information, rejection diagnostics and final counters go to standard error. This separation lets another program consume valid records without parsing status messages.

## Output

A valid decoded frame produces one summary line. This example came from an existing 40 MSps capture:

```text
frame time=0.007555s sample=302184 center=2437000000Hz rate=6Mbps length=14 type=control subtype=cts dst=e0:3e:44:08:34:da cfo=-13670.9Hz fcs=ok
```

| Field | Meaning |
| --- | --- |
| `time` | Position of the detected long training field, derived from the 40 MSps input sample count |
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
| `fcs` | The value `ok`, because frames with a failed FCS are not printed to standard output |

The decoder maps data-frame addresses according to the To DS and From DS bits. Four-address data frames use address 3 as the destination and address 4 as the source. Control frames expose only the addresses present in their shorter subtype-specific headers.

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

## Input and transport contract

The UDP input matches the raw output from `birria_udp`:

| Property | Required value |
| --- | --- |
| Sample rate | 40 MSps complex |
| Sample type | Interleaved signed 16-bit I, Q |
| Byte order | Little-endian |
| Source range | Normally −8192 to 8191 for each BirriaSDR component |
| Datagram metadata | None |
| Datagram alignment | A whole number of four-byte complex samples |
| Expected channel | One 20 MHz Wi-Fi channel centered at zero frequency offset |

Datagram boundaries have no PHY meaning. The decoder keeps sample-pair state across input blocks and continues its preamble search across datagrams.

The raw UDP protocol has no sequence number, timestamp or acknowledgement. The decoder cannot identify the exact location or size of a lost datagram. A loss commonly causes a SIGNAL rejection or FCS failure, after which the decoder searches for the next preamble.

Do not bind SDR++ and `birria_wifi` to the same unicast UDP address and port. Socket reuse behavior can deliver each datagram to only one process rather than duplicating it. Use separate `birria_udp` runs, a multicast or fanout tool, or different recorded inputs when both consumers need the same samples.

The 40 MSps CS16 payload is 160 MB/s before UDP and IP overhead. A one gigabit network cannot carry the complete stream. Localhost is the safest initial setup, and remote use requires a faster link, adequate socket buffers and an MTU plan for the sender's datagram size.

## Decoder pipeline

The implementation in [`birria_wifi.cpp`](birria_wifi.cpp) keeps the signal path self-contained:

1. The input reader converts little-endian CS16 into complex floating-point samples.
2. The rate converter averages each adjacent pair of 40 MSps samples to form a 20 MSps waveform.
3. The packet detector searches for the legacy short training field with a lag-16 autocorrelation and a rolling energy estimate.
4. The synchronizer correlates against both 64-sample long training symbols and estimates coarse and fine carrier offset.
5. The channel estimator divides the received long training subcarriers by the known training sequence.
6. The SIGNAL decoder applies a 64-point FFT, pilot phase correction, BPSK soft decisions, deinterleaving and rate-one-half Viterbi decoding.
7. The DATA decoder follows the SIGNAL rate and length, applies pilot tracking, QAM soft decisions, deinterleaving, depuncturing and Viterbi decoding.
8. The descrambler finds the seven-bit initial scrambler state from the zero SERVICE prefix.
9. The frame checker compares the received FCS with IEEE CRC32 before exposing MAC information.
10. The MAC parser labels frame type, subtype and addresses, then parses selected beacon and probe-response fields.

The 64-point FFT is implemented in the example. The convolutional decoder uses the standard constraint-length-seven code with octal generators `133` and `171`. The supported puncturing patterns cover every legacy OFDM rate.

The adjacent-pair average is a small and fast conversion for a centered 20 MHz channel. It is not a general channelizer, and it does not move an offset channel to baseband. A future decoder that searches the full 40 MHz span would need frequency translation and a qualified decimation filter for each candidate channel.

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
- The decoder does not search other 20 MHz channel centers within the 40 MHz capture.
- The decoder uses one receive stream and does not perform MIMO processing, beamforming recovery or antenna combining.
- The decoder performs pilot common-phase tracking but no full per-symbol channel tracking.
- The decoder prints metadata and addresses but does not decrypt protected data frames or interpret higher-layer payloads.
- The decoder has no UDP reorder buffer, loss concealment or source timestamp.
- The decoder does not identify the RF channel from the samples. The `--frequency` argument is only a label.

A missing beacon is therefore not evidence that an access point is absent. The beacon may use DSSS or CCK, lie on another channel, arrive below the decoding threshold or fail because samples were lost.

## Performance and validation

The self-test covers the constraint-length-seven convolutional decoder, the standard CRC32 check value and basic control-frame address parsing. CTest runs it as `wifi_decoder_self_test`:

```sh
ctest --test-dir build -R wifi_decoder_self_test --output-on-failure
```

The decoder was also exercised against a 64 MB recording containing 16,000,000 complex samples. On the development Apple Silicon Mac, it processed the 0.4-second recording in about 0.4 seconds and printed a valid 6 Mbps CTS frame with a correct FCS. A localhost UDP replay of the same prefix decoded the same frame. These observations establish that the current implementation can decode at least that captured legacy control frame, not that every supported rate or frame subtype has been qualified over the air.

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

Confirm that the file is raw little-endian CS16 at 40 MSps complex rather than WAV, CF32 or another container. Confirm that the file starts on a complete I,Q pair and contains the centered channel of interest.

### UDP reception stops or loses many frames

The data rate is high enough to stress socket buffers and schedulers. Keep both programs on the same host during initial testing. Reduce other load and use the sender's default packet size before experimenting with larger datagrams.
