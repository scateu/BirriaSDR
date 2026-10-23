# Scan Bluetooth LE advertisements with BirriaSDR

The `birria_ble` example decodes Bluetooth Low Energy advertisements and Apple
Find My candidates. Native USB capture is the default. The scanner can also read
a recorder file or receive little-endian CS16 over UDP.

The scanner recognizes Apple manufacturer data. It marks the Apple Find My
pattern used by AirTag and compatible accessories as an AirTag candidate. This
pattern does not prove that the transmitter is an AirTag.

> [!IMPORTANT]
>
> Receive wireless traffic only where local law and your authorization permit
> it. Bluetooth advertisements can expose device addresses, names and tracking
> identifiers.

## Build and test the scanner

Build from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j --target birria_ble birria_udp birria_rx
./build/birria_ble --self-test
```

The self-test generates an offline LE 1M advertisement. It checks channel
filtering, GFSK demodulation, access address detection, whitening, CRC handling
and Apple Find My parsing. The test accepts each implemented input rate:

```sh
./build/birria_ble --self-test --sample-rate 40000000
./build/birria_ble --self-test --sample-rate 80000000
./build/birria_ble --self-test --sample-rate 160000000
```

## Capture directly from USB

The default command opens native device 0 at 40 MSps and monitors advertising
channel 38 at 2,426 MHz:

```sh
./build/birria_ble
```

The manual option applies LNA 2, filter stage 8 and digital 0:

```sh
./build/birria_ble --manual
```

Select another device, center or advertising channel when needed:

```sh
./build/birria_ble \
    --device 1 \
    --frequency 2402000000 \
    --channel 37 \
    --manual
```

The scanner links to `libbirriasdr`, opens the USB receiver, tunes it and drains
samples on a dedicated capture path. Signal processing runs on a separate worker
so decoder work does not block USB delivery. A decoder queue overflow is
reported as an error rather than hidden.

## Sweep the whole advertising band

The sweep option monitors primary advertising channels 37, 38 and 39:

```sh
./build/birria_ble --sweep --manual
```

The default 40 MSps mode retunes among the three channel centers. The scanner
discards a settling interval after each live retune and labels records with the
absolute input sample index and channel frequency.

The experimental 80 MSps mode keeps one 80 MHz capture centered at 2,441 MHz.
It software-sweeps the three advertising channels without RF retuning:

```sh
./build/birria_ble \
    --sample-rate 80000000 \
    --sweep \
    --dwell-ms 300 \
    --manual
```

A connected generic-ID MT7961 and MT7972 board decoded valid packets on all
three primary advertising channels from this wide capture on October 23, 2026.
The 80 MSps mode requires about 320 MB/s of CS16 payload and remains
experimental.

The 160 MSps mode is available only for controlled research:

```sh
./build/birria_ble --sample-rate 160000000 --sweep --manual
```

The program prints a prominent warning and disables automatic recovery. The
current dense CS16 USB exporter is expected to overrun, so the first overrun
stops the run with a nonzero status.

The [advertising channel sweep guide](README_ble_sweep.md) explains dwell time,
coverage and one-receiver versus multi-receiver operation.

## Receive UDP optionally

The UDP mode preserves the earlier standalone workflow. Start the scanner first:

```sh
./build/birria_ble \
    --udp \
    --bind 127.0.0.1 \
    --port 1235 \
    --frequency 2426000000 \
    --channel 38
```

Start the sender with the same sample rate, destination and center:

```sh
./build/birria_udp \
    --host 127.0.0.1 \
    --port 1235 \
    --frequency 2426000000 \
    --sample-rate 40000000 \
    --manual
```

The UDP mode cannot sweep because the raw datagrams contain no control channel,
frequency metadata, sequence number or configuration epoch. Native capture is
required for the built-in sweep.

## Read a recorder file

The scanner can read a file written by `birria_rx`:

```sh
./build/birria_rx \
    --frequency 2426000000 \
    --sample-rate 40000000 \
    ble-channel-38.cs16 \
    40000000 \
    manual

./build/birria_ble \
    --file ble-channel-38.cs16 \
    --frequency 2426000000 \
    --sample-rate 40000000 \
    --channel 38
```

The file must contain continuous little-endian signed 16-bit I and Q pairs. The
sample rate and center supplied to the scanner must match the recording.

## Read packet output

The scanner writes each valid packet as one line on standard output. Startup
information, warnings, sweep events, rejected candidates and final counters go
to standard error.

A record includes radio timing and quality, the PDU header, applicable
addresses, decoded advertising data and complete raw bytes. Apple records can
include company information, iBeacon fields and the Find My candidate pattern.

The `rssi_dbfs` value is uncalibrated. It describes channelized sample power
relative to the signed 16-bit numeric range, not received power in dBm. The
`cfo_hz` value is estimated from the two GFSK frequency clusters.

## Command line options

Run the help command for the current option summary:

```sh
./build/birria_ble --help
```

| Option | Default | Purpose and limits |
| --- | --- | --- |
| `--udp` | Disabled | Selects UDP instead of native USB capture |
| `--bind ADDRESS` | `0.0.0.0` | Selects the local UDP address |
| `--port PORT` | `1234` | Selects the local UDP port |
| `--file PATH` | Disabled | Reads a CS16 file instead of USB or UDP |
| `--frequency HZ` | `2426000000` | Selects native tuning or labels file and UDP input |
| `--channel 37\|38\|39` | Automatic | Selects an advertising channel and can be repeated |
| `--device INDEX` | `0` | Selects the native USB device |
| `--sample-rate RATE` | `40000000` | Selects 40, 80 or 160 MSps |
| `--manual` | Disabled | Selects LNA 2, filter stage 8 and digital 0 for native capture |
| `--recovery COUNT` | `3` | Selects the native full restart budget |
| `--sweep` | Disabled | Monitors all three primary advertising channels in native mode |
| `--dwell-ms MS` | `1000` | Selects the sweep dwell per advertising channel |
| `--include-invalid` | Disabled | Prints access address hits whose decoded CRC is invalid |
| `--verbose` | Disabled | Prints rejected candidate diagnostics |
| `--self-test` | Disabled | Runs the offline PHY and parser test |
| `--help` | Not applicable | Prints the option summary and exits |

The USB, UDP and file sources are mutually exclusive. Native-only options are
rejected with UDP and file input. The sweep option is native-only and cannot be
combined with an explicit frequency or channel list.

## Supported packets and limits

The scanner decodes the LE 1M PHY on primary advertising channels 37, 38 and
39. It decodes legacy advertising PDUs, scan requests, scan responses,
connection requests and the primary `ADV_EXT_IND` header.

The scanner does not follow an auxiliary pointer to a secondary channel. It
does not decode LE 2M, LE Coded, periodic advertising payloads or connection
data after a connection request. It does not actively transmit scan requests.

The Apple Find My parser is a passive format heuristic. Find My accessories use
rotating addresses and key material. The output must not be treated as a stable
device identity.

## Signal processing

For each active advertising channel, the scanner performs these steps:

1. The channelizer mixes the selected channel to baseband.
2. A moving average and a 25-tap low-pass filter reduce the rate to 4 MSps.
3. A phase discriminator produces four samples per LE 1M symbol.
4. Four timing phases search for the advertising access address.
5. The strongest timing phase demodulates the header and payload.
6. The decoder removes channel whitening and validates the 24-bit CRC.
7. The parser prints addresses, advertising data and raw bytes.

## Troubleshooting

### Native capture cannot open the dongle

Confirm that no kernel Wi-Fi driver owns the interface and that no vendor patch
is retained. Check USB permissions and run `./build/birria_rx --list`.

### The scanner prints no packets

Try the default channel 38 center first. Use `--sweep` when the target might
advertise only while the receiver is on another primary channel.

### The decoder queue reports dropped samples

Reduce output processing, use 40 MSps or increase the dwell time. A queue drop
means the host decoder did not consume every delivered sample, and the program
returns a nonzero status.

### The Apple fields do not say AirTag

The scanner marks only the Apple Find My manufacturer pattern. Other Apple
formats are decoded without an AirTag label, and compatible accessories can use
the same Find My pattern.
