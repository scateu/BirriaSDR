# Sweep Bluetooth LE advertising channels

The `birria_ble --sweep` mode monitors Bluetooth Low Energy primary advertising
channels 37, 38 and 39. Native USB capture is required because raw UDP has no
return control or frequency metadata.

## Advertising channels

| Channel | Frequency |
| --- | --- |
| 37 | 2,402 MHz |
| 38 | 2,426 MHz |
| 39 | 2,480 MHz |

Monitoring all three channels reduces discovery latency and the chance of
missing an advertising event. Monitoring one channel can still find repeating
advertisers, but it provides weaker coverage.

## Sweep at 40 MSps

The default mode retunes the receiver among the three advertising channel
centers:

```sh
./build/birria_ble \
    --sweep \
    --dwell-ms 1000 \
    --manual
```

The capture and decoder run on separate threads. Each successful live retune
starts a new configuration epoch. The scanner discards a settling interval
before decoding the new center.

A shorter dwell revisits every channel sooner, but leaves less useful time after
tuning and settling. A longer dwell improves the chance of observing a slowly
repeating advertiser on each channel.

## Software sweep at 80 MSps

The experimental 80 MSps mode keeps the receiver centered at 2,441 MHz. All
three advertising channels lie inside that sampled span. The scanner switches
its software channelizer among them without RF retuning:

```sh
./build/birria_ble \
    --sample-rate 80000000 \
    --sweep \
    --dwell-ms 300 \
    --manual
```

The connected generic-ID board decoded valid channel 37, 38 and 39 packets from
one 80 MSps capture on October 23, 2026. The mode remains experimental because
it requires about 320 MB/s of USB payload and its passband is not calibrated.

## The 160 MSps research mode

The 160 MSps mode prints a warning and disables automatic recovery:

```sh
./build/birria_ble --sample-rate 160000000 --sweep --manual
```

The current CS16 exporter is expected to report a source overrun. The program
stops after the first failure instead of repeatedly restarting. This mode is a
controlled transport experiment, not a reliable scanner mode.

## Continuous alternatives

One 80 MSps receiver can software-sweep all primary advertising channels
without retuning. A strict simultaneous decoder would need enough host CPU to
run three channelizers at once. The current sweep deliberately processes one
advertising channel per dwell to keep up reliably.

Multiple 40 MSps receivers can avoid dwell gaps. Assign one receiver to each
primary channel, or use one receiver for channels 37 and 38 experiments and
another for channel 39.

## Room coverage

Frequency coverage and physical room coverage are separate. Sweeping covers the
three advertising frequencies. Walls, people, furniture, interference and
antenna placement still determine whether packets from every part of a room
reach the receiver.

The [complete Bluetooth LE scanner guide](README_ble.md) describes native, UDP
and file input, packet fields and decoder limitations.
