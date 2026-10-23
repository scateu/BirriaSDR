# Wide-band coverage and receiver interoperability

This design note compares frequency sweeping with wider native capture. It also
records the first 80 MSps MT7961 and MT7972 raw I/Q experiment. The experiment
used the generic-ID USB board connected on October 23, 2026.

The library now exposes 40, 80 and 160 MSps. The 40 MSps mode is the default.
The 80 MSps mode is experimental but hardware-tested. The 160 MSps CS16 mode is
available for controlled research and is expected to overrun the current USB
exporter.

## Coverage goals

The phrase whole band needs a precise boundary. The 2.4 GHz ISM allocation from
2,400 to 2,483.5 MHz spans 83.5 MHz. Bluetooth LE primary advertising channels
span 78 MHz from channel 37 at 2,402 MHz to channel 39 at 2,480 MHz. Regional
Wi-Fi channel sets occupy different subsets of the same allocation.

A complex sample rate gives the same nominal frequency span. The usable span is
smaller when edge rolloff and alias rejection need margin.

| Capture mode | Nominal span | Practical consequence |
| --- | --- | --- |
| 40 MSps | 40 MHz | The full ISM allocation needs at least three overlapping tuning positions with the measured 40 MHz profile |
| 80 MSps | 80 MHz | One tuning can contain all Bluetooth LE channels and common regional Wi-Fi channel sets, but not every point of the 83.5 MHz allocation with guard margin |
| 160 MSps CS16 | 160 MHz | The current USB exporter cannot sustain the 640 MB/s sample payload |

## Current 40 MSps sweep

The measured 40 MHz profile has useful response near offsets of plus and minus
18 MHz, but it is not a brick-wall filter. A conservative full-band sweep can
use these centers:

| Sweep position | Center | Approximate span with an 18 MHz margin |
| --- | --- | --- |
| 1 | 2,418 MHz | 2,400 to 2,436 MHz |
| 2 | 2,450 MHz | 2,432 to 2,468 MHz |
| 3 | 2,482 MHz | 2,464 to 2,500 MHz |

The overlap absorbs filter rolloff and tuning uncertainty. A Bluetooth-only
sweep can use fewer positions because its channel plan is sparse. A center of
2,414 MHz covers primary advertising channels 37 and 38, and a center of
2,480 MHz covers channel 39.

Live tuning already works through `birriasdr_set_center_freq()`. The current raw
UDP stream cannot describe a sweep because its datagrams carry no sequence,
frequency, sample index or configuration epoch. A downstream decoder cannot
know whether queued samples precede a retune or belong to the new center.

## Control and application compatibility

The project should not add a private control socket and call it general SDR
compatibility. The existing receiver applications already support device APIs
that carry tuning, sample-rate and gain controls.

### Preferred interface: SoapySDR

A SoapySDR device module is the best primary integration target:

- The SDR++ Soapy source asks the device for its sample-rate list and sends live
  frequency changes through the device API.
- The Gqrx installation supports SoapySDR devices through its existing source
  stack.
- The module can preserve native CS16 samples instead of reducing them to eight
  bits.
- The module can expose 40 MSps and an experimental 80 MSps rate explicitly.
- The SoapyRemote project can provide a standard remote path when the network
  has enough capacity.

The first module should expose one receive channel, CS16 and CF32 stream formats,
the qualified tuning range, the supported sample rates, raw gain controls and
live tuning. The bandwidth control should report the fixed profile associated
with each sample rate rather than pretend that arbitrary widths are calibrated.

The direct local module avoids the 160 MB/s or 320 MB/s network hop. A remote
40 MSps stream already exceeds one gigabit Ethernet, and an 80 MSps CS16 stream
needs about 2.56 Gbit/s before transport overhead.

### Limited compatibility interface: RTL-TCP

The RTL-TCP protocol is useful for narrow compatibility, but it should not be
the wide-band foundation. Its reference data stream contains unsigned eight-bit
I and Q samples. Its control stream uses five-byte commands for frequency,
sample rate, gain mode and related RTL-specific controls.

The current SDR++ RTL-TCP source offers a fixed rate list from 250 kS/s to
3.2 MSps and converts unsigned eight-bit samples. It therefore cannot request
or represent the native 40 MSps and 80 MSps BirriaSDR modes through its normal
user interface.

An optional BirriaSDR RTL-TCP server could decimate on the host and provide a
compatible narrow view. It would be useful for existing applications, but it
would discard sample resolution and most of the captured bandwidth. Unsupported
RTL-specific commands should remain harmless and should not be mapped to
fictional hardware features.

### Raw UDP sweep interface

The raw UDP example remains useful for local scanners and experiments. A sweep
mode needs a second, framed transport rather than changing the legacy payload
in place. The legacy mode must stay byte-for-byte compatible with the SDR++
Network source.

A framed datagram should include at least these fields:

| Field | Purpose |
| --- | --- |
| Magic, version and header length | Allows independent evolution and rejects accidental raw data |
| Stream and configuration epoch | Separates reconnects, retunes and recovery restarts |
| Datagram sequence | Detects UDP loss and reordering |
| First delivered sample | Orders data within one acquisition epoch |
| Center frequency and sample rate | Labels every payload without relying on control timing |
| Sample format and count | Supports CS16 now and an explicit future format |
| Flags | Marks data, retune transition, source gap and terminal events |

The control connection should return a result for each command and publish the
new configuration epoch. A control thread should call the existing live tuning
API while the sample callback continues draining USB. The sender should mark a
conservative transition interval because the library does not identify the
exact RF settling sample and host queues can contain older data.

A later library operation could stop ICAP, drain queued source data, retune and
restart ICAP without a complete device reboot. That operation would create a
clean sweep boundary and a precise gap event. It is preferable for measurement
sweeps, but it needs a new lifecycle contract and firmware tests.

## Experimental 80 MSps result

The temporary 80 MSps build changed the matched fields together:

| Mechanism | The 40 MSps value | Experimental 80 MSps value |
| --- | --- | --- |
| Capture divider at `0x830ad448[19:18]` | Code 3, word `0x008c0400` | Code 2, word `0x00880400` |
| Analog filter at RF `0x160[31:30]` | Code 1 | Code 2 |
| Digital bandwidth class at `0x83081000[27:24]` and `0x83091000[27:24]` | Code 1 | Code 2 |
| Complex sample rate | 40 MSps | 80 MSps |

The experiment retained the dense single-chain CS16 format, the 2,048-sample
firmware payload and the existing original device program.

The following observations passed on the connected board:

- A finite 40,000,000-sample capture completed without a stream error.
- A discard-only run delivered 800,000,000 complex samples in 10.95 seconds of
  wall time, including setup and cleanup. The stream reported no overrun.
- An 80 to 20 MSps software conversion decoded valid 6 Mbps and 12 Mbps legacy
  Wi-Fi frames with correct FCS values.
- A capture centered at 2,441 MHz produced valid BLE packets on all three primary
  advertising channels at the same time. The half-second file contained 23
  valid channel 37 packets, 41 valid channel 38 packets and 15 valid channel 39
  packets.
- The edge channels were centered at offsets of minus 39 MHz and plus 39 MHz.
  Their successful CRC checks establish useful reception there for those packets.

These results establish a working 80 MSps path on one board. They do not yet
establish calibrated amplitude flatness, stopband rejection, performance on
other hosts or a guaranteed plus and minus 39 MHz passband.

## Experimental 160 MSps result

A second temporary build used capture-divider code 1 and analog and digital
filter code 3. It attempted the same dense CS16 format at 160 MSps.

The run failed after 63,488 complex samples. The firmware reported state 9,
which is a source overrun, and the host returned a continuity error. The reset
completed successfully.

The result is consistent with the transport limit. A 160 MSps CS16 payload is
640 MB/s before event and USB overhead. SuperSpeed USB at 5 Gbit/s cannot carry
that payload after line coding and protocol overhead. The failure does not prove
that the internal ADC or ICAP source cannot run at that rate. It proves that the
current dense CS16 USB exporter cannot sustain it.

A future 160 MSps experiment would need a lower-bit sample format, on-device
decimation or another transport. An eight-bit complex format would reduce the
payload to 320 MB/s, but an MCU repacker could become the next bottleneck and
would discard six source bits per component.

## Implementation status and next steps

The native protocol decoders now open USB capture by default. Both programs
retain mutually exclusive UDP and file modes. Their native sweep modes use live
RF tuning at 40 MSps and software channel selection within one 80 MSps capture.
The recorder and UDP sender also accept the three native rates.

The completed work includes these items:

- The library applies rate-specific capture dividers and matched filter profiles.
- The 80 MSps and 160 MSps qualification levels appear in help and startup output.
- The BLE scanner sweeps channels 37, 38 and 39.
- The Wi-Fi decoder sweeps channel centers 1 through 13.
- Decoder workers drain native samples without blocking the USB callback.
- The 160 MSps mode disables recovery and stops after the first expected overrun.

The remaining interoperability work follows this order:

1. Add an optional SoapySDR module for direct SDR++ and Gqrx integration.
2. Add a framed UDP mode when remote sweep metadata and loss detection are
   required, while retaining the current raw UDP mode.
3. Add an optional RTL-TCP server with server-side decimation for narrow legacy
   application compatibility.
4. Investigate a clean stop, retune and restart operation for precise sweep
   boundaries.

The 80 MSps mode is the highest sustainable native CS16 rate observed with the
current device program and USB transport. Strict continuous coverage of every
frequency from 2,400 to 2,483.5 MHz still needs two overlapping 80 MHz tunings
or two receivers because the allocation is wider than the sampled span.
