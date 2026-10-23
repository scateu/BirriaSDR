# Why CSI is available from BirriaSDR I/Q

The `birria_wifi` decoder can calculate channel state information because BirriaSDR preserves the complex baseband waveform, and every legacy OFDM Wi-Fi packet begins with a standardized reference signal. The decoder compares the received reference with the signal that a compliant transmitter was required to send.

The resulting values are software-derived channel estimates. They are useful for relative channel measurements and sensing, but they include transmitter, receiver and synchronization effects in addition to radio propagation.

This page explains the signal model, packet processing, CSI confidence and practical limits. The [CSI export guide](README_wifi_csi.md) describes commands, record formats and parsing.

## Channel estimation model

For one OFDM subcarrier `k`, the receiver observes approximately:

```text
Y[k] = H[k] X[k] + N[k]
```

The terms have these meanings:

| Term | Meaning |
| --- | --- |
| `X[k]` | Complex symbol transmitted on subcarrier `k` |
| `H[k]` | Complex channel response on subcarrier `k` |
| `Y[k]` | Complex symbol measured by the receiver |
| `N[k]` | Noise, interference and model error |

When the transmitted symbol is known and nonzero, the receiver can estimate the channel:

```text
H[k] ≈ Y[k] / X[k]
```

Each estimate is complex:

```text
H[k] = real[k] + j imaginary[k]
```

Its magnitude describes the received amplitude response:

```text
magnitude[k] = sqrt(real[k]^2 + imaginary[k]^2)
```

Its phase describes the measured phase response:

```text
phase[k] = atan2(imaginary[k], real[k])
```

A CSI record is a collection of these complex estimates across the active OFDM subcarriers.

## The Wi-Fi packet supplies a known reference

A legacy OFDM packet begins with this preamble and header sequence:

```text
Short Training Field
Long Training Field
SIGNAL
DATA
```

The long training field, called the L-LTF, carries a predefined BPSK pattern on the active OFDM subcarriers. A compliant legacy OFDM transmitter uses this known pattern before sending its unknown payload.

The receiver therefore already knows `X[k]` for the training field. It can measure `Y[k]` from the captured waveform and calculate `H[k]`.

The L-LTF contains two copies of a 64-sample training symbol. The decoder uses both copies to:

- Refine carrier-frequency offset.
- Verify packet timing.
- Average two channel observations.
- Reduce estimation noise.

## BirriaSDR preserves the required information

BirriaSDR exports complex baseband samples in this order:

```text
I[0], Q[0], I[1], Q[1], ...
```

These samples preserve amplitude and phase. Both are required for a complex channel estimate.

A power spectrum alone would not be sufficient because it discards most phase information. Raw complex I/Q retains the complete received OFDM preamble, so CSI can be estimated in host software without enabling a separate chipset CSI-reporting mechanism.

## Converting native samples to the Wi-Fi sample grid

Legacy 20 MHz OFDM uses a 20 MSps sample grid. The decoder accepts native input
at 40, 80 or 160 MSps and averages groups of two, four or eight samples:

```text
y[n] = sum(x[D n + i], i = 0 .. D - 1) / D
```

The decimation factor `D` is the input rate divided by 20 MSps. A wide 80 MSps
sweep first translates the selected 20 MHz channel to baseband.

At 20 MSps:

- One sample represents 50 ns.
- The useful OFDM symbol contains 64 samples.
- The useful symbol duration is 3.2 microseconds.
- The subcarrier spacing is 312.5 kHz.

The moving average is fast, but its frequency response becomes part of every
measured CSI vector. It is not a calibrated channelizing filter. A future
qualified polyphase decimator could provide a better controlled response.

Implementation: the `feed_sample()` method in [`birria_wifi.cpp`](birria_wifi.cpp).

## Finding the packet

The Short Training Field repeats every 16 samples at 20 MSps. The decoder searches for this repetition with a delayed autocorrelation:

```text
P[n] = sum(conj(r[n + i]) r[n + i + 16])
```

A sustained high normalized correlation indicates a likely legacy OFDM preamble. This identifies a candidate region but does not yet prove that the packet is valid.

The decoder then searches for two adjacent 64-sample symbols that correlate with the known time-domain L-LTF. Both correlations must exceed the configured threshold.

This second step supplies:

- More precise packet timing.
- Evidence that the candidate contains a real legacy OFDM preamble.
- The exact training samples used for CSI estimation.

The smaller of the two normalized L-LTF correlations is exported as `ltf_correlation`.

Implementation: the `ltf_correlation()` and `decode_candidate()` methods in [`birria_wifi.cpp`](birria_wifi.cpp).

## Correcting carrier-frequency offset

The transmitter and receiver oscillators are not perfectly equal. Their difference rotates the received complex waveform:

```text
r[n] = s[n] exp(j omega n)
```

Without correction, this rotation would contaminate the channel phase and make payload decoding unreliable.

### Coarse carrier estimate

The repeated 16-sample Short Training Field provides an initial estimate:

```text
omega_coarse = angle(P) / 16
```

### Fine carrier estimate

The two L-LTF symbols should be identical. Their measured phase difference provides a finer estimate:

```text
omega_fine = angle(sum(conj(LTF1[n]) LTF2[n])) / 64
```

The decoder combines the estimates and rotates the samples in the opposite direction:

```text
corrected[n] = received[n] exp(-j omega n)
```

This removes the principal packet-wide carrier rotation. Residual oscillator, sampling and synchronization errors can remain.

Implementation: the carrier calculations and `corrected()` method in [`birria_wifi.cpp`](birria_wifi.cpp).

## Transforming the training field into subcarriers

The decoder applies a 64-point FFT to each corrected L-LTF symbol:

```text
LTF time samples → 64-point FFT → received subcarriers
```

The two FFT outputs provide separate observations, `Y1[k]` and `Y2[k]`, of the same known training value.

The decoder calculates the channel estimate as:

```text
H[k] = (Y1[k] + Y2[k]) / (2 Xltf[k])
```

The average of the two received symbols reduces uncorrelated noise. The known L-LTF values are BPSK values of positive or negative one on the active bins, although the implementation uses the general complex expression.

Implementation: the `symbol_fft()` and `estimate_channel()` functions in [`birria_wifi.cpp`](birria_wifi.cpp).

## Why the export contains 52 values

A 64-point legacy OFDM symbol does not use every FFT bin. The active subcarriers are:

```text
-26 through -1
+1 through +26
```

This produces 52 complex estimates. The exporter omits:

- The DC subcarrier at index 0.
- The lower guard subcarriers.
- The upper guard subcarriers.

The fixed exported order is:

```text
-26, -25, ... -2, -1, +1, +2, ... +25, +26
```

The `active_csi()` function selects these values from the 64-bin estimate. Version 1 binary records use this fixed order, while JSON Lines records also include the explicit subcarrier array.

Implementation: the `csi_bins` array and `active_csi()` function in [`birria_wifi.cpp`](birria_wifi.cpp).

## The decoder already needs the channel estimate

The CSI calculation is not an unrelated measurement added after packet decoding. The decoder needs the same `H[k]` values to recover SIGNAL and DATA symbols.

For each received subcarrier, equalization is approximately:

```text
equalized[k] = received[k] / H[k]
```

Without equalization, multipath and receiver response would rotate and scale each BPSK, QPSK or QAM symbol differently.

The processing path is therefore:

```text
L-LTF
  ↓
Estimate H[k]
  ├── Export H[k] as CSI
  └── Use H[k] to equalize SIGNAL and DATA
```

Successful packet decoding is independent evidence that the channel estimate was useful for that packet.

## Why the FCS increases confidence

The default CSI exporter writes a record only after several independent checks pass:

1. The repeating Short Training Field is detected.
2. Two matching Long Training Field symbols are detected.
3. Carrier-frequency offset is estimated and corrected.
4. The legacy SIGNAL rate code is recognized.
5. The SIGNAL parity bit passes.
6. The six SIGNAL tail bits are zero.
7. The PSDU length is valid.
8. Convolutional decoding produces a payload bitstream.
9. The scrambler is reversed using the required zero SERVICE prefix.
10. The calculated IEEE CRC32 equals the received four-byte FCS.

The FCS does not directly prove that every CSI subcarrier is exact. It strongly confirms that the packet timing, carrier correction and equalization were coherent enough to recover the complete associated MAC frame.

The default export policy is:

```text
FCS valid → export CSI
FCS failed → export only with --csi-invalid
```

Records exported through `--csi-invalid` have `fcs_valid=false` and `mac_valid=false`. Their decoded MAC addresses are not exposed as trustworthy metadata.

## Confidence levels

The decoder distinguishes three practical confidence levels:

| Level | Evidence | Default CSI export |
| --- | --- | --- |
| Training valid | The short and long training detectors accepted the candidate | No |
| SIGNAL valid | The rate, parity, tail and PSDU length passed | Only with `--csi-invalid` when the FCS fails |
| Frame valid | The SIGNAL field passed and the MAC FCS matched | Yes |

A 32-bit FCS makes an accidental match unlikely for an ordinary corrupted candidate. The preamble, SIGNAL, coding and scrambler checks reduce false acceptance further. The FCS is not cryptographic authentication and does not prove transmitter identity.

## What the exported CSI measures

The estimate is not purely the propagation channel:

```text
Hmeasured[k] = Htransmitter[k] Hpropagation[k] Hreceiver[k]
```

It also includes effects from:

- Transmitter filters and analog circuitry.
- Transmitter packet timing.
- Environmental multipath.
- Receiver antenna and RF circuitry.
- BirriaSDR analog and digital filtering.
- Receiver gain settings.
- The 40 to 20 MSps adjacent-pair average.
- Residual carrier-frequency offset.
- Packet-detection timing error.
- Sampling-clock error.

The output is best described as a software-derived receiver channel estimate.

It is useful for:

- Comparing packets from the same transmitter.
- Observing motion-induced changes.
- Comparing relative subcarrier magnitudes.
- Relative sensing experiments.
- Plotting CSI magnitude or phase over time.

It is not yet a calibrated source for:

- Absolute path-loss measurement.
- Absolute phase comparison between unrelated captures.
- Calibrated time of flight.
- Distance measurement.
- Angle of arrival.
- A MIMO channel matrix.

## Why phase changes between packets

Raw CSI phase can change in a stationary environment. A useful phase model is:

```text
measured_phase[k] =
    propagation_phase[k]
  + common_phase
  + linear_slope k
  + hardware_phase[k]
  + noise[k]
```

The common phase can come from oscillator and synchronization error. The linear slope can come from packet timing and sampling offset.

The exporter preserves raw phase. It does not silently remove common phase or fit a linear slope because some experiments need those terms.

Post-processing can optionally:

- Remove one common phase from all subcarriers.
- Fit and remove a linear phase slope.
- Divide by a complex reference capture.
- Average repeated packets from one MAC address.
- Reject low-magnitude subcarriers.

The optional normalized vector removes only one packet-wide amplitude scale. It does not correct phase, flatten the receiver response or remove the center-frequency depression.

## Why the result is SISO CSI

The current BirriaSDR stream contains one receive chain. The CSI dimensions are therefore:

```text
1 receive chain × 1 legacy spatial stream × 52 subcarriers
```

A multi-antenna system would produce a matrix such as:

```text
receive chains × transmit streams × subcarriers
```

Multiple receive-chain CSI requires simultaneous synchronized I/Q capture from additional hardware chains. Separate captures cannot normally be combined because their oscillator phase and packet timing are not coherent.

## Effects near the center subcarrier

The DC subcarrier itself is unused and is not exported. However, the center-frequency depression observed in the receiver is wider than one OFDM bin. It can therefore affect nearby active subcarriers, especially indices `-1` and `+1`.

The exporter preserves those measurements rather than inventing replacements. Experiments should retain raw CSI, inspect the magnitude near DC and consider a stable complex reference captured with the same frequency, gain and processing version.

## Validation in the implementation

The built-in self-test verifies the CSI path independently of a live radio:

1. It constructs a known complex channel with varying amplitude and phase.
2. It multiplies the known L-LTF subcarriers by that channel.
3. It transforms the result into time-domain training symbols.
4. It applies a known carrier-frequency offset.
5. It corrects the offset and performs the same FFT-based estimate.
6. It compares the recovered channel with the original channel.
7. It checks that normalized CSI has unit root mean square magnitude.
8. It checks the version 1 binary record magic, sizes, flags and subcarrier count.

Run the test with:

```sh
./build/birria_wifi --self-test
```

The existing 64 MB capture also produced an FCS-valid 52-subcarrier CSI record associated with its decoded 6 Mbps CTS frame. A localhost UDP replay produced the same valid record. These checks validate the implemented path and serialization, but they do not establish absolute channel calibration or qualify every PHY rate over the air.

## Summary

The CSI estimate is available because:

1. BirriaSDR preserves complex amplitude and phase in raw I/Q.
2. Legacy OFDM packets contain a standardized known training sequence.
3. The decoder identifies and carrier-corrects the two L-LTF symbols.
4. Their FFT values are compared with the known transmitted values.
5. The resulting 52 complex ratios form the channel frequency response.
6. The same response equalizes the packet's SIGNAL and DATA symbols.
7. A valid MAC FCS confirms packet association by default.

The necessary channel information was already present in the captured Wi-Fi waveform. A special chipset CSI firmware path is not required for this software-derived legacy estimate.
