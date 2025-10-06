# Heart Rate Detection Algorithms

This project includes two different algorithms for heart rate detection from MAX301### Configuration

### Buffer Size
- Minimum: 128 samples at 100 Hz (32 samples at 25 Hz after decimation)
- Default: 512 samples at 100 Hz (128 samples at 25 Hz)
- Optimal: 512-1024 samples for best accuracy

### Sample Rate
- Collection: 100 Hz
- Processing: 25 Hz (decimated by 4)
- FIR designed for: 25 Hz ✓als.

## Algorithm Selection

Change the algorithm by modifying `USE_FIR_ALGORITHM` in `hello_world_main.c`:
- `0` = Autocorrelation-based algorithm
- `1` = FIR filter-based algorithm (default)

## 1. Autocorrelation Algorithm

### Method
- **Preprocessing**: Smooth → Remove DC → Remove trend
- **Detection**: Autocorrelation to find periodic peaks
- **Frequency Range**: 40-180 BPM (0.67-3 Hz)

### How it Works
1. Smooths signal with 3-point moving average
2. Removes DC component (mean)
3. Removes linear trend using regression
4. Calculates autocorrelation for different time lags
5. Searches for peaks in 3 ranges (prioritizing 50-90 BPM)
6. Converts peak lag to heart rate

### Advantages
- No filter startup transients
- Works with any buffer size
- Good for stable, rhythmic signals
- Finds fundamental frequency

### Disadvantages
- Sensitive to noise and artifacts
- Can lock onto harmonics
- Computationally intensive (O(n²) for full search)
- Needs longer signals for accuracy

### Parameters
- `AUTOCORR_THRESHOLD_HIGH`: 0.25 (good peak)
- `AUTOCORR_THRESHOLD_LOW`: 0.20 (acceptable peak)
- Prefers 50-90 BPM range

## 2. FIR Filter Algorithm

### Method
- **Preprocessing**: Remove DC only
- **Downsampling**: 100 Hz → 25 Hz (every 4th sample)
- **Filtering**: 95-tap FIR bandpass (0.3-4 Hz at 25 Hz)
- **Detection**: Peak detection in filtered signal

### How it Works
1. Downsamples from 100 Hz to 25 Hz (takes every 4th sample)
2. Removes DC component (mean)
3. Applies 95-tap FIR bandpass filter
4. Calculates adaptive threshold (mean + 0.3×std_dev)
5. Finds peaks above threshold (with minimum spacing)
6. Calculates average interval between peaks
7. Converts to heart rate

### FIR Filter Specifications
- **Taps**: 95
- **Design Sample Rate**: 25 Hz
- **Actual Sample Rate**: 25 Hz (downsampled from 100 Hz)
- **Passband**: 0.3-4 Hz (18-240 BPM range)
- **Cutoff Width**: 0.3 Hz (lower), 1.25 Hz (upper)
- **Type**: Bandpass
- **Implementation**: Custom C implementation (no external dependencies)

### Advantages
- Excellent noise rejection
- Sharp frequency selectivity
- Linear phase (no distortion)
- No external dependencies
- Simple peak detection
- Efficient (operates on downsampled data)

### Disadvantages
- 95-sample startup transient (at 25 Hz = 3.8s)
- Fixed frequency response
- Requires minimum buffer size (>128 samples at 100 Hz)
- Can miss beats if peaks are weak
- Loses 75% of samples through decimation

### Parameters
- Adaptive threshold: `mean + 0.3 × std_dev`
- Valid BPM range filtered: 20-200 BPM
- Quality based on signal strength

## Performance Comparison

| Feature | Autocorrelation | FIR Filter |
|---------|----------------|------------|
| Noise Handling | Moderate | Excellent |
| Harmonic Rejection | Poor | Good |
| Startup Time | Immediate | 3.8s (95 samples @ 25Hz) |
| Computation | High | Low (downsampled) |
| Memory | Low | Medium (decimation) |
| Stability | Variable | High |
| Best For | Clean signals | Noisy signals |
| Dependencies | None | None |

## Usage Example

```c
algorithm_result_t result;

// Option 1: Autocorrelation
algorithm_process_signals(ir_buffer, red_buffer, size, &result);

// Option 2: FIR-based
algorithm_process_signals_fir(ir_buffer, red_buffer, size, &result);

if (result.valid) {
    printf("HR: %d BPM, SpO2: %.1f%%, Quality: %.3f\n",
           result.heart_rate, result.spo2, result.quality);
}
```

## Configuration

### Buffer Size
- Minimum: 256 samples (2.56s at 100 Hz)
- Default: 512 samples (5.12s at 100 Hz)
- Optimal: 500-1000 samples for best accuracy

### Sample Rate
- Expected: 100 Hz
- FIR designed for: 25 Hz (may need adjustment)
- Both algorithms adapt to actual rate

### Analysis Interval
- Default: 5 seconds between analyses
- Adjust via `ANALYSIS_INTERVAL_MS`

## Recommendations

### Use Autocorrelation When:
- Signal is clean and stable
- You need immediate results (no startup delay)
- Computational power is available
- Working with longer signal segments

### Use FIR Filter When:
- Signal is noisy (motion artifacts, etc.)
- You need consistent, reliable results
- ESP-DSP optimization is available
- You can tolerate startup delay

### For Best Results:
1. Use FIR algorithm by default
2. Ensure good sensor contact
3. Collect at least 512 samples before analysis
4. Run analysis every 3-5 seconds
5. Validate with `result.valid` flag
6. Check `result.quality` metric (>0.3 is good)

## Future Improvements

1. **Adaptive Filtering**: Adjust FIR cutoffs based on detected HR
2. **Peak Quality**: Score individual peaks, reject outliers
3. **Kalman Filter**: Smooth HR estimates over time
4. **Template Matching**: Learn personal pulse shape
5. **Multi-Algorithm Fusion**: Combine both for best of both worlds
6. **Motion Detection**: Flag data during movement
7. **Breathing Rate**: Extract from 0.1-0.5 Hz band

## Filter Design Notes

The FIR filter is designed for 25 Hz sampling. The implementation:
1. **Downsamples** from 100 Hz to 25 Hz (takes every 4th sample)
2. **Applies** 95-tap FIR filter at 25 Hz
3. **Passband**: 0.3-4 Hz = 18-240 BPM range
4. **Transient**: 95 samples at 25 Hz = 3.8 seconds

This matches the original filter design perfectly! No redesign needed.

Benefits of downsampling:
- ✅ Filter works at designed sample rate
- ✅ 4× less computation for FIR filtering
- ✅ Smaller memory footprint
- ✅ Better SNR through implicit anti-aliasing

To redesign for different sample rate:
```python
import scipy.signal as signal
fs = 25  # Sample rate
cutoff_low = 0.3  # Hz (18 BPM)
cutoff_high = 4.0  # Hz (240 BPM)
taps = signal.firwin(95, [cutoff_low, cutoff_high], 
                     fs=fs, pass_zero=False)
```
