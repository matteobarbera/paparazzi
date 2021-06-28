from matplotlib import pyplot as plt
import numpy as np
from scipy.io import wavfile
from scipy.signal import welch, find_peaks, peak_prominences, spectrogram, windows


def read_wav_file(filename, channel=0):
    """ Extract sample rate and data (1D) of WAV file """
    fs, data = wavfile.read(filename)
    if len(data.shape) > 1:
        print()
        print(f"WAV file has multiple channels, using channel {channel}")
        data = data[:, channel]
    return fs, data


def compute_spectrogram(fs, data):
    nwind = 4000
    noverlap = nwind - 800
    f, t_seg, sxx = spectrogram(data, fs, window=windows.hann(nwind), noverlap=noverlap)
    return f, t_seg, sxx


def compute_welch(fs, data):
    nwind = 4000
    noverlap = nwind - 800
    f_pxx, pxx = welch(data, fs, window=windows.hann(nwind), noverlap=noverlap)
    return f_pxx, pxx


def peak_frequencies(f_pxx, pxx):
    """ Find the two highest power frequencies from the Power Spectral Density """
    peaks_idx, _ = find_peaks(pxx)
    prominences = peak_prominences(pxx, peaks_idx)[0]
    highest_peaks = np.argsort(prominences)[-2:]  # index of two highest
    peaks_idx = peaks_idx[highest_peaks]
    min_peak_f, max_peak_f = sorted(f_pxx[peaks_idx])
    return peaks_idx, min_peak_f, max_peak_f


def filter_spectrogram(f_sxx, sxx):
    """ Find index of highest power frequency per time segment """
    sxx_filt = []
    for f in sxx.T:  # SXX has shape [nFreqs, nTime]
        idx_maxf = np.argmax(f)
        sxx_filt.append(f_sxx[idx_maxf])
    return np.asarray(sxx_filt)


def compute_spool_t_idx(sxx_filt_sliced, min_peak_f, max_peak_f):
    """ Find index pairs of start/end time of spool up/down """

    # The indices are concatenated in a single list
    spool_up_t_idx = []
    spool_down_t_idx = []
    for i in range(len(sxx_filt_sliced) - 1):
        if sxx_filt_sliced[i] == min_peak_f and sxx_filt_sliced[i + 1] == max_peak_f:
            spool_up_t_idx += [i, i + 1]
        elif sxx_filt_sliced[i] == max_peak_f and sxx_filt_sliced[i + 1] == min_peak_f:
            spool_down_t_idx += [i, i + 1]
    return spool_up_t_idx, spool_down_t_idx


def compute_tf(sxx_filt, t_seg, spool_up_t, max_peak_f):
    """ Use average of initial slope to compute the transfer function"""
    a_arr = []
    for t in spool_up_t[::2]:
        idx = np.argwhere(t_seg == t).flatten()[0]  # Get single idx out of ndarray
        t1, t2 = t_seg[idx + 1:idx + 3]
        f1, f2 = sxx_filt[idx + 1:idx + 3]
        slope = (f2 - f1) / (t2 - t1)
        a = (max_peak_f - f1) / slope
        a_arr.append(a)
    a_avg = np.mean(a_arr)
    transfer_func = f"{round(a_avg, 6)} / (s * (s + {round(a_avg, 6)}))"
    return transfer_func


def compute_spool_dt(t_sliced, spool_up_t_idx, spool_down_t_idx):
    """ Calculate array of dt """

    # The start/end times of dt are concatenated, so find difference between elements and take
    # every other element of the result
    spool_up_dt = np.diff(t_sliced[spool_up_t_idx])[::2]
    spool_down_dt = np.diff(t_sliced[spool_down_t_idx])[::2]
    return spool_up_dt, spool_down_dt


def plot_sxx(f_sxx, t_seg, sxx, max_peak_f, xmin, xmax, ax=None):
    if ax is None:
        ax = plt.gca()

    # We're only interested in the frequencies lower than max_peak_f
    f_slice = np.where(f_sxx <= max_peak_f + 200)  # + 250 to make it prettier
    ax.contourf(t_seg, f_sxx[f_slice], sxx[f_slice, :][0], 40, cmap='twilight')

    ax.set_xlim((xmin, xmax))
    ax.set_title("SXX")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Frequency [Hz]")
    return ax


def plot_sxx_filt(t_seg, sxx_filt, mask, min_peak_f, max_peak_f, spool_up_t_idx, spool_down_t_idx, xmin, xmax, ax=None):
    if ax is None:
        ax = plt.gca()

    t_sliced = t_seg[mask]

    ax.scatter(t_seg, sxx_filt, color="C0", alpha=0.9, zorder=2)

    ymin = min_peak_f - 50
    ymax = max_peak_f + 50
    f = np.linspace(0, ymax, 1000)  # For shading

    # Better visualize the frequencies considered in dt calculations to check for anomalies
    for i in range(0, len(t_sliced[spool_up_t_idx]) - 1, 2):
        t_start = t_sliced[spool_up_t_idx][i]
        t_end = t_sliced[spool_up_t_idx][i + 1]
        ax.vlines(t_start, ymin=ymin, ymax=ymax, color="C0", alpha=0.4, zorder=1)
        ax.vlines(t_end, ymin=ymin, ymax=ymax, color="C0", alpha=0.4, zorder=1)
        # Shade in area of dt
        ax.fill_betweenx(f, t_start, t_end, where=(f > ymin) & (f < ymax), color="C0", alpha=0.1, zorder=1)

    for i in range(0, len(t_sliced[spool_down_t_idx]) - 1, 2):
        t_start = t_sliced[spool_down_t_idx][i]
        t_end = t_sliced[spool_down_t_idx][i + 1]
        ax.vlines(t_start, ymin=ymin, ymax=ymax, color="C1", alpha=0.4, zorder=1)
        ax.vlines(t_end, ymin=ymin, ymax=ymax, color="C1", alpha=0.4, zorder=1)
        # Shade in area of dt
        ax.fill_betweenx(f, t_start, t_end, where=(f > ymin) & (f < ymax), color="C1", alpha=0.1, zorder=1)
    
    ax.scatter(t_sliced, sxx_filt[mask], marker="x", color="C1", alpha=0.4, zorder=2, label="Frequency peaks")

    ax.set_xlim((xmin, xmax))
    ax.set_ylim((0, max_peak_f + 200))
    ax.set_title("Filtered SXX")
    ax.set_ylabel("Frequency [Hz]")
    ax.set_xlabel("Time [s]")
    ax.legend()
    return ax


def plot_pxx(f_pxx, pxx, peaks_idx, ax=None):
    if ax is None:
        ax = plt.gca()

    ax.semilogx(f_pxx, pxx, color="C0")
    ax.plot(f_pxx[peaks_idx], pxx[peaks_idx], "x", color="C1")
    ax.set_title("PXX")
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PXX [V^2 / Hz]")
    return ax


def plot_hist(spool_up_dt, spool_down_dt, mean_up, mean_down, median_up, median_down, ax=None):
    if ax is None:
        ax = plt.gca()

    n, _, _ = ax.hist([spool_up_dt, spool_down_dt], 15, alpha=0.5, label=["Spool up", "Spool down"], color=["C0", "C1"])
    # Visualize mean and median
    ax.vlines(mean_up, ymin=0, ymax=np.max(n) + 1, color="C0", linestyle="--")
    ax.vlines(median_up, ymin=0, ymax=np.max(n) + 1, color="C0")
    ax.vlines(mean_down, ymin=0, ymax=np.max(n) + 1, color="C1", linestyle="--")
    ax.vlines(median_down, ymin=0, ymax=np.max(n) + 1, color="C1")
    
    # Trick for 'shared' legend
    ax.plot(spool_up_dt[0], [0], color="k", linestyle="--", label="Mean")
    ax.plot(spool_up_dt[0], [0], color="k", label="Median")
    
    ax.set_title("dt histogram")
    ax.set_xlabel("dt [s]")
    ax.legend(loc='upper center')
    return ax
