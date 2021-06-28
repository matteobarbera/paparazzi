import argparse

from motor_dynamics_utils import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compute motor spool up and spool down time")
    parser.add_argument("filename", metavar="wavfile", type=str,
                        help="WAV file to be processed")
    parser.add_argument("-c", "--channel", metavar="C", type=int, default=0,
                        help="WAV file channel index")
    args = parser.parse_args()

    fs, data = read_wav_file(args.filename, args.channel)

    f_sxx, t_seg, sxx = compute_spectrogram(fs, data)
    f_pxx, pxx = compute_welch(fs, data)

    # Calculate frequency corresponding to the two energy peaks
    peaks_idx, min_peak_f, max_peak_f = peak_frequencies(f_pxx, pxx)
    print()
    print(f"Frequency peaks: {min_peak_f} [Hz] {max_peak_f} [Hz]")

    # Take only the highest frequency per time segment
    sxx_filt = filter_spectrogram(f_sxx, sxx)

    # Only take frequencies corresponding to peaks
    peak_f_mask = (sxx_filt == min_peak_f) | (sxx_filt == max_peak_f)
    t_sliced = t_seg[peak_f_mask]
    sxx_filt_sliced = sxx_filt[peak_f_mask]

    # Compute indices of start/end time pairs
    spool_up_t_idx, spool_down_t_idx = compute_spool_t_idx(sxx_filt_sliced, min_peak_f, max_peak_f)
    # Compute transfer function
    tf = compute_tf(sxx_filt, t_seg, t_sliced[spool_up_t_idx], max_peak_f)
    # Compute time difference between pairs to get spool up/down delta t
    spool_up_dt, spool_down_dt = compute_spool_dt(t_sliced, spool_up_t_idx, spool_down_t_idx)

    mean_dt_up = np.mean(spool_up_dt)
    mean_dt_down = np.mean(spool_down_dt)
    median_dt_up = np.median(spool_up_dt)
    median_dt_down = np.median(spool_down_dt)

    print()
    print("========= Spool up ==========")
    print(f"Mean: {round(mean_dt_up, 5)} [s]   Median: {round(median_dt_up, 5)} [s]")
    print("======== Spool down =========")
    print(f"Mean: {round(mean_dt_down, 5)} [s]   Median: {round(median_dt_down, 5)} [s]")

    print()
    print("========= Transfer Function =========")
    print(tf)

    plt.rcParams.update({'font.size': 16})
    fig, axs = plt.subplots(2, 2)
    fig.tight_layout()
    for ax in axs.reshape(-1):
        ax.grid(alpha=0.5)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    t_start = min(t_sliced[spool_up_t_idx]) - 0.5
    t_end = max(t_sliced[spool_down_t_idx]) + 0.5
    plot_sxx(f_sxx, t_seg, sxx, max_peak_f, t_start, t_end, ax=axs[0, 0])
    plot_sxx_filt(t_seg, sxx_filt, peak_f_mask, min_peak_f, max_peak_f, spool_up_t_idx, spool_down_t_idx, t_start, t_end, ax=axs[0, 1])
    plot_pxx(f_pxx, pxx, peaks_idx, ax=axs[1, 0])
    plot_hist(spool_up_dt, spool_down_dt, mean_dt_up, mean_dt_down, median_dt_up, median_dt_down, ax=axs[1, 1])

    plt.show()
