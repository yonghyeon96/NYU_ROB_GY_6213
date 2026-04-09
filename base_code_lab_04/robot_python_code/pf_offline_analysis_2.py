import math
from pathlib import Path

import matplotlib.pyplot as plt

import particle_filter
import parameters
from pf_offline_analysis import (
    run_pf_on_log,
    try_extract_camera_truth_xy,
    plot_xy_trajectories,
    plot_particle_cloud_overlay,
    plot_error_vs_time,
    plot_error_metrics_table,
    print_run_summary,
)


if __name__ == "__main__":
    # ---------------------------
    # 1) Select data file
    # ---------------------------
    data_file = "./data/straight.pkl"
    if not Path(data_file).exists():
        raise FileNotFoundError(f"Data file not found: {data_file}")

    # ---------------------------
    # 2) Known vs Unknown setup
    # ---------------------------
    # "Known" initial guess
    known_initial_state = particle_filter.State(0.68, 0.53, 0.0)
    known_initial_stdev = particle_filter.State(0.1, 0.1, 0.1)

    # Enable/disable unknown case for this script.
    # If False, Plot 1 will only show known trajectory.
    use_unknown_case = True

    # Unknown offset (meters, meters, degrees)
    unknown_offset_x_m = 0.30
    unknown_offset_y_m = -0.30
    unknown_offset_theta_deg = 0.5
    unknown_initial_state = particle_filter.State(
        known_initial_state.x + unknown_offset_x_m,
        known_initial_state.y + unknown_offset_y_m,
        particle_filter.angle_wrap(known_initial_state.theta + math.radians(unknown_offset_theta_deg)),
    )

    # ---------------------------
    # 3) Optional manual truth line for Plot 1
    # ---------------------------
    use_manual_truth_line = False
    manual_truth_start = (0.68, 0.53)
    manual_truth_end = (1.88, 0.53)
    manual_truth_label = "manual truth (measured start-end)"

    manual_truth_by_file = {
        # "robot_data_35_0_05_03_26_18_08_09.pkl": ((0.68, 0.53), (1.88, 0.53)),
    }
    data_filename = Path(data_file).name
    if data_filename in manual_truth_by_file:
        use_manual_truth_line = True
        manual_truth_start, manual_truth_end = manual_truth_by_file[data_filename]

    # ---------------------------
    # 4) Run PF
    # ---------------------------
    map_obj = particle_filter.Map(parameters.wall_corner_list)

    known_pf = run_pf_on_log(
        filename=data_file,
        run_name="Known start (Full PF)",
        use_correction=True,
        known_start_state=True,
        initial_state=known_initial_state,
        initial_stdev=known_initial_stdev,
        seed=1,
    )

    unknown_pf = None
    if use_unknown_case:
        unknown_pf = run_pf_on_log(
            filename=data_file,
            run_name=(
                f"Unknown start (dx={unknown_offset_x_m:+.2f} m, "
                f"dy={unknown_offset_y_m:+.2f} m, dth={unknown_offset_theta_deg:+.1f} deg)"
            ),
            use_correction=True,
            known_start_state=True,
            initial_state=unknown_initial_state,
            initial_stdev=known_initial_stdev,
            seed=1,
        )

    print_run_summary(known_pf)
    if unknown_pf is not None:
        print_run_summary(unknown_pf)

    # Camera truth overlay only for Plot 1 if available in the log
    truth_xy, truth_valid = try_extract_camera_truth_xy(data_file, len(known_pf.time))
    if truth_xy is None:
        print("No usable camera truth found in log. XY plot will skip camera truth overlay.")

    # ---------------------------
    # 5) Plot 1: KNOWN vs UNKNOWN trajectory
    # ---------------------------
    plot1_runs = [known_pf] if unknown_pf is None else [known_pf, unknown_pf]
    plot_xy_trajectories(
        map_obj,
        plot1_runs,
        truth_xy=truth_xy,
        truth_valid=truth_valid,
        manual_truth_enabled=use_manual_truth_line,
        manual_truth_start=manual_truth_start,
        manual_truth_end=manual_truth_end,
        manual_truth_label=manual_truth_label,
    )

    # ---------------------------
    # 6) Optional additional plots
    # ---------------------------
    plot_particle_cloud_overlay(map_obj, known_pf, max_snapshots=6)
    if unknown_pf is not None:
        plot_particle_cloud_overlay(map_obj, unknown_pf, max_snapshots=6)
        plot_error_vs_time([known_pf, unknown_pf])
        plot_error_metrics_table([known_pf, unknown_pf])

    plt.show()
