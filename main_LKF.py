import numpy as np
import matplotlib.pyplot as plt
import src


def plotting(
    df,
    df_gt,
    estimated_states,
    estimated_covariances,
    variance_measurements,
    nStates,
    nSamples,
):

    lower_boundaries, upper_boundaries = src.get_states_confidence_interval(
        estimated_states, estimated_covariances, nStates, nSamples
    )
    lower_boundary_measurement_px, upper_boundary_measurement_px = src.get_boundaries(
        df["x"], variance_measurements
    )
    lower_boundary_measurement_py, upper_boundary_measurement_py = src.get_boundaries(
        df["y"], variance_measurements
    )

    ellipses_pose_estimates = src.get_ellipses(
        estimated_states[0],
        estimated_states[3],
        estimated_covariances[0, 0],
        estimated_covariances[3, 3],
        "darkgreen",
        "lightgreen",
        nSamples,
    )

    ellipses_pose_measurements = src.get_ellipses(
        df["x"],
        df["y"],
        variance_measurements,
        variance_measurements,
        "orange",
        "lightyellow",
        nSamples,
    )

    fig_pose, ax_pose = plt.subplots()
    ax_pose.plot(
        df_gt["px"].iloc[-1],
        df_gt["py"].iloc[-1],
        "*",
        color="gold",
        markersize=25.0,
        label="Ground truth end",
    )
    ax_pose.plot(df_gt["px"], df_gt["py"], "-o", color="black", label="Ground Truth")
    ax_pose.plot(df["x"], df["y"], "-*", color="orange", label="Measurements")
    for i, ellipse in enumerate(ellipses_pose_measurements):
        ax_pose.add_artist(ellipse)
        if i == (len(ellipses_pose_measurements) - 1):
            ellipse.set_label("2$\sigma$ measurement")

    ax_pose.plot(
        estimated_states[0],
        estimated_states[3],
        "*-",
        color="darkgreen",
        label="Estimated states",
    )

    for i, ellipse in enumerate(ellipses_pose_estimates):
        ax_pose.add_artist(ellipse)
        if i == (len(ellipses_pose_estimates) - 1):
            ellipse.set_label("2$\sigma$ estimates")

    ax_pose.set_xlim(0, 302)
    ax_pose.set_ylim(-400, 400)
    ax_pose.grid()
    ax_pose.set_xlabel("Position X\n[m]")
    ax_pose.set_ylabel("Position Y\n[m]")
    ax_pose.legend()
    fig_pose.tight_layout()

    fig_pose_propagation_with_time, (ax_pose_x, ax_pose_y) = plt.subplots(
        nrows=2, ncols=1, sharex=True
    )
    ax_pose_x.plot(df["time"], df_gt["px"], "-o", color="black", label="Ground truth")
    ax_pose_x.plot(df["time"], df["x"], "-*", color="orange", label="Measurement")
    ax_pose_x.fill_between(
        df["time"],
        lower_boundary_measurement_px,
        upper_boundary_measurement_px,
        facecolor="lightYellow",
    )
    ax_pose_x.plot(
        df["time"],
        lower_boundary_measurement_px,
        ":",
        color="orange",
        label="2$\sigma$ measurement",
    )
    ax_pose_x.plot(df["time"], upper_boundary_measurement_px, ":", color="orange")
    ax_pose_x.plot(
        df["time"],
        estimated_states[0],
        "-d",
        color="darkgreen",
        label="Estimated state",
    )
    ax_pose_x.fill_between(
        df["time"], lower_boundaries[0], upper_boundaries[0], facecolor="lightgreen"
    )
    ax_pose_x.plot(
        df["time"],
        lower_boundaries[0],
        ":",
        color="darkgreen",
        label="2$\sigma$ state",
    )
    ax_pose_x.plot(df["time"], upper_boundaries[0], ":", color="darkgreen")
    ax_pose_x.grid()
    ax_pose_x.set_ylabel("Position X\n[m]")
    ax_pose_x.legend()
    ax_pose_y.plot(df["time"], df_gt["py"], "-o", color="black", label="Ground truth")
    ax_pose_y.plot(df["time"], df["y"], "-*", color="orange", label="Measurement")
    ax_pose_y.fill_between(
        df["time"],
        lower_boundary_measurement_py,
        upper_boundary_measurement_py,
        facecolor="lightYellow",
    )
    ax_pose_y.plot(
        df["time"],
        lower_boundary_measurement_py,
        ":",
        color="orange",
        label="2$\sigma$ measurement",
    )
    ax_pose_y.plot(df["time"], upper_boundary_measurement_py, ":", color="orange")
    ax_pose_y.plot(
        df["time"],
        estimated_states[3],
        "-d",
        color="darkgreen",
        label="Estimated state",
    )
    ax_pose_y.fill_between(
        df["time"], lower_boundaries[3], upper_boundaries[3], facecolor="lightgreen"
    )
    ax_pose_y.plot(
        df["time"],
        lower_boundaries[3],
        ":",
        color="darkgreen",
        label="2$\sigma$ state",
    )
    ax_pose_y.plot(df["time"], upper_boundaries[3], ":", color="darkgreen")
    ax_pose_y.grid()
    ax_pose_y.set_xlabel("Time [s]")
    ax_pose_y.set_ylabel("Position Y\n[m]")
    ax_pose_y.legend()
    fig_pose_propagation_with_time.tight_layout()

    fig_error, (ax_error) = plt.subplots(nrows=1, ncols=1)
    error_x = df_gt["px"] - estimated_states[0]
    error_y = df_gt["py"] - estimated_states[3]

    rmse = np.sqrt(np.sum(error_x**2.0 + error_y**2.0)) / nSamples
    ax_error.plot(df["time"], error_x, label="Error x")
    ax_error.plot(df["time"], error_y, label="Error y")
    ax_error.plot(df["time"], np.ones_like(error_x) * rmse, label="RMSE")
    ax_error.legend()
    ax_error.grid()
    ax_error.set_xlabel("Time [s]")
    ax_error.set_ylabel("Error [m]")

    fig_velocity, (ax_vel_x, ax_vel_y) = plt.subplots(nrows=2, ncols=1, sharex=True)
    ax_vel_x.plot(df["time"], df_gt["vx"], "-o", color="black", label="Ground truth")
    ax_vel_x.plot(
        df["time"],
        estimated_states[1],
        "-d",
        color="darkgreen",
        label="Estimated state",
    )
    ax_vel_x.fill_between(
        df["time"], lower_boundaries[1], upper_boundaries[1], facecolor="lightgreen"
    )
    ax_vel_x.plot(
        df["time"],
        lower_boundaries[1],
        ":",
        color="darkgreen",
        label="2$\sigma$ state",
    )
    ax_vel_x.plot(df["time"], upper_boundaries[1], ":", color="darkgreen")
    ax_vel_x.grid()
    ax_vel_x.set_ylabel("Velocity Vx\n[m/s]")
    ax_vel_x.legend()
    ax_vel_y.plot(df["time"], df_gt["vy"], "-o", color="black", label="Ground truth")
    ax_vel_y.plot(
        df["time"],
        estimated_states[4],
        "-d",
        color="darkgreen",
        label="Estimated state",
    )
    ax_vel_y.fill_between(
        df["time"], lower_boundaries[4], upper_boundaries[4], facecolor="lightgreen"
    )
    ax_vel_y.plot(
        df["time"],
        lower_boundaries[4],
        ":",
        color="darkgreen",
        label="2$\sigma$ state",
    )
    ax_vel_y.plot(df["time"], upper_boundaries[4], ":", color="darkgreen")
    ax_vel_y.grid()
    ax_vel_y.set_xlabel("Time [s]")
    ax_vel_y.set_ylabel("Velocity Vy\n[m/s]")
    ax_vel_y.legend()
    fig_velocity.tight_layout()

    fig_acceleration, (ax_acc_x, ax_acc_y) = plt.subplots(nrows=2, ncols=1, sharex=True)
    ax_acc_x.plot(df["time"], df_gt["ax"], "-o", color="black", label="Ground truth")
    ax_acc_x.plot(
        df["time"],
        estimated_states[2],
        "-d",
        color="darkgreen",
        label="Estimated state",
    )
    ax_acc_x.fill_between(
        df["time"], lower_boundaries[2], upper_boundaries[2], facecolor="lightgreen"
    )
    ax_acc_x.plot(
        df["time"],
        lower_boundaries[2],
        ":",
        color="darkgreen",
        label="2$\sigma$ state",
    )
    ax_acc_x.plot(df["time"], upper_boundaries[2], ":", color="darkgreen")
    ax_acc_x.grid()
    ax_acc_x.set_ylabel("Acceleration\n[m/s²]")
    ax_acc_x.legend()
    ax_acc_y.plot(df["time"], df_gt["ay"], "-o", color="black", label="Ground truth")
    ax_acc_y.plot(
        df["time"],
        estimated_states[5],
        "-d",
        color="darkgreen",
        label="Estimated state",
    )
    ax_acc_y.fill_between(
        df["time"], lower_boundaries[5], upper_boundaries[5], facecolor="lightgreen"
    )
    ax_acc_y.plot(
        df["time"],
        lower_boundaries[5],
        ":",
        color="darkgreen",
        label="2$\sigma$ state",
    )
    ax_acc_y.plot(df["time"], upper_boundaries[5], ":", color="darkgreen")
    ax_acc_y.grid()
    ax_acc_y.set_xlabel("Time [s]")
    ax_acc_y.set_ylabel("Acceleration\n[m/s²]")
    ax_acc_y.legend()
    fig_acceleration.tight_layout()

    fig_covariance, (ax_cov_p, ax_cov_v, ax_cov_a) = plt.subplots(
        nrows=3, ncols=1, sharex=True
    )
    ax_cov_p.plot(
        np.sqrt(estimated_covariances[0, 0]),
        "-o",
        color="darkgreen",
        label="$\mu_{Px}$",
    )
    ax_cov_p.plot(
        np.sqrt(estimated_covariances[3, 3]),
        "-*",
        color="lightgreen",
        label="$\mu_{Py}$",
    )
    ax_cov_p.set_ylabel("Positions [m]")
    ax_cov_p.legend()
    ax_cov_p.grid()
    ax_cov_v.plot(
        np.sqrt(estimated_covariances[1, 1]),
        "-o",
        color="darkgreen",
        label="$\mu_{Vx}$",
    )
    ax_cov_v.plot(
        np.sqrt(estimated_covariances[4, 4]),
        "-*",
        color="lightgreen",
        label="$\mu_{Vy}$",
    )
    ax_cov_v.set_ylabel("Velocity [m/s]")
    ax_cov_v.legend()
    ax_cov_v.grid()
    ax_cov_a.plot(
        np.sqrt(estimated_covariances[2, 2]),
        "-o",
        color="darkgreen",
        label="$\mu_{Ax}$",
    )
    ax_cov_a.plot(
        np.sqrt(estimated_covariances[5, 5]),
        "-*",
        color="lightgreen",
        label="$\mu_{Ay}$",
    )
    ax_cov_a.set_ylabel("Acceleration [m/s²]")
    ax_cov_a.legend()
    ax_cov_a.set_xlabel("Time [s]")
    ax_cov_a.grid()
    fig_covariance.tight_layout()
    fig_covariance.suptitle("Estimated standard deviation")


def main(path_to_measurements, path_to_ground_truth_data, parser):

    # Get data
    df, df_gt = src.load_data(
        path_to_measurements, path_to_ground_truth_data, parser.dt
    )
    nSamples = len(df)

    # Initialize Kalman
    KF = src.LinearKalmanFilter(parser.dt)
    initial_state_vector = np.zeros((KF.nStates, 1))
    cov = src.Covariance(std=parser.std_measurements)
    KF.initialize(
        initial_state_vector=initial_state_vector,
        std_measurements=cov.std,
        std_process=parser.std_process,
    )

    estimated_states = np.zeros((KF.nStates, nSamples))
    estimated_covariances = np.zeros((6, 6, nSamples))

    for i in range(nSamples):
        KF.load_measurement(df["x"][i], df["y"][i])
        KF.predict()
        KF.update()

        # Logging
        estimated_states[:, i] = KF.state_vector[:, 0]
        estimated_covariances[:, :, i] = KF.P

    plotting(
        df,
        df_gt,
        estimated_states,
        estimated_covariances,
        cov.variance,
        KF.nStates,
        nSamples,
    )
    plt.show()


if __name__ == "__main__":
    path_to_measurements = "data/vehicle_location_measurements.csv"
    path_to_ground_truth_data = "data/ground_truth.csv"
    parser = src.get_parser()
    main(path_to_measurements, path_to_ground_truth_data, parser)
