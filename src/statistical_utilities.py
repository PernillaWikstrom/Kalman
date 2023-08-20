import numpy as np
from matplotlib.patches import Ellipse


def get_ellipses(
    mean_pos_x, mean_pos_y, var_pos_x, var_pos_y, edgecolor, facecolor, nSamples
):

    covariance_pose = np.zeros((2, 2, nSamples))
    covariance_pose[0, 0] = var_pos_x
    covariance_pose[1, 1] = var_pos_y

    ellipses = []
    ellipse_params = np.zeros((3, nSamples))
    for i in range(nSamples):
        # Find eigenvalues and eigenvectors
        eigVal, eigVec = np.linalg.eig(covariance_pose[:, :, i])

        # The length of the half-major axis a is given by the highest eigenvalue square root
        a = np.sqrt(eigVal[0])
        # The length of the half-minor axis b is given by the second eigenvalue square root
        b = np.sqrt(eigVal[1])

        # Ellipse orientation angle:
        # The numerator and denominator is the x and y - coordinate of the eigenvector that corresponds to the highest eigenvalue
        theta = np.arctan(eigVec[1, 0] / eigVec[0, 0])

        ellipse_params[0, i] = a
        ellipse_params[1, i] = b
        ellipse_params[2, i] = theta
        ellipses.append(
            Ellipse(
                np.array([mean_pos_x[i], mean_pos_y[i]]),
                width=a,
                height=b,
                angle=np.rad2deg(theta),
                edgecolor=edgecolor,
                facecolor=facecolor,
                linestyle=":",
            )
        )
    return ellipses


def get_boundaries(mean, variance):
    lower_boundary = mean - 2.0 * np.sqrt(variance)
    upper_boundary = mean + 2.0 * np.sqrt(variance)
    return lower_boundary, upper_boundary


def get_states_confidence_interval(
    estimated_states, estimated_covariances, nStates, nSamples
):

    lower_boundaries = np.zeros((nStates, nSamples))
    upper_boundaries = np.zeros_like(lower_boundaries)

    for i in range(nStates):
        lower_boundaries[i], upper_boundaries[i] = get_boundaries(
            estimated_states[i], estimated_covariances[i, i]
        )
    return lower_boundaries, upper_boundaries
