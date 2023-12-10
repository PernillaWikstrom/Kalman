# Installation
```bash

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

# Kalman
This is a repository for kalman filtering


## Project 1: Linear Kalman filter with simulated ground truth data:

### Theory
A vehicle's position moves in 2D space with its position (x,y), the states $\hat{x}_k$ for the kalman filter are the pose and speed in both x- and y-directions. By taking the current position and integrate speed with the time step and adding the second integration of the acceleration we can compute a new position (x,y). The state vector $\textbf{x}_k\in m\times 1 $ is formed as;

$\textbf{x}_k= \begin{Bmatrix}\hat{x}_k \\ \dot{\hat{x}}_k \\ \ddot{\hat{x}}_k \\  \hat{y}_k\\  \dot{\hat{y}}_k \\ \ddot{\hat{y}}_k\end{Bmatrix}$

When that is known the state transition matrix $F\in m\times m $ id formed as;

$F=\begin{Bmatrix}
1 & \Delta t & \frac{\Delta t^2}{2}& 0&0  & 0\\
0 & 1& \Delta t& 0 & 0 & 0 \\
0& 0&1 &0 & 0 & 0 \\
0&0  & 0 &1 & \Delta t & \frac{\Delta t^2}{2}\\
0 & 0 &0 &0 & 1& \Delta t \\
0& 0&0 &0 & 0 & 1 \\
\end{Bmatrix}$

Such that for each new prediction we have that

$\textbf{x}_k^p = F\textbf{x}_{k-1} + w_k$.

The prediction of the covariance is computed by:

$\textbf{P}_k^p = \textbf{F}\textbf{P}_{k-1} \textbf{F}^T + \textbf{Q}$.


where $Q$ represents the covariance matrix of the process,

$Q= \sigma_a^2 \begin{Bmatrix}
\frac{\Delta t^4}{4} &  \frac{\Delta t^3}{2} & \frac{\Delta t^2}{2}& 0&0  & 0\\
\frac{\Delta t^3}{2} & \frac{\Delta t^2}{2}& \Delta t& 0 & 0 & 0 \\
\frac{\Delta t^2}{2}& \Delta t&1&0 & 0 & 0 \\
0&0  & 0 &\frac{\Delta t^4}{4} &  \frac{\Delta t^3}{2} & \frac{\Delta t^2}{2}\\
0 & 0 &0 &\frac{\Delta t^3}{2} & \frac{\Delta t^2}{2}& \Delta t \\
0& 0&0 &\frac{\Delta t^2}{2}& \Delta t&1 \\
\end{Bmatrix}$

When the predictions are done the Kalman filter uses the measurements to correctly update the estimated states. Then, measurements and the observed states will be compared to get an innovation which can be accounted for, meaning using the 2D-position measurements $\bar{p}_k$;

$\bar{p}_k= \begin{Bmatrix}p^x_k \\ p^y_k\end{Bmatrix}$

Then by forming the observation matrix:

$H =\begin{Bmatrix}
1& 0&0 &0 & 0 & 0 \\
0& 0&0 &1 & 0 & 0 \\
\end{Bmatrix}$

the innovation of the state $y_k$ is computed as:

$ \bar{y}_k = \bar{p}_k - H\textbf{x}_k^p $

and the innovation of the covariance $S$ as:

$ \bar{S}_k = \textbf{H}\textbf{P}_k^p \textbf{H}^T + \textbf{R}$

where $R$ represents the covariance matrix of the measurements,

$R= \begin{Bmatrix}
\sigma_x^2 & 0 \\
0 & \sigma_y^2 \\
\end{Bmatrix}$

When the innovation is known, the kalman gain is computed as

$K = \textbf{P}_k^p H^T \bar{S}_k^{-1}$

and the states is updated by taking the previous state, adding the new prediction multiplied by the kalman gain


$\textbf{x}_k = \textbf{x}_{k-1} + K \bar{y}_k \newline
\textbf{P}_k = (I-KH)\textbf{P}_{k-1}k$.

## Running the script

```bash

python3 main_LKF.py ---std_measurements sigma_p --std_process sigma_a
```
$\sigma_a$ : [m/s²] standard deviation of the process assuming that the acceleration is constant.

$\sigma_p$ : [m] standard deviation of the position measurements.
