# Niryo-Robot-Kinematics-Matlab

MATLAB implementation of direct kinematics (DKM), inverse kinematics (IKM), direct velocity mapping (DVM), and inverse velocity mapping (IVM) for a 6-DOF Niryo robot.

## Direct Kinematic Model (DKM)

The direct kinematic model determines the Cartesian pose of the end effector from the robot joint coordinates

$$
\mathbf{q} =
\begin{bmatrix}
q_1 & q_2 & q_3 & q_4 & q_5 & q_6
\end{bmatrix}^{T}.
$$

### Denavit–Hartenberg transformation

The robot is modeled using the standard Denavit–Hartenberg (DH) convention. For each row of the DH table, the homogeneous transformation from frame \(i-1\) to frame \(i\) is

$$
{}^{i-1}\mathbf{T}_{i} =
\begin{bmatrix}
\cos\theta_i &
-\sin\theta_i\cos\alpha_i &
\sin\theta_i\sin\alpha_i &
a_i\cos\theta_i \\
\sin\theta_i &
\cos\theta_i\cos\alpha_i &
-\cos\theta_i\sin\alpha_i &
a_i\sin\theta_i \\
0 &
\sin\alpha_i &
\cos\alpha_i &
d_i \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

The parameters used by the MATLAB model are:

| Transform | \(a_i\) | \(\alpha_i\) | \(d_i\) | \(\theta_i\) |
|---:|---:|---:|---:|---:|
| 1 | \(0\) | \(\pi/2\) | \(L_1\) | \(q_1\) |
| 2 | \(L_2\) | \(0\) | \(0\) | \(q_2+\pi/2\) |
| 3 | \(L_3\) | \(\pi/2\) | \(0\) | \(q_3\) |
| 4 | \(0\) | \(0\) | \(L_4\) | \(0\) |
| 5 | \(0\) | \(-\pi/2\) | \(0\) | \(q_4\) |
| 6 | \(0\) | \(\pi/2\) | \(0\) | \(q_5\) |
| 7 | \(0\) | \(0\) | \(0\) | \(q_6\) |
| 8 | \(0\) | \(\pi/2\) | \(L_5\) | \(\pi/2\) |
| 9 | \(0\) | \(0\) | \(-L_6\) | \(\pi/2\) |
| 10 | \(L_{\mathrm{tool}}\) | \(0\) | \(0\) | \(0\) |

The dimensions used in the implementation are

$$
\begin{aligned}
L_1 &= 183,      & L_2 &= 210,  & L_3 &= 30,\\
L_4 &= 221.5,    & L_5 &= 23.7, & L_6 &= 5.5,\\
L_{\mathrm{tool}} &= 0.
\end{aligned}
$$

All linear dimensions are expressed in millimetres.

### End-effector pose

The base-to-end-effector transformation is obtained by multiplying the individual transformations in their kinematic order:

$$
{}^{0}\mathbf{T}_{E}(\mathbf{q})
=
\prod_{i=1}^{10}{}^{i-1}\mathbf{T}_{i}
=
{}^{0}\mathbf{T}_{1}
{}^{1}\mathbf{T}_{2}
\cdots
{}^{9}\mathbf{T}_{E}.
$$

It has the form

$$
{}^{0}\mathbf{T}_{E} =
\begin{bmatrix}
{}^{0}\mathbf{R}_{E} & {}^{0}\mathbf{p}_{E}\\
\mathbf{0}^{T} & 1
\end{bmatrix},
$$

where

$$
{}^{0}\mathbf{p}_{E} =
\begin{bmatrix}
E_x & E_y & E_z
\end{bmatrix}^{T}
$$

is the end-effector position and \({}^{0}\mathbf{R}_{E}\in SO(3)\) is its orientation. The function `DKM.m` returns the position together with equivalent roll–pitch–yaw and Euler-angle representations extracted from this rotation matrix.

## Inverse Velocity Mapping (IVM)

Inverse velocity mapping calculates the joint velocities required to produce a specified end-effector Cartesian velocity.

The end-effector twist is defined as

$$
\dot{\mathbf{x}} =
\begin{bmatrix}
{}^{0}\mathbf{v}_{E}\\
{}^{0}\boldsymbol{\omega}_{E}
\end{bmatrix}
=
\begin{bmatrix}
v_x & v_y & v_z & \omega_x & \omega_y & \omega_z
\end{bmatrix}^{T},
$$

where \({}^{0}\mathbf{v}_{E}\) is the linear velocity and \({}^{0}\boldsymbol{\omega}_{E}\) is the angular velocity, both expressed in the base frame.

### Geometric Jacobian

For revolute joint \(i\), let \({}^{0}\mathbf{z}_{i-1}\) denote the joint-axis unit vector, \({}^{0}\mathbf{p}_{i-1}\) the joint origin, and \({}^{0}\mathbf{p}_{E}\) the end-effector position. The corresponding column of the geometric Jacobian is

$$
\mathbf{J}_i(\mathbf{q}) =
\begin{bmatrix}
{}^{0}\mathbf{z}_{i-1}
\times
\left({}^{0}\mathbf{p}_{E}-{}^{0}\mathbf{p}_{i-1}\right)\\
{}^{0}\mathbf{z}_{i-1}
\end{bmatrix}.
$$

Therefore,

$$
\mathbf{J}(\mathbf{q}) =
\begin{bmatrix}
\mathbf{J}_1 &
\mathbf{J}_2 &
\mathbf{J}_3 &
\mathbf{J}_4 &
\mathbf{J}_5 &
\mathbf{J}_6
\end{bmatrix}
\in \mathbb{R}^{6\times6}.
$$

The differential kinematic relation is

$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}},
$$

with

$$
\dot{\mathbf{q}} =
\begin{bmatrix}
\dot q_1 & \dot q_2 & \dot q_3 &
\dot q_4 & \dot q_5 & \dot q_6
\end{bmatrix}^{T}.
$$

The inverse velocity solution implemented in `IVM.m` is

$$
\boxed{
\dot{\mathbf{q}} =
\mathbf{J}^{\dagger}(\mathbf{q})\dot{\mathbf{x}}
}
$$

where \(\mathbf{J}^{\dagger}\) is the Moore–Penrose pseudoinverse, evaluated in MATLAB using `pinv(J)`. When the Jacobian is nonsingular, \(\mathbf{J}^{\dagger}=\mathbf{J}^{-1}\). At or near a singular configuration, the pseudoinverse provides a least-squares, minimum-norm joint-velocity solution; however, the resulting joint velocities may become large when the Jacobian is poorly conditioned.

With link lengths specified in millimetres, the expected twist units are millimetres per second for the linear component and radians per second for the angular component. The resulting joint velocities are expressed in radians per second.

