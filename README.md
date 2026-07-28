# Niryo Robot Kinematics in MATLAB

MATLAB implementation of the direct kinematic model (DKM), inverse
kinematic model (IKM), direct velocity model (DVM), and inverse velocity
model (IVM) for a 6-DOF Niryo robot.

## Direct Kinematic Model (DKM)

The direct kinematic model determines the Cartesian pose of the end
effector from the joint-coordinate vector:

```math
\mathbf{q}
=
\begin{bmatrix}
q_1 & q_2 & q_3 & q_4 & q_5 & q_6
\end{bmatrix}^{T}.
```

### Denavit-Hartenberg transformation

The robot is modeled using the standard Denavit-Hartenberg (DH)
convention. For each link, the homogeneous transformation from frame
$i-1$ to frame $i$ is:

```math
{}^{i-1}\mathbf{T}_{i}
=
\begin{bmatrix}
\cos\theta_i
&
-\sin\theta_i\cos\alpha_i
&
\sin\theta_i\sin\alpha_i
&
a_i\cos\theta_i
\\
\sin\theta_i
&
\cos\theta_i\cos\alpha_i
&
-\cos\theta_i\sin\alpha_i
&
a_i\sin\theta_i
\\
0
&
\sin\alpha_i
&
\cos\alpha_i
&
d_i
\\
0 & 0 & 0 & 1
\end{bmatrix}.
```

The DH parameters implemented in this project are:

| Transform | $a_i$ | $\alpha_i$ | $d_i$ | $\theta_i$ |
|:--:|:--:|:--:|:--:|:--:|
| 1 | $0$ | $\pi/2$ | $L_1$ | $q_1$ |
| 2 | $L_2$ | $0$ | $0$ | $q_2+\pi/2$ |
| 3 | $L_3$ | $\pi/2$ | $0$ | $q_3$ |
| 4 | $0$ | $0$ | $L_4$ | $0$ |
| 5 | $0$ | $-\pi/2$ | $0$ | $q_4$ |
| 6 | $0$ | $\pi/2$ | $0$ | $q_5$ |
| 7 | $0$ | $0$ | $0$ | $q_6$ |
| 8 | $0$ | $\pi/2$ | $L_5$ | $\pi/2$ |
| 9 | $0$ | $0$ | $-L_6$ | $\pi/2$ |
| 10 | $L_{\mathrm{tool}}$ | $0$ | $0$ | $0$ |

The link dimensions used in the MATLAB implementation are:

```math
\begin{aligned}
L_1 &= 183\ \mathrm{mm},
&
L_2 &= 210\ \mathrm{mm},
&
L_3 &= 30\ \mathrm{mm},
\\
L_4 &= 221.5\ \mathrm{mm},
&
L_5 &= 23.7\ \mathrm{mm},
&
L_6 &= 5.5\ \mathrm{mm},
\\
L_{\mathrm{tool}} &= 0\ \mathrm{mm}.
\end{aligned}
```

### End-effector pose

The base-to-end-effector transformation is obtained by multiplying the ten
DH transformations in their kinematic order:

```math
{}^{0}\mathbf{T}_{E}(\mathbf{q})
=
\prod_{i=1}^{10}{}^{i-1}\mathbf{T}_{i}
=
{}^{0}\mathbf{T}_{1}
{}^{1}\mathbf{T}_{2}
\cdots
{}^{9}\mathbf{T}_{E}.
```

The resulting homogeneous transformation is partitioned into a rotation
matrix and a position vector:

```math
{}^{0}\mathbf{T}_{E}
=
\begin{bmatrix}
{}^{0}\mathbf{R}_{E}
&
{}^{0}\mathbf{p}_{E}
\\
\mathbf{0}^{T}
&
1
\end{bmatrix},
\qquad
{}^{0}\mathbf{p}_{E}
=
\begin{bmatrix}
E_x \\ E_y \\ E_z
\end{bmatrix}.
```

Here, ${}^{0}\mathbf{R}_{E}\in SO(3)$ describes the end-effector
orientation and ${}^{0}\mathbf{p}_{E}$ describes its position relative to
the robot base. The function `DKM.m` returns $E_x$, $E_y$, and $E_z$, and
converts ${}^{0}\mathbf{R}_{E}$ into both roll-pitch-yaw and Euler-angle
representations.

## Inverse Kinematic Model (IKM)

The inverse kinematic model determines all valid joint configurations that
produce a desired end-effector pose:

```math
{}^{0}\mathbf{T}_{E,d}
=
\begin{bmatrix}
\mathbf{R}_{d}
&
\mathbf{p}_{d}
\\
\mathbf{0}^{T}
&
1
\end{bmatrix},
\qquad
\mathbf{p}_{d}
=
\begin{bmatrix}
E_x \\ E_y \\ E_z
\end{bmatrix}.
```

The desired rotation matrix $\mathbf{R}_{d}$ is constructed from the
requested Euler angles $(\Psi,\Theta,\Phi)$.

### 1. Remove the fixed tool transformation

The final three DH transformations describe the fixed end-effector and
tool geometry:

```math
\mathbf{T}_{wE}
=
{}^{7}\mathbf{T}_{8}
{}^{8}\mathbf{T}_{9}
{}^{9}\mathbf{T}_{E}.
```

The desired wrist pose is obtained by removing this fixed transformation:

```math
{}^{0}\mathbf{T}_{w}
=
{}^{0}\mathbf{T}_{E,d}
\mathbf{T}_{wE}^{-1}.
```

Let the wrist position be:

```math
\mathbf{p}_{w}
=
\begin{bmatrix}
P_x \\ P_y \\ P_z
\end{bmatrix}.
```

### 2. Solve for the first joint angle

The base joint has two possible solutions:

```math
\begin{aligned}
q_{1,a}
&=
\operatorname{atan2}(-P_y,-P_x),
\\
q_{1,b}
&=
\operatorname{wrapToPi}(q_{1,a}+\pi).
\end{aligned}
```

For each value of $q_1$, define the radial and vertical wrist coordinates:

```math
r
=
\begin{cases}
-P_x/\cos q_1,
&
\left|\cos q_1\right|>\left|\sin q_1\right|,
\\
-P_y/\sin q_1,
&
\text{otherwise},
\end{cases}
\qquad
z=P_z-L_1.
```

The conditional expression for $r$ avoids division by a trigonometric
term that is close to zero.

### 3. Solve for the third joint angle

Define:

```math
D
=
\frac{
r^2+z^2-L_2^2-L_3^2-L_4^2
}{
2L_2
},
\qquad
h
=
\sqrt{L_3^2+L_4^2},
\qquad
\rho
=
\operatorname{atan2}(L_4,L_3).
```

The elbow-up and elbow-down solutions are:

```math
\begin{aligned}
q_{3,a}
&=
\operatorname{wrapToPi}
\left(
\rho+\cos^{-1}\left(\frac{D}{h}\right)
\right),
\\
q_{3,b}
&=
\operatorname{wrapToPi}
\left(
\rho-\cos^{-1}\left(\frac{D}{h}\right)
\right).
\end{aligned}
```

A real arm solution exists only when:

```math
\left|\frac{D}{h}\right|\leq 1.
```

### 4. Solve for the second joint angle

For each solution of $q_3$, define:

```math
\begin{aligned}
a
&=
L_2+L_3\cos q_3+L_4\sin q_3,
\\
b
&=
L_3\sin q_3-L_4\cos q_3.
\end{aligned}
```

The following linear system is solved:

```math
\begin{bmatrix}
a & b \\
-b & a
\end{bmatrix}
\begin{bmatrix}
c_1 \\ c_2
\end{bmatrix}
=
\begin{bmatrix}
r \\ z
\end{bmatrix}.
```

The second joint angle is then:

```math
q_2
=
\operatorname{wrapToPi}
\left(
\operatorname{atan2}(c_1,c_2)
\right).
```

The two base solutions and two elbow solutions produce four possible arm
configurations.

### 5. Solve for the wrist angles

For each arm configuration, the transformation through the first three
joints and the fixed $L_4$ offset is:

```math
{}^{0}\mathbf{T}_{a}
=
{}^{0}\mathbf{T}_{1}
{}^{1}\mathbf{T}_{2}
{}^{2}\mathbf{T}_{3}
{}^{3}\mathbf{T}_{4}.
```

The required wrist orientation is extracted from:

```math
\mathbf{T}_{m}
=
\left({}^{0}\mathbf{T}_{a}\right)^{-1}
{}^{0}\mathbf{T}_{w},
\qquad
\mathbf{R}_{m}
=
\mathbf{T}_{m}(1\!:\!3,1\!:\!3).
```

Let $R_{ij}$ denote the element in row $i$ and column $j$ of
$\mathbf{R}_{m}$. The two wrist-flip solutions are:

```math
q_5
=
\pm\cos^{-1}(R_{33}).
```

For $\sin q_5\neq 0$, the remaining wrist angles are:

```math
q_4
=
\operatorname{atan2}
\left(
\frac{R_{23}}{\sin q_5},
\frac{R_{13}}{\sin q_5}
\right),
```

```math
q_6
=
\operatorname{atan2}
\left(
\frac{R_{32}}{\sin q_5},
-\frac{R_{31}}{\sin q_5}
\right).
```

### 6. Wrist singularity

A wrist singularity occurs when $|\sin q_5|$ is close to zero. In this
configuration, $q_4$ and $q_6$ cannot be determined independently. The
implementation selects:

```math
q_4=0,
\qquad
q_6=\operatorname{atan2}(R_{21},R_{11}).
```

### 7. Select valid solutions

The arm and wrist branches produce up to eight candidate joint
configurations. Each angle is wrapped to $[-\pi,\pi]$, and the candidates
are checked against the Niryo joint limits:

| Joint | Minimum | Maximum |
|:--:|--:|--:|
| $q_1$ | $-175^\circ$ | $175^\circ$ |
| $q_2$ | $-90^\circ$ | $36.7^\circ$ |
| $q_3$ | $-80^\circ$ | $90^\circ$ |
| $q_4$ | $-175^\circ$ | $175^\circ$ |
| $q_5$ | $-100^\circ$ | $110^\circ$ |
| $q_6$ | $-147.5^\circ$ | $147.5^\circ$ |

Only configurations that satisfy all six joint limits are returned by
`IKM.m`.
