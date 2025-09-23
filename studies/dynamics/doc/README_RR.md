# RR Kinematics

This is the Chemnitz revolute-revolute (RR) example that begins on slide 79.

<img src="image_rr.png">

The setup is:

* a revolute joint at the origin with angle $q_1$.
* a link of mass $m_1$ centered $l_{c_1}$ from the origin, with inertia $I_{c_1}$ and length $l_1$. 
* a revolute joint at the end of the first link, with relative angle $q_2$.
* a link of mass $m_2$ centered $l_{c_2}$ from the joint, with inertia $I_{c_2}$ and length $l_2$.

This setup involves the sum of angles, so
for things like $sin(q_1 + q_2)$, we use a shorthand like
$s_{12}$

## Mass

The center of mass of the first link is just a rotation:

$$
p_{c_1} =
\begin{bmatrix}
l_{c_1} c_1 \\[4pt]
l_{c_1} s_1\\[4pt]
0
\end{bmatrix}
$$

The center of mass of the second link is a rotation on top
of wherever the first link end is:

$$
p_{c_2} =
\begin{bmatrix}
l_{c_2} c_{12} + l_1 c_1\\[4pt]
l_{c_2} s_{12} + l_1 s_1\\[4pt]
0
\end{bmatrix}
$$

The translational Jacobian for the first link is the
straightforward derivative:

$$
J_{v_1} =
\begin{bmatrix}
-l_{c_1} s_1 & 0 \\[4pt]
l_{c_1} c_1 & 0 \\[4pt]
0 & 0
\end{bmatrix}
$$

And the same for the second link:

$$
J_{v_2} =
\begin{bmatrix}
-l_{c_2} s_{12} - l_1 s_1 & -l_{c_2}s_{12} \\[4pt]
l_{c_2} c_{12} + l_1 c_1 & l_{c_2}c_{12} \\[4pt]
0 & 0
\end{bmatrix}
$$

The angular Jacobians are very simple, just rigid rotation
of one joint:

$$
J_{\omega_1} =
\begin{bmatrix}
0 & 0 \\[4pt]
0 & 0 \\[4pt]
1 & 0
\end{bmatrix}
$$

and the sum of the two rotations:

$$
J_{\omega_2} =
\begin{bmatrix}
0 & 0 \\[4pt]
0 & 0 \\[4pt]
1 & 1
\end{bmatrix}
$$
