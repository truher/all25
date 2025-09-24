# Dynamics

This is a study of second-order dynamics applied to three different
mechanisms:

* [Prismatic-Revolute {PR}](README_PR.md) -- an elevator an simple arm
* [Revolute-Revolute {RR}](README_RR.md) -- a two-jointed arm
* [Prismatic-Revolute-Revolute {PRR}](README_PRR.md) -- what we actually have


The reason to explore this topic is to improve the accuracy of the
feedforward terms in actuation, to include joint acceleration, centrifugal force,
and Coriolis force.

Most resources on second-order dynamics involve a lot of abstraction and not
a lot of examples, but I did find one reference that did include several examples:

https://www.tu-chemnitz.de/informatik/KI/edu/robotik/ws2017/Dyn.pdf

This reference is also pretty good:

https://oramosp.epizy.com/teaching/18/robotics/lectures/Topic11_Dynamics_I.pdf

## Equation of Motion

The general equation of motion relates the generalized force (force or torque), $\tau$, to
the mechanism joint configuration, $q$, joint velocity $\dot{q}$, and joint acceleration, $\ddot{q}$:

$$
\tau = M(q)\ddot{q} + V(q,\dot{q}) + G(q)
$$

* The first term, $M(q)\ddot{q}$, represents the inertia of the mechanism.
* The second term, $V(q,\dot{q})$, represents the centrifugal and coriolis forces.
* The third term, $G(q)$, represents the effect of gravity.

## Jacobians

The dynamics are all functions involving the __center-of-mass Jacobians__, which relate
joint velocities to the (absolute cartesian and rotational) velocities of the
centers of mass (COM) of each link:

$$
v_{c_i} = J_{v_i}\dot{q} \\
\omega_{c_i} = J_{\omega_i}\dot{q}
$$

that is, the cartesian COM Jacobian for the $i$'th link is:

$$
J_{v_i}
=
\begin{bmatrix}
\partial p_{C_i} \over \partial q_1
&
\partial p_{C_i} \over \partial q_2
&
\partial p_{C_i} \over \partial q_i
&
...
&
0
&
...
&
0
\end{bmatrix}
$$

where p_{C_i} is the position of the center of mass of each link.

There are trailing zeros (to $n$) because the configuration of links beyond
$i$ don't affect the position of the $i$'th one, i.e. this is a "serial chain".

These Jacobians are different from the end-effector Jacobian we use for kinematics.

## Mass Matrix in General

The mass matrix is the sum of the translation and rotation terms:

$$
M = \sum\limits_{i=0}^{n}
(m_i J_{v_i}^TJ_{v_i}
+
J_{\omega_i}^T I_{C_i} J_{\omega_i} )
$$

## Centrifugal and Coriolis in General

TODO: expand this section.

$$
V = \dot{M}\dot{q} - {1\over2}
\begin{bmatrix}
\dot{q}^T{\partial M \over \partial q_1}\dot{q}\\[4pt]
\vdots\\[4pt]
\dot{q}^T{\partial M \over \partial q_n}\dot{q}\\[4pt]
\end{bmatrix}
$$


## Gravity in General

The gravity expression is the partial derivative of system
potential energy with respect to each joint:

$$
G_k(q) = {\partial P \over \partial q_k}
$$

In a uniform gravitational field, the potential energy is
a vector quantity, $g$, and the potential energy is just
the dot product with COM displacement, $p$:

$$
P_i = m_i ( -g^T \cdot p_{c_i})
$$

Since $m$ and $g$ are constant, we can write:

$$
G_k{q} =
- \sum\limits_{i=0}^{n}
(m_i g^T {\partial p_{c_i} \over \partial q{k}})
$$

The latter expression, ${\partial p_{c_i} \over \partial q{k}}$, is 
exactly the Jacobian, so we can then rewrite:

$$

G = -
\begin{bmatrix}
m_1 g & m_2 g & \cdots m_n g \\
\end{bmatrix}
\begin{bmatrix}
J_{v_1}^T\\
J_{v_2}^T\\
\vdots\\
J_{v_n}^T
\end{bmatrix}
$$