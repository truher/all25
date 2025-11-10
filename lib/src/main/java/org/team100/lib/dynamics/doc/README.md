# Dynamics

This is a study of second-order dynamics applied to three different
mechanisms:

* [Revolute (R)](README_R.md) -- a very simple single-jointed arm
* [Prismatic-Revolute (PR)](README_PR.md) -- an elevator an simple arm
* [Revolute-Revolute (RR)](README_RR.md) -- a two-jointed arm
* [Prismatic-Revolute-Revolute (PRR)](README_PRR.md) -- what we actually have


The reason to explore this topic is to improve the accuracy of the
feedforward terms in actuation, to include joint acceleration, centrifugal force,
and Coriolis force.

## Equation of Motion

The general equation of motion relates the generalized force (force or torque), $\tau$, to
the mechanism joint configuration, $q$, joint velocity $\dot{q}$, and joint acceleration, $\ddot{q}$:

```math
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)
```

* The first term, $M(q)\ddot{q}$, represents the inertia of the mechanism.  $M$ is called the "mass matrix."
* The second term, $C(q,\dot{q})$, represents the centrifugal and Coriolis forces.  $C$ is called the "Coriolis matrix."
* The third term, $G(q)$, represents the effect of gravity.

The derivation of this equation is beyond the scope of this analysis;
it comes from the Lagrange equation:

```math
\tau = {d \over dt}\left({\partial L \over \partial \dot{q}}\right)
- {\partial L \over \partial q}
```

For more about Lagrange methods, see [this reference](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/b39e882f1524a0f6a98553ee33ea6f35_MIT16_07F09_Lec20.pdf).

## Jacobians

The dynamics are all functions involving the __center-of-mass Jacobians__, which relate
joint velocities to the (absolute cartesian and rotational) velocities of the
centers of mass (COM) of each link:

```math
v_{c_i} = J_{v_i}\dot{q} \\
\omega_{c_i} = J_{\omega_i}\dot{q}
```

that is, the cartesian COM Jacobian for the $i$'th link is:

```math
J_{v_i} =
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
```

where $p_{C_i}$ is the position of the center of mass of each link.

So the procedure for deriving the Jacobians is to simply write the
expression for position, and then take the derivative with respect
to each config axis.

There are trailing zeros (to $n$) because the configuration of links beyond
$i$ don't affect the position of the $i$'th one, i.e. this is a "serial chain".

These Jacobians are different from the end-effector Jacobian we use for kinematics.

## Mass

The mass matrix is the sum of the translation and rotation terms:

```math
M = \sum\limits_{i=0}^{n}
(m_i J_{v_i}^TJ_{v_i}
+
J_{\omega_i}^T I_{C_i} J_{\omega_i} )
```

So, for example, for a 2-DOF system,

```math
M =
\begin{bmatrix}
m_{11} & m_{12}\\
m_{21} & m_{22}
\end{bmatrix}
```

The diagonal elements represent the inertia on each joint when the
other joint doesn't move.  The off-diagonal elements represent
the effect of the joints on each other.

## Centrifugal and Coriolis Forces

The elements of the Coriolis matrix are given:

```math
c_{ij} =
\sum\limits_{k=1}^{n} \Gamma_{ijk}(q)\dot{q_k}
```

where $\Gamma$ is the Christoffel symbol of the first kind:

```math
\Gamma_{ijk}(q) = {1\over2}
\left(
{\partial m_{ij} \over \partial q_k}
+
{\partial m_{ik} \over \partial q_j}
-
{\partial m_{jk} \over \partial q_i}
\right)
```

The derivation of Christoffel symbols is beyond the scope of this text.  For more about them, see [this reference](https://en.wikipedia.org/wiki/Christoffel_symbols#Christoffel_symbols_of_the_first_kind).



## Gravity

The gravity expression is the partial derivative of system
potential energy with respect to each joint:

```math
G_k(q) = {\partial P \over \partial q_k}
```

In a uniform gravitational field, the potential energy is
a vector quantity, $g$, and the potential energy is just
the dot product with COM displacement, $p$:

```math
P_i = m_i ( -g^T \cdot p_{c_i})
```

Since $m$ and $g$ are constant, we can write:

```math
G_k{q} =
- \sum\limits_{i=0}^{n}
(m_i g^T {\partial p_{c_i} \over \partial q{k}})
```

The latter expression, ${\partial p_{c_i} \over \partial q{k}}$, is 
exactly the Jacobian, so we can then rewrite:

```math
G = -
\begin{bmatrix}
m_1 g & m_2 g & \cdots & m_n g \\
\end{bmatrix}
\begin{bmatrix}
J_{v_1}^T\\
J_{v_2}^T\\
\vdots\\
J_{v_n}^T
\end{bmatrix}
```


## References

Most resources on second-order dynamics involve a lot of abstraction and not
a lot of examples, but I did find one reference that did include several examples:

https://www.tu-chemnitz.de/informatik/KI/edu/robotik/ws2017/Dyn.pdf

More references:

https://oramosp.epizy.com/teaching/18/robotics/lectures/Topic11_Dynamics_I.pdf


https://publish.illinois.edu/ece470-intro-robotics/files/2021/10/ECE470FA21Lec16.pdf