# PR Kinematics

This is a study of second-order dynamics applied to the prismatic-revolute (PR)
mechanism.  The reason to explore this topic is to improve the accuracy of the
feedforward terms in actuation, to include joint acceleration, centrifugal force,
and Coriolis force.

The actual Calgames arm is not PR, it is PRR, so this study could be considered
a first step towards what we really want.

Most resources on second-order dynamics involve a lot of abstraction and not
a lot of examples, but I did find one reference that did include several examples:

https://www.tu-chemnitz.de/informatik/KI/edu/robotik/ws2017/Dyn.pdf

Let's work through the PR example that begins on slide 72.

<img src="image.png">

The setup is:

* a prismatic link at the origin, parameterized by $q_1$, the distance to the second link.
* a link of mass $m_1$ centered $l$ from the far end (so that $l$ doesn't depend on $q_1$), with inertia $I_{c1}$.
* a revolute joint parameterized by $q_2$, the angle measured from the x axis
* a link of mass $m_2$ centered at $d$ from the joint, with intertia $I_{c2}$.

The important results are as follows (remember that
joint configurations are $q_1$ and $q_2$ and
things like $s_2$ really mean $sin(q_2)$):

## Mass

The mass matrix is:

$$
M =
\begin{bmatrix}
m_1+m_2 & -m_2ds_2 \\[4pt]
-m_2ds_2 & I_{zz2}+m_2d^2
\end{bmatrix}
$$

To interpret the mass matrix, think of how it is used:

$$
\tau = M(q)\ddot{q}
$$

So the prismatic joint force "feels" the
usual $ma$ term for the total mass, minus a term
proportional to the revolute joint acceleration,
and orientation (maximum when
bent 90 degrees).  Intuitively, if the revolute
joint acceleration is opposing the prismatic one,
the force felt by the prismatic joint is less.
Similarly, the revolute joint feels the acceleration
of the pivot by the prismatic joint.

Note the derivation of $M$ in the reference uses a "Jacobian" concept
which is different from the joint Jacobian we use
elsewhere: this is the Jacobian __of the centers of masses__.

## Gravity

The reference shows this as the gravity vector:

$$
G = 
\begin{bmatrix}
0 \\[4pt] 
dc_2m_2g
\end{bmatrix}
$$

But this is the wrong orientation of gravity, along the $y$ axis.

$$
g =
\begin{bmatrix}
0 \\ -g \\ 0
\end{bmatrix}
$$

For our orientation, the correct gravity is along the $x$ axis:

$$
g =
\begin{bmatrix}
-g \\ 0 \\ 0
\end{bmatrix}
$$

And so the gravity vector is

$$
G = -
\begin{bmatrix}
1 & 0 \\
0 & 0 \\
0&  0
\end{bmatrix}^T
\begin{bmatrix}
-m_1g \\
0 \\
0
\end{bmatrix}
-
\begin{bmatrix}
1 & -ds_2 \\
0 & dc_2 \\
0 & 0
\end{bmatrix}^T

\begin{bmatrix}
-m_2g \\
0 \\
0
\end{bmatrix}
$$
So
$$
G =
\begin{bmatrix}
(m_1+m_2)g \\[4pt]
-ds_2m_2g
\end{bmatrix}
$$
... so this represents the total mass pushing
down on the prismatic joint, and the arm mass pushing
on the revolute joint.

## Centrifugal and Coriolis

The combined vector $V$ is given as:

$$
V(q,\dot{q}) = 
\begin{bmatrix}
0\\[4pt]
0
\end{bmatrix}
\begin{bmatrix}
\dot{q}_1 & \dot{q}_2
\end{bmatrix}
+
\begin{bmatrix}
0 & -m_2dc_2 \\[4pt]
0 & 0
\end{bmatrix}
\begin{bmatrix}
\dot{q}_1^2 \\[4pt]
\dot{q}_2^2
\end{bmatrix}
$$

The first term is the coriolis force: it
appears *across* radial motion.  The second term
is centrifugal force: it appears *along* the radius
of motion.

In the PR case, there is no coriolis term, and the
centrifugal term affects only the P joint, proportional
to its position (greatest at full extension).

## Equation of Motion

Finally (using our corrected gravity expression),
we can write the expressions for the P joint
force, $f_1$:

$$
f_1 =
(m_1+m_2)\ddot{q}_1
- m_2ds_2\ddot{q}_2
- m_2dc_2\dot{q}_2^2
+ (m_1+m_2)g
$$

and the R joint torque, $\tau_2$:
$$
\tau_2 =
-m_2ds_2\ddot{q}_1
+ (I_{zz2} + m_2d^2)\ddot{q}_2
- ds_2m_2g
$$

