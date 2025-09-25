# PR Dynamics

This is the Chemnitz prismatic-revolute (PR) example that begins on slide 72.

<img src="image_pr.png">

The setup is:

* a prismatic joint at the origin, parameterized by $q_1$, the distance to the second joint.
* a link of mass $m_1$ centered $l$ from the far end (so that $l$ doesn't depend on $q_1$), with inertia $I_{c1}$.
* a revolute joint parameterized by $q_2$, the angle measured from the x axis.
* a link of mass $m_2$ centered at $d$ from the joint, with intertia $I_{c2}$.

The important results are as follows.  Remember that
joint configurations are $q_1$ and $q_2$ and
things like $s_2$ really mean $sin(q_2)$.


## Mass

For the PR case, let's start with the locations of the centers of mass.  We're using
three dimensions here even though this is a 2d drawing, just because the real thing
has 3d aspects which appear in the moment of inertia matrix -- for rotations in the
x-y plane, we'll use the "z" component of inertia.

So the center of mass of the first link is:

```math
p_{c_1} =
\begin{bmatrix}
q_1 - l \\
0\\
0
\end{bmatrix}
```

The center of mass of the second link is:

```math
p_{c_2} =
\begin{bmatrix}
q_1 + d c_2\\
d s_2\\
0
\end{bmatrix}
```

We can write the Jacobians by inspection.  The translational velocity of the center of mass
of the first link depends only on the first joint, $q_1$, so it has only one
non-zero value:

```math
J_{v_1} =
\begin{bmatrix}
1 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}
```

The translational velocity of the center of mass of the second link depends on both joints,
translation in the first column (from $q_1$) and rotation in the second (from $q_2$):

```math
J_{v_2} =
\begin{bmatrix}
1 & -ds_2 \\
0 & dc_2 \\
0 & 0
\end{bmatrix}
```

The angular velocity of the center of mass of the first link is always zero, because
the first joint is prismatic:

```math
J_{\omega_1} =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}
```

The angular velocity of the center of mass of the second link is always the same
as the angular velocity of the joint itself:

```math
J_{\omega_2} =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 1
\end{bmatrix}
```

Now we can put all these terms together into the expression for the mass matrix:

```math
M =
m_1 J_{v_1}^T J_{v_1}
+
J_{\omega_1}^T I_{c_1} J_{\omega_1}
+
m_2 J_{v_2}^T J_{v_2}
+
J_{\omega_2}^T  I_{c_2} J_{\omega_2}
```

Which is:

```math
M =
m_1
\begin{bmatrix}
1 & 0 & 0\\
0 & 0 & 0\\
\end{bmatrix}
\begin{bmatrix}
1 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}
\\
+
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
\end{bmatrix}
\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz} \\
-I_{yx} & I_{yy} & -I_{yz} \\
-I_{zx} & -I_{zy} & I_{zz} \\
\end{bmatrix}
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}
\\
+
m_2
\begin{bmatrix}
1 & 0 & 0\\
-ds_2 & dc_2 & 0\\
\end{bmatrix}
\begin{bmatrix}
1 & -ds_2 \\
0 & dc_2 \\
0 & 0
\end{bmatrix}
\\
+
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 1\\
\end{bmatrix}
\begin{bmatrix}
I_{xx_2} & -I_{xy_2} & -I_{xz_2} \\
-I_{yx_2} & I_{yy_2} & -I_{yz_2} \\
-I_{zx_2} & -I_{zy_2} & I_{zz_2} \\
\end{bmatrix}
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 1
\end{bmatrix}
```

Which is:

```math
M =
m_1
\begin{bmatrix}
1 & 0 \\
0 & 0
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 \\
0 & 0
\end{bmatrix}
+
m_2
\begin{bmatrix}
1 & - d s_2 \\
- d s_2 & d^2
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 \\
0 & I_{zz_2}
\end{bmatrix}
```

Which gives us the mass matrix:

```math
M =
\begin{bmatrix}
m_1+m_2 & -m_2ds_2 \\
-m_2ds_2 & I_{zz2}+m_2d^2
\end{bmatrix}
```

To interpret the mass matrix, think of how it is used:

```math
\tau = M(q)\ddot{q}
```

So the prismatic joint force "feels" the
usual $ma$ term for the total mass, minus a term
proportional to the revolute joint acceleration,
and orientation (maximum when
bent 90 degrees).  Intuitively, if the revolute
joint acceleration is opposing the prismatic one,
the force felt by the prismatic joint is less.
Similarly, the revolute joint feels the acceleration
of the pivot by the prismatic joint.


## Centrifugal and Coriolis

To derive the centrifugal/Coriolis term, start by writing out
the Christoffel symbol permutations:

```math
\Gamma_{111} = {1\over2}
\left( {\partial m_{11} \over \partial q_1}
+ {\partial m_{11} \over \partial q_1}
- {\partial m_{11} \over \partial q_1} \right) = 0
```

```math
\Gamma_{112} = {1\over2}
\left( {\partial m_{11} \over \partial q_2}
+ {\partial m_{12} \over \partial q_1}
- {\partial m_{12} \over \partial q_1} \right) = 0
```

```math
\Gamma_{121} = {1\over2}
\left( {\partial m_{12} \over \partial q_1}
+ {\partial m_{11} \over \partial q_2}
- {\partial m_{21} \over \partial q_1} \right) = 0
```

```math
\Gamma_{122} = {1\over2}
\left( {\partial m_{12} \over \partial q_2}
+ {\partial m_{12} \over \partial q_2}
- {\partial m_{22} \over \partial q_1} \right) = -m_2dc_2
```

```math
\Gamma_{211} = {1\over2}
\left( {\partial m_{21} \over \partial q_1}
+ {\partial m_{21} \over \partial q_1}
- {\partial m_{11} \over \partial q_2} \right) = 0
```

```math
\Gamma_{212} = {1\over2}
\left( {\partial m_{21} \over \partial q_2}
+ {\partial m_{22} \over \partial q_1}
- {\partial m_{12} \over \partial q_2} \right) = 0
```

```math
\Gamma_{221} = {1\over2}
\left( {\partial m_{22} \over \partial q_1}
+ {\partial m_{21} \over \partial q_2}
- {\partial m_{21} \over \partial q_2} \right) = 0
```

```math
\Gamma_{222} = {1\over2}
\left( {\partial m_{22} \over \partial q_2}
+ {\partial m_{22} \over \partial q_2}
- {\partial m_{22} \over \partial q_2} \right) = 0
```

Then we can write the $C$ matrix:

```math
C = 
\begin{bmatrix}
0 & -m_2dc_2\dot{q_2}\\
0 & 0
\end{bmatrix}
```


In the PR case, there is no coriolis term, and the
centrifugal term affects only the P joint, proportional
to its position (greatest at full extension).


## Gravity

The reference shows this as the gravity vector:

```math
G = 
\begin{bmatrix}
0 \\ 
dc_2m_2g
\end{bmatrix}
```

But this is the wrong orientation of gravity, along the $y$ axis.

```math
g =
\begin{bmatrix}
0 \\ -g \\ 0
\end{bmatrix}
```

For our orientation, the correct gravity is along the $x$ axis:

```math
g =
\begin{bmatrix}
-g \\ 0 \\ 0
\end{bmatrix}
```

And so the gravity vector is

```math
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
```
So
```math
G =
\begin{bmatrix}
(m_1+m_2)g \\
-ds_2m_2g
\end{bmatrix}
```

... so this represents the total mass pushing
down on the prismatic joint, and the arm mass pushing
on the revolute joint.




## Equation of Motion

Finally (using our corrected gravity expression),
we can write the equation of motion, starting with the definition:

```math
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)
```

and substituting the matrices we computed above:

```math
\tau = 
\begin{bmatrix}
m_1+m_2 & -m_2ds_2 \\
-m_2ds_2 & I_{zz2}+m_2d^2
\end{bmatrix}
\ddot{q}
+
\begin{bmatrix}
0 & -m_2dc_2\dot{q_2}\\
0 & 0
\end{bmatrix}
\dot{q}
+
\begin{bmatrix}
(m_1+m_2)g \\
-ds_2m_2g
\end{bmatrix}
```


So the scalar expressions for each component are as follows:

P joint force, $f_1$:

```math
f_1 =
(m_1+m_2)\ddot{q}_1
- m_2ds_2\ddot{q}_2
- m_2dc_2\dot{q}_2^2
+ (m_1+m_2)g
```

R joint torque, $\tau_2$:

```math
\tau_2 =
-m_2ds_2\ddot{q}_1
+ (I_{zz2} + m_2d^2)\ddot{q}_2
- ds_2m_2g
```

