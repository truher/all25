# PRR Dynamics

This covers a three-jointed, prismatic-revolute-revolute (PRR) mechanism.

<img src="image_prr.png">

The setup is:

* a prismatic joint at the origin, parameterized by $q_1$, the distance to the second joint.
* a link of mass $m_1$ centered $l_{c_1}$ from the far end (so that $l_{c_1}$ doesn't depend on $q_1$), with inertia $I_{c_1}$.
* a revolute joint parameterized by $q_2$, the angle measured from the x axis.
* a link of mass $m_2$ centered at $l_{c_2}$ from the joint, with inertia $I_{c_2}$ and length $l_2$.
* a revolute joint parameterized by $q_3$, the angle relative to the second link.
* a link of mass $m_3$ centered at $l_{c_3}$ from the joint, with inertia $I_{c_3}$ and lengh $l_3$.

Remember that expressions like $c_{23}$ mean $cosine(q_2+q_3)$

## Mass

Start with the positions of the centers of mass, $p_{c_1}$, $p_{c_2}$, and $p_{c_3}$.

The first link is just like the PR example:

$$
p_{c_1} =
\begin{bmatrix}
q_1 - l \\[4pt]
0\\[4pt]
0
\end{bmatrix}
$$

The second link is also identical to the PR example (with different notation):

$$
p_{c_2} =
\begin{bmatrix}
q_1 + l_{c_2} c_2\\[4pt]
l_{c_2} s_2\\[4pt]
0
\end{bmatrix}
$$

The third link is similar to the second link in the RR example:

$$
p_{c_3} =
\begin{bmatrix}
q_1  + l_2 c_2 + l_{c_3} c_{23} \\[4pt]
l_2 s_2 + l_{c_3} s_{23} \\[4pt]
0
\end{bmatrix}
$$

The translational center-of-mass Jacobians are as follows:

The first link depends only on the first joint:

$$
J_{v_1} =
\begin{bmatrix}
1 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
$$

The second link depends translationally on the first joint
and rotationally on the second joint:

$$
J_{v_2} =
\begin{bmatrix}
1 & -l_{c_2} s_2 & 0 \\[4pt]
0 & l_{c_2} c_2 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
$$

And finally the third link:

$$
J_{v_3} =
\begin{bmatrix}
1 & -l_2 s_2 - l_{c_3} s_{23} & -l_{c_3}s_{23} \\[4pt]
0 & l_2 c_2 + l_{c_3} c_{23}  & l_{c_3}c_{23} \\[4pt]
0 & 0 & 0
\end{bmatrix}
$$

And the angular center-of-mass Jacobians: all rotations are around Z,
so there are only values in the last row.

The first rotational Jacobian is always zero since the first joint is prismatic.

$$
J_{\omega_1} =
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
$$

The second rotation Jacobian is identical to the second PR link:

$$
J_{\omega_2} =
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 1 & 0 
\end{bmatrix}
$$

The last rotation sums the two joints:

$$
J_{\omega_3} =
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 1 & 1
\end{bmatrix}
$$

Now we can put all these terms together into the expression for the mass matrix:

$$
M =
m_1 J_{v_1}^T J_{v_1}
+
J_{\omega_1}^T I_{c_1} J_{\omega_1}
+
m_2 J_{v_2}^T J_{v_2}
+
J_{\omega_2}^T  I_{c_2} J_{\omega_2}
+
m_3 J_{v_3}^T J_{v_3}
+
J_{\omega_3}^T  I_{c_3} J_{\omega_3}
$$

To obtain:

$$
M =
m_1
\begin{bmatrix}
1 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\
+
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
I_{c1}
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\
+
m_2
\begin{bmatrix}
1 & 0 & 0 \\[4pt]
-l_{c_2} s_2  & l_{c_2} c_2 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
1 & -l_{c_2} s_2 & 0 \\[4pt]
0 & l_{c_2} c_2 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\
+
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 1 \\[4pt]
0 & 0 & 0 
\end{bmatrix}
\begin{bmatrix}
I_{xx_2} & -I_{xy_2} & -I_{xz_2} \\[4pt]
-I_{yx_2} & I_{yy_2} & -I_{yz_2} \\[4pt]
-I_{zx_2} & -I_{zy_2} & I_{zz_2} \\[4pt]
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 1 & 0 
\end{bmatrix}
\\
+
m_3
\begin{bmatrix}
1 & 0 & 0 \\[4pt]
-l_2 s_2 - l_{c_3} s_{23} & l_2 c_2 + l_{c_3} c_{23}  & 0 \\[4pt]
-l_{c_3}s_{23} & l_{c_3}c_{23} & 0
\end{bmatrix}
\begin{bmatrix}
1 & -l_2 s_2 - l_{c_3} s_{23} & -l_{c_3}s_{23} \\[4pt]
0 & l_2 c_2 + l_{c_3} c_{23}  & l_{c_3}c_{23} \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\
+
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 1 \\[4pt]
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
I_{xx_3} & -I_{xy_3} & -I_{xz_3} \\[4pt]
-I_{yx_3} & I_{yy_3} & -I_{yz_3} \\[4pt]
-I_{zx_3} & -I_{zy_3} & I_{zz_3} \\[4pt]
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 1 & 1
\end{bmatrix}
$$

Which is:

$$
M =
m_1
\begin{bmatrix}
1 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\
+
m_2
\begin{bmatrix}
1 & -l_{c_2}s_2 & 0 \\[4pt]
-l_{c_2}s_2 & l_{c_2}^2 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & I_{zz_2} & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\
+
m_3
\begin{bmatrix}
1 & -l_2 s_2 - l_{c_3} s_{23}  & -l_{c_3}s_{23} \\[4pt]
-l_2 s_2 - l_{c_3} s_{23} & l_2^2 + 2l_2l_{c_3}c_3 + l_{c_3}^2 & l_{c_3}^2 + l_2l_{c_3}c_3 \\[4pt]
-l_{c_3}s_{23} & l_{c_3}^2 + l_2l_{c_3}c_3 & l_{c_3}^2
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 & 0 \\[4pt]
0 & I_{zz_3} & I_{zz_3} \\[4pt]
0 & I_{zz_3} & I_{zz_3}
\end{bmatrix}
$$

The $m_3$ component makes use of the identity,
$$
cos(a)cos(b)+sin(a)sin(b) = cos(a-b)
$$
 
to simplify
$$
c_2c_{23} + s_2s_{23} = c_3
$$

Simplifying a bit further:

$$
M = 
\begin{bmatrix}
m_1 & 0 & 0 \\[4pt]
0 & 0 & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
+
\begin{bmatrix}
m_2 & -m_2l_{c_2}s_2 & 0 \\[4pt]
-m_2l_{c_2}s_2 & m_2l_{c_2}^2 +  I_{zz_2} & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}
\\[15pt]
+
\begin{bmatrix}
m_3 & -m_3l_2 s_2 - m_3l_{c_3} s_{23}  & -m_3l_{c_3}s_{23} \\[4pt]
-m_3l_2 s_2 - m_3l_{c_3} s_{23} & m_3l_2^2 + m_32l_2l_{c_3}c_3 + m_3l_{c_3}^2 + I_{zz_3}& m_3l_{c_3}^2 + m_3l_2l_{c_3}c_3 + I_{zz_3}\\[4pt]
-m_3l_{c_3}s_{23} & m_3l_{c_3}^2 + m_3l_2l_{c_3}c_3 + I_{zz_3} & m_3l_{c_3}^2 + I_{zz_3}
\end{bmatrix}
$$

And then finally, the terms of the mass matrix, written out individually so it's
easier to read:

$
m_{11} = m_1 + m_2 + m_3
$

$
m_{12} = -m_2l_{c_2}s_2 -m_3l_2 s_2 - m_3l_{c_3} s_{23} 
$

$
m_{13} = -m_3l_{c_3}s_{23} 
$

$
m_{21} = -m_2l_{c_2}s_2 -m_3l_2 s_2 - m_3l_{c_3} s_{23} = m_{12}
$

$
m_{22} = m_2l_{c_2}^2 +  I_{zz_2} + m_3l_2^2 + m_32l_2l_{c_3}c_3 + m_3l_{c_3}^2 + I_{zz_3}
$

$
m_{23} = m_3l_{c_3}^2 + m_3l_2l_{c_3}c_3 + I_{zz_3}
$

$
m_{31} = -m_3l_{c_3}s_{23} = m_{13}
$

$
m_{32} = m_3l_{c_3}^2 + m_3l_2l_{c_3}c_3 + I_{zz_3} = m_{23}
$

$
m_{33} = m_3l_{c_3}^2 + I_{zz_3}
$

## Centrifugal and Coriolis

There are 27 Christoffel symbols (!), but they are not all unique.  Also,
the partial derivatives with respect to $q_1$ is always zero.

$m_{11}$ is constant:
$$
\Gamma_{111} = {1\over2}
\left( {\partial m_{11} \over \partial q_1}
+ {\partial m_{11} \over \partial q_1}
- {\partial m_{11} \over \partial q_1} \right)
= {1\over2}  {\partial m_{11} \over \partial q_1}
= 0
$$
$$
\Gamma_{112} = {1\over2}
\left( {\partial m_{11} \over \partial q_2}
+ {\partial m_{12} \over \partial q_1}
- {\partial m_{12} \over \partial q_1} \right)
= {1\over2}{\partial m_{11} \over \partial q_2} 
= 0
$$
$$
\Gamma_{113} = {1\over2}
\left( {\partial m_{11} \over \partial q_3}
+ {\partial m_{13} \over \partial q_1}
- {\partial m_{13} \over \partial q_1} \right) 
= {1\over2} {\partial m_{11} \over \partial q_3}
= 0
$$

$m_{12}$
$$
\Gamma_{121} = {1\over2}
\left( {\partial m_{12} \over \partial q_1}
+ {\partial m_{11} \over \partial q_2}
- {\partial m_{21} \over \partial q_1} \right)
= {1\over2}{\partial m_{11} \over \partial q_2}
= \Gamma_{112} 
= 0
$$
$$
\Gamma_{122} = {1\over2}
\left( {\partial m_{12} \over \partial q_2}
+ {\partial m_{12} \over \partial q_2}
- {\partial m_{22} \over \partial q_1} \right)
= {\partial m_{12} \over \partial q_2}
= -m_2l_{c_2}c_2 - m_3l_2c_2 - m_3l_{c_3}c_{23}
$$
$$
\Gamma_{123} = {1\over2}
\left( {\partial m_{12} \over \partial q_3}
+ {\partial m_{13} \over \partial q_2}
- {\partial m_{23} \over \partial q_1} \right)
= {1\over2} \left( {\partial m_{12} \over \partial q_3}
+ {\partial m_{13} \over \partial q_2} \right)
$$

$m_{13}$
$$
\Gamma_{131} = {1\over2}
\left( {\partial m_{13} \over \partial q_1}
+ {\partial m_{11} \over \partial q_3}
- {\partial m_{31} \over \partial q_1} \right)
= {1\over2} {\partial m_{11} \over \partial q_3}
= \Gamma_{113} 
= 0
$$
$$
\Gamma_{132} = {1\over2}
\left( {\partial m_{13} \over \partial q_2}
+ {\partial m_{12} \over \partial q_3}
- {\partial m_{32} \over \partial q_1} \right)
= {1\over2}
\left( {\partial m_{13} \over \partial q_2}
+ {\partial m_{12} \over \partial q_3} \right)
$$
$$
\Gamma_{133} = {1\over2}
\left( {\partial m_{13} \over \partial q_3}
+ {\partial m_{13} \over \partial q_3}
- {\partial m_{33} \over \partial q_1} \right)
= 
{\partial m_{13} \over \partial q_3}
$$

$m_{21}$
$$
\Gamma_{211} = {1\over2}
\left( {\partial m_{21} \over \partial q_1}
+ {\partial m_{21} \over \partial q_1}
- {\partial m_{11} \over \partial q_2} \right)
= 
- {1\over2} {\partial m_{11} \over \partial q_2}
$$
$$
\Gamma_{212} = {1\over2}
\left( {\partial m_{21} \over \partial q_2}
+ {\partial m_{22} \over \partial q_1}
- {\partial m_{12} \over \partial q_2} \right)
= 0
$$
$$
\Gamma_{213} = {1\over2}
\left( {\partial m_{21} \over \partial q_3}
+ {\partial m_{23} \over \partial q_1}
- {\partial m_{13} \over \partial q_2} \right)
= {1\over2} \left( {\partial m_{21} \over \partial q_3}
- {\partial m_{13} \over \partial q_2} \right)
$$

$m_{22}$
$$
\Gamma_{221} = {1\over2}
\left( {\partial m_{22} \over \partial q_1}
+ {\partial m_{21} \over \partial q_2}
- {\partial m_{21} \over \partial q_2} \right)
= 0
$$
$$
\Gamma_{222} = {1\over2}
\left( {\partial m_{22} \over \partial q_2}
+ {\partial m_{22} \over \partial q_2}
- {\partial m_{22} \over \partial q_2} \right)
= {1\over2} {\partial m_{22} \over \partial q_2}
= 
$$
$$
\Gamma_{223} = {1\over2}
\left( {\partial m_{22} \over \partial q_3}
+ {\partial m_{23} \over \partial q_2}
- {\partial m_{23} \over \partial q_2} \right)
= {1\over2} {\partial m_{22} \over \partial q_3}
= 
$$

$m_{23}$
$$
\Gamma_{231} = {1\over2}
\left( {\partial m_{23} \over \partial q_1}
+ {\partial m_{21} \over \partial q_3}
- {\partial m_{31} \over \partial q_2} \right)
= 
{1\over2}
\left( {\partial m_{21} \over \partial q_3}
- {\partial m_{31} \over \partial q_2} \right)
$$
$$
\Gamma_{232} = {1\over2}
\left( {\partial m_{23} \over \partial q_2}
+ {\partial m_{22} \over \partial q_3}
- {\partial m_{32} \over \partial q_2} \right)
= {1\over2} {\partial m_{22} \over \partial q_3}
= \Gamma_{223} 
$$
$$
\Gamma_{233} = {1\over2}
\left( {\partial m_{23} \over \partial q_3}
+ {\partial m_{23} \over \partial q_3}
- {\partial m_{33} \over \partial q_2} \right)
= 
{\partial m_{23} \over \partial q_3}
- {1\over2}{\partial m_{33} \over \partial q_2}
$$

$m_{31}$
$$
\Gamma_{311} = {1\over2}
\left( {\partial m_{31} \over \partial q_1}
+ {\partial m_{31} \over \partial q_1}
- {\partial m_{11} \over \partial q_3} \right)
=
 - {1\over2} {\partial m_{11} \over \partial q_3}
$$
$$
\Gamma_{312} = {1\over2}
\left( {\partial m_{31} \over \partial q_2}
+ {\partial m_{32} \over \partial q_1}
- {\partial m_{12} \over \partial q_3} \right)
=
{1\over2}\left( {\partial m_{31} \over \partial q_2}
- {\partial m_{12} \over \partial q_3} \right)
$$
$$
\Gamma_{313} = {1\over2}
\left( {\partial m_{31} \over \partial q_3}
+ {\partial m_{33} \over \partial q_1}
- {\partial m_{13} \over \partial q_3} \right)
= 0
$$

$m_{32}$
$$
\Gamma_{321} = {1\over2}
\left( {\partial m_{32} \over \partial q_1}
+ {\partial m_{31} \over \partial q_2}
- {\partial m_{21} \over \partial q_3} \right)
= {1\over2} \left( {\partial m_{31} \over \partial q_2}
- {\partial m_{21} \over \partial q_3} \right)
$$
$$
\Gamma_{322} = {1\over2}
\left( {\partial m_{32} \over \partial q_2}
+ {\partial m_{32} \over \partial q_2}
- {\partial m_{22} \over \partial q_3} \right)
= 
{\partial m_{32} \over \partial q_2}
- {1\over2}{\partial m_{22} \over \partial q_3}
$$
$$
\Gamma_{323} = {1\over2}
\left( {\partial m_{32} \over \partial q_3}
+ {\partial m_{33} \over \partial q_2}
- {\partial m_{23} \over \partial q_3} \right)
= {1\over2} {\partial m_{33} \over \partial q_2}
= 
$$

$m_{33}$
$$
\Gamma_{331} = {1\over2}
\left( {\partial m_{33} \over \partial q_1}
+ {\partial m_{31} \over \partial q_3}
- {\partial m_{31} \over \partial q_3} \right)
= 0
$$
$$
\Gamma_{332} = {1\over2}
\left( {\partial m_{33} \over \partial q_2}
+ {\partial m_{32} \over \partial q_3}
- {\partial m_{32} \over \partial q_3} \right)
= {1\over2} {\partial m_{33} \over \partial q_2} 
= \Gamma_{323}
$$
$$
\Gamma_{333} = {1\over2}
\left( {\partial m_{33} \over \partial q_3}
+ {\partial m_{33} \over \partial q_3}
- {\partial m_{33} \over \partial q_3} \right)
= {1\over2} {\partial m_{33} \over \partial q_3}
= 
$$



## Gravity


## Equation of Motion