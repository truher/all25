# Intercept

How to hit a moving target from a moving shooter platform.

In the global reference frame, the situation looks like this:

<img src="global_diagram.png" width=600 />

The robot position over time is $R_G(t) = R_{0_G} + v_{R_G} t$. 

The target position over time is $T_G(t) = T_{0_G} + v_{T_G} t$.

The math is easier if we use the robot reference frame:

<img src="local_diagram.png" width=600 />

The robot's position is fixed at the origin.

The target's initial position is $T_0 = T_{0_G} - R_{0_G}$.

The target velocity in the robot frame is $v_T = (v_{T_G} - v_{R_G})$

The target position over time in the robot frame is $T(t) = T_0 + v_T t$.

The projectile muzzle speed is always the same, $s_m$.

We don't know what the intersection position is, but we do know
its distance from the origin: $s_m t$.  When intersection occurs at $t$, the
target should be that far from the origin:

```math
s_m t = \left\| T_0 + v_T t \right\|
```

Square both sides:

```math
s_m^2 t^2 = \left\| T_0 + v_T t \right\|^2
```

The squared magnitude is the self-dot-product, so expanding:

```math
s_m^2 t^2 =  T_0 \cdot T_0 + 2(T_0 \cdot v_T) t + (v_T \cdot v_T) t^2
```

This is quadratic in $t$:

```math
(v_T \cdot v_T - s_m^2) t^2  + 2(T_0 \cdot v_T) t +   T_0 \cdot T_0 = 0
```

There are three possible solutions:

* two solutions: target can be reached on approach, or later while receding
* one solution: target can just barely be reached
* no solution: target is moving away faster than the projectile can catch it



## Resources
Some resources about this problem.

* https://indyandyjones.wordpress.com/2010/04/08/intercepting-a-target-with-projectile/
* https://playtechs.blogspot.com/2007/04/aiming-at-moving-target.html
* https://stackoverflow.com/questions/17204513/how-to-find-the-interception-coordinates-of-a-moving-target-in-3d-space