# Cubli: study guide and first math problem

## 1. What the ETH reference says

**Michael Muehlebach (2018), *On the Stabilization of Unstable Systems -
Some Comments and Applications*, ETH thesis no. 24789.**
[Local PDF](phdthesisE-Collection.pdf) |
[Official record](https://doi.org/10.3929/ethz-b-000254022).

The thesis has four parts: Cubli, a ducted-fan flying vehicle, constrained
LQR/MPC approximations, and distributed state estimation. **Read Part A first**;
the other parts are not prerequisites for building a balancing cube.
Page numbers below are printed thesis pages; PDF viewer pages are also given.

| Read | Main result | What it means for this project |
|---|---|---|
| P1, dynamics: pp. 25-28 (PDF 44-47) | A 150 mm cube with three perpendicular reaction wheels; gravity, inertia, and angular momentum determine its motion. | Motor torque accelerates a wheel and reacts on the frame. Wheel speeds are states, not just outputs. |
| P1, nonlinear control: pp. 28-37 (PDF 47-56) | Backstepping with a Lyapunov argument stabilizes the upright set almost everywhere in the ideal model. Feedback linearization supports trajectory tracking. | First understand stability proofs and the model; the result is not a guarantee under arbitrary motor limits or impacts. |
| P1, jump-up: pp. 37-45 (PDF 56-64) | Braking transfers wheel momentum to the cube. A gradient-based learning method adjusts initial wheel speeds; a guiding controller tracks the rise. | Jump-up is a separate hybrid maneuver, not ordinary balancing with larger gains. |
| P1, experiments: pp. 45-47 (PDF 64-66) | Reported steady-state RMS inclination error below 0.025 degrees; recovery in under 1.8 s after a 0.17 N m, 60 ms disturbance on one wheel. | These are results for ETH's apparatus, not performance promises for a printed prototype. |
| P2, estimation: pp. 51-78 (PDF 70-97) | Multiple accelerometers and a non-accelerated pivot assumption estimate tilt using rigid-body kinematics, without mass/inertia knowledge. | A moving accelerometer measures rotational acceleration as well as gravity; simply treating its reading as gravity can give wrong tilt. |

**Important limitations:** the reduced gravity-vector representation describes
tilt but discards yaw orientation. In the ideal frictionless-pivot model,
angular momentum about gravity is conserved: one cannot generally force both
frame yaw rate and all wheel speeds to zero. Ground friction changes this.
ETH's almost-global balancing result concerns an upright set, not arbitrary
full-attitude regulation with every wheel stopped.

**INDI is not the controller developed in these Cubli chapters.** It is our
proposed comparison. Start with their dynamics and a conventional baseline;
do not assume INDI will outperform it without experiments.

## 2. Nonlinear-control book

**Hassan K. Khalil, *Nonlinear Systems*, 3rd edition, Prentice Hall, 2002,
ISBN 9780130673893.**
[Author's companion site](https://www.egr.msu.edu/~khalil/NonlinearSystems/index.html) |
[Verified contents](https://www.egr.msu.edu/~khalil/NonlinearSystems/contents.html) |
[Publisher](https://www.pearson.com/en-us/subject-catalog/p/nonlinear-systems/P200000003306/9780130673893).
Obtain the book through a library or publisher; only its citation and links
are included here, not a copied textbook.

Suggested order: section 1.2.1 (pendulum), chapter 2 (phase portraits),
sections 4.1-4.3 (Lyapunov, invariance, linearization), section 12.2
(stabilization via linearization), chapter 13 (feedback linearization),
then section 14.3 (backstepping). Chapters 4 and 13 are the priority.

## 3. Similar concept: a reaction-wheel inverted pendulum

Think of one Cubli face on a fixed horizontal pivot, carrying one motor and
wheel. This has the same gravity-versus-reaction-torque mechanism, without
three-dimensional coupling. The following is an original educational
exercise, not a reproduction of a textbook problem or ETH's full model.

Assume a rigid body, fixed frictionless pivot, ideal motor torque, no impacts,
and no wheel/body drag. Let:

- $\theta$: body angle from upright; $\omega=\dot\theta$.
- $\phi$: wheel angle relative to the body; $s=\dot\theta+\dot\phi$ is its absolute speed.
- $I$: pivot inertia of the non-spinning assembly, including wheel-center
  translation but excluding the wheel's axial spin inertia $J$.
- $m$: total mass; $l$: pivot-to-total-center-of-mass distance.
- $\tau$: torque on the wheel; the reaction torque on the body is $-\tau$.

Use these **illustrative parameters, not a hardware shopping specification**:

$$
m=0.40\ {\rm kg},\quad l=0.08\ {\rm m},\quad
I=0.004\ {\rm kg\,m^2},\quad J=0.0002\ {\rm kg\,m^2},\quad g=9.81\ {\rm m/s^2}.
$$

### Problems to solve

1. Start with
   $L=\frac12 I\dot\theta^2+\frac12 J(\dot\theta+\dot\phi)^2-mgl\cos\theta$.
   The generalized forces for $(\theta,\phi)$ are $(0,\tau)$ because the
   motor is internal. Derive the two Euler-Lagrange equations and show
   $I\dot\omega=mgl\sin\theta-\tau$ and $J\dot s=\tau$.
2. Linearize at $(\theta,\omega,s,\tau)=(0,0,0,0)$. Write $A,B$ for
   $x=(\theta,\omega,s)$; show the upright angle is open-loop unstable.
3. For the **angle subsystem**, choose
   $\tau=mgl\sin\theta+I(k_p\theta+k_d\omega)$.
   Find the closed-loop equation and choose gains for a critically damped
   response with natural frequency $8\ {\rm rad/s}$.
4. Use $V=\frac12 k_p\theta^2+\frac12\omega^2$ to prove angle convergence.
   Explain why $\dot V\leq0$ alone is insufficient and use LaSalle's principle.
   Does this also prove $s\to0$?
5. Starting from $\theta=5$ degrees, $\omega=s=0$, calculate the initial torque.
   Simulate the nonlinear plant with $|\tau|\leq0.20\ {\rm N\,m}$ and a
   wheel-speed ceiling $|s|\leq300\ {\rm rad/s}$.
   Detect and stop/report a speed-limit violation; do not clip $s$, since that
   would introduce an unmodeled braking torque.
6. Write $\dot\omega=f(\theta)+G\tau$ and derive an INDI update for
   target acceleration $v=-k_p\theta-k_d\omega$, using a recent measured
   acceleration $\alpha_0$ and the corresponding applied torque $\tau_0$.
   Explain what fails if the acceleration measurement is delayed or noisy.

<details>
<summary>Checkpoints: open after attempting the problems</summary>

- For the Euler-Lagrange coordinates, $q_1=\theta$ (body angle) and
  $q_2=\phi$ (wheel angle relative to the body). Their generalized forces are
  $Q_1=Q_\theta=0$ and $Q_2=Q_\phi=\tau$: the motor applies $-\tau$ to the
  body and $+\tau$ to the wheel, so its virtual work is
  $-\tau\,\delta\theta+\tau\,\delta(\theta+\phi)=\tau\,\delta\phi$.
  Since
  $\partial L/\partial\dot\theta=I\dot\theta+J(\dot\theta+\dot\phi)$ and
  $\partial L/\partial\theta=mgl\sin\theta$, the $\theta$ equation is
  $(I+J)\ddot\theta+J\ddot\phi-mgl\sin\theta=0$, or
  $(I+J)\ddot\theta+J\ddot\phi=mgl\sin\theta$.
  The $\phi$ equation is
  $J(\ddot\theta+\ddot\phi)=\tau$, i.e. $J\dot s=\tau$.
  Substituting this into the $\theta$ equation gives
  $I\ddot\theta=mgl\sin\theta-\tau$, or $I\dot\omega=mgl\sin\theta-\tau$.
- The coupled equations are
  $(I+J)\ddot\theta+J\ddot\phi=mgl\sin\theta$ and
  $J(\ddot\theta+\ddot\phi)=\tau$.
- With $a=mgl/I=78.48\ {\rm s^{-2}}$:
  $A=\begin{bmatrix}0&1&0\\a&0&0\\0&0&0\end{bmatrix}$,
  $B=\begin{bmatrix}0\\-1/I\\1/J\end{bmatrix}$.
  The open-loop eigenvalues are $0$ and approximately $\pm8.86\ {\rm s^{-1}}$.
- Ideal cancellation gives $\ddot\theta+k_d\dot\theta+k_p\theta=0$;
  choose $k_p=64\ {\rm s^{-2}}$, $k_d=16\ {\rm s^{-1}}$.
- $\dot V=-k_d\omega^2$. The largest invariant set with $\omega=0$
  also has $\theta=0$, proving angle convergence in the ideal,
  unsaturated angle model. This is not a global physical Cubli proof.
- Wheel speed need not converge to zero: $s(t)=s(0)+J^{-1}\int_0^t\tau\,dt$.
  The angle-only controller leaves the wheel's final speed unconstrained.
  As an extension, use full-state LQR to include wheel-speed regulation.
- The initial torque is approximately $0.0497\ {\rm N\,m}$.
  Even when initial torque is feasible, check the entire trajectory's limits.
- $f=(mgl/I)\sin\theta$, $G=-1/I$.
  Locally, $\dot\omega\approx\alpha_0+G(\tau-\tau_0)$, hence
  $\tau=\tau_0+I(\alpha_0-v)$.
  This assumes gravity changes little over the increment and applied torque
  responds fast enough. Time-align/filter the torque and acceleration
  signals consistently and include actuator dynamics and saturation.

</details>
