---
title: "Tutorial on Observability of Dynamical Systems"
permalink: /observability_tutorial/
---

# Observability of LTI Systems {#sec:observability}

Observability measures how well internal states can be inferred from external
outputs. A system is observable if the initial state $\mathbf{x}(0)$ can be
uniquely determined from the output $\mathbf{y}(t)$ and input $\mathbf{u}(t)$
measured over a finite time interval. In this section, we consider a
continuous-time linear time-invariant (LTI) system:

$$
    \begin{align}
        \dot{\mathbf{x}}(t) & = \boldsymbol{A}\mathbf{x}(t) + \boldsymbol{B}\mathbf{u}(t),\label{eq:ltiState} \\
        \mathbf{y}(t)       & = \boldsymbol{C}\mathbf{x}(t).\label{eq:ltiObservation}
    \end{align}
$$

where $\mathbf{x}(t) \in \mathbb{R}^m$, $\mathbf{u}(t) \in \mathbb{R}^n$, and
$\mathbf{y}(t) \in \mathbb{R}^p$ are the state, input, and output vectors,
respectively.

## The Observability Gramian

The fundamental tool for analyzing observability is the Observability Gramian
(OG). We derive the OG by developing the mathematical relationship between the
initial state and the output of the system, and examining whether this
relationship is injective --- that is, whether distinct initial states produce
distinct outputs.

The solution of the state equation, Eq. \eqref{eq:ltiState}, is given by:

$$
\begin{equation}
\label{eq:stateTransition}
    \mathbf{x}(t) = \boldsymbol{\Phi}_t \mathbf{x}(0) + \int_0^t \boldsymbol{\Phi}_{t-\tau} \boldsymbol{B} \mathbf{u}(\tau) d\tau,
\end{equation}
$$

where $\boldsymbol{\Phi}_t$ is the the state transition matrix, which satisfies
the matrix differential equation:
$$\dot{\boldsymbol{\Phi}}_t = \boldsymbol{A}\boldsymbol{\Phi}_t, \quad \boldsymbol{\Phi}_0 = \mathbf{I}.$$

Under zero input, the solution reduces to:

$$
\begin{equation}
\label{eq:zeroInputResponse}
    \mathbf{x}(t) = \boldsymbol{\Phi}_t \mathbf{x}(0),
\end{equation}
$$

which is known as the _zero-input response_:

Substituting Eq. \eqref{eq:zeroInputResponse} into
Eq. \eqref{eq:ltiObservation}, we obtain the _system response_, a mapping from
the initial state $\mathbf{x}(0)$ to the output $\mathbf{y}(t)$ through the
product $\boldsymbol{C}\boldsymbol{\Phi}_t$.

$$
\begin{equation}
\label{eq:systemResponse}
    \mathbf{y}(t) = \boldsymbol{C}\boldsymbol{\Phi}_t \mathbf{x}(0),
\end{equation}
$$

To assess observability, we examine whether $\mathbf{x}(0)$ can be uniquely
determined from $\mathbf{y}(t)$ over a time interval $[0, T]$. To quantify this
uniqueness, we use the $\ell^2$ inner product of two output signals:

$$
\begin{equation}
\label{eq:innerProduct}
    \langle \mathbf{y}_1, \mathbf{y}_2 \rangle = \int_0^T {\mathbf{y}_1(t)}^\top \mathbf{y}_2(t) dt,
\end{equation}
$$

which measures their similarity, in a manner analogous to the vector dot
product. Substituting two different system responses
$\mathbf{y}_i(t) = \boldsymbol{C}\boldsymbol{\Phi}_t \mathbf{x}_i(0)$ for
$i = 1, 2$ into Eq. \eqref{eq:innerProduct} gives:

$$
\begin{equation}
\langle \mathbf{y}_1, \mathbf{y}_2 \rangle = {\mathbf{x}_1(0)}^\top \left(\int_0^T \boldsymbol{\Phi}_t^\top \boldsymbol{C}^\top \boldsymbol{C} \boldsymbol{\Phi}_t dt\right) \mathbf{x}_2(0). \label{eq:innerProductWithGramian}
\end{equation}
$$

The associated symmetric matrix in Eq. \eqref{eq:innerProductWithGramian} is
known as the Observability Gramian (OG):

$$
\mathbf{W}(T) = \int_0^T \boldsymbol{\Phi}_t^\top \boldsymbol{C}^\top \boldsymbol{C} \boldsymbol{\Phi}_t dt,
$$

If the OG is positive-definite, then the system response is injective ---
distinct initial states produce distinct outputs, satisfying the definition of
observability. Since the OG is symmetric and positive-semidefinite by
construction, nonsingularity is equivalent to positive-definiteness. Therefore,
following Kailath [^0], an LTI system is observable if and only if
$\mathbf{W}(T)$ is nonsingular for some finite $T > 0$.

[^0]:
    T. Kailath, Linear systems. Prentice-Hall Englewood Cliffs, NJ, 1980, vol.
    156

### Intuition of unobservability

If the OG is singular, its null space contains a nonzero vector.

Let
$\mathbf{x}^\ast(0) \in \mathrm{ker}(\mathbf{W}(T)), \mathbf{x}^\ast(0) \neq 0$.
Then:

$$
\left\langle \mathbf{y}^\ast, \mathbf{y}^\ast \right\rangle = {\mathbf{x}^\ast(0)}^\top \mathbf{W}(T) \mathbf{x}^\ast(0) = 0,
$$

which implies that the output $\mathbf{y}^\ast(t)$ is identically zero and hence
indistinguishable from that of the zero state.

By the linearity of LTI systems, any initial state of the form
$\mathbf{x}^\ast(0) + \mathbf{x}^\dagger(0)$ will produce an output
$\mathbf{y}^\ast(t) + \mathbf{y}^\dagger(t) = \mathbf{y}^\dagger(t)$, which is
identical to the output from $\mathbf{x}^\dagger(0)$ alone. Hence,
$\mathbf{x}^\dagger(0)$ and $\mathbf{x}^\ast(0) + \mathbf{x}^\dagger(0)$ are
indistinguishable based solely on the output $\mathbf{y}(t)$, violating
observability.

## The Observability Matrix and the Rank Test

The OG provides a complete theoretical characterization of observability, but it
generally cannot be computed exactly in practice. Therefore, testing the OG's
positive-definiteness is not straightforward.

Fortunately, a simpler algebraic test exists. We derive this test by
investigating the null space of the OG. Consider the quadratic form with a test
vector $\mathbf{v}\in \mathbb{R}^m$:

$$
\mathbf{v}^\top \mathbf{W}(T) \mathbf{v} = \int_0^T \mathbf{v}^\top \boldsymbol{\Phi}_t^\top \boldsymbol{C}^\top \boldsymbol{C} \boldsymbol{\Phi}_t \mathbf{v} dt = \int_0^T \left\lVert{\boldsymbol{C} \boldsymbol{\Phi}_t \mathbf{v}}^2 \right\rVert dt.
$$

$\mathbf{v}$ is in the null space of $\mathbf{W}(T)$ if and only if the system
response $\boldsymbol{C} \boldsymbol{\Phi}_t \mathbf{v}$ is identically zero for
all $t\in [0, T]$:

$$
\mathrm{ker}(\mathbf{W}(T)) = \left\{\mathbf{v} \in \mathbb{R}^m : \boldsymbol{C} \boldsymbol{\Phi}_t \mathbf{v} = 0, \forall t \in [0, T]\right\},
$$

Since the system response is real-analytic in $t$, we can analyze it through its
Taylor series expansion about $t = 0$. For LTI systems, $\boldsymbol{\Phi}_t$
admits an analytical expression based on the matrix exponential:
$$\boldsymbol{\Phi}_t = \exp\left(\boldsymbol{A}t\right) = \sum_{k=0}^\infty \frac{\boldsymbol{M}^k}{k!}$$
Substituting this into the system response gives:
$$\boldsymbol{C} \boldsymbol{\Phi}_t \mathbf{v} = \sum_{k=0}^\infty \frac{t^k}{k!} \boldsymbol{C} \boldsymbol{A}^k \mathbf{v}.$$

An analytic function is identically zero if and only if all its Taylor
coefficients vanish. Therefore,
$\boldsymbol{C}\boldsymbol{\Phi}_t \mathbf{v} \equiv \mathbf{0}$ if and only if

$$
\boldsymbol{C} \boldsymbol{A}^k \mathbf{v} = \mathbf{0}\ \forall\ k = 0, 1, 2, \ldots
$$

The _Cayley-Hamilton theorem_ states that for $k \geq m$, the matrix powers
$\boldsymbol{A}^k$ are linearly dependent on
$\mathbf{1},\boldsymbol{A}^1,\ldots,\boldsymbol{A}^{m-1}$. Therefore, the null
space of the OG is characterized by a finite set of conditions:
$$\ker(\mathbf{W}(T)) = \left\{\mathbf{v} \in \mathbb{R}^m : \boldsymbol{C} \boldsymbol{A}^k \mathbf{v} = 0, k = [0, m-1]\right\}.$$

The conditions for $k = 0, 1, \ldots, m-1$ can be compactly written using the
_observability matrix_:

$$
\boldsymbol{\mathcal{O}} = \begin{bmatrix}
        \boldsymbol{C}                \\
        \boldsymbol{C} \boldsymbol{A} \\
        \vdots                        \\
        \boldsymbol{C} \boldsymbol{A}^{m-1}
    \end{bmatrix},
$$

which leads to the fundamental identity:

$$
\ker(\mathbf{W}(T)) = \ker(\boldsymbol{\mathcal{O}}).
$$

There, an LTI system is observable if and only if any of the following
equivalent conditions hold for any finite $T > 0$:

- $\mathbf{W}(T)$ is positive-definite,

- $\ker(\mathbf{W}(T)) = \ker(\boldsymbol{\mathcal{O}}) = \{\mathbf{0}\}$, and

- $\operatorname{rank}(\boldsymbol{\mathcal{O}}) = m$.

# Observability of nonlinear systems {#sec:nonlinearObservability}

In this section, we turn our attention to nonlinear systems of the form:

$$
\begin{align}
\dot{\mathbf{x}}(t) & = \mathbf{f}(\mathbf{x}(t),
\mathbf{u}(t)), \label{eq:systemDynamics} \\ \mathbf{y}(t) & =
\mathbf{h}(\mathbf{x}(t)),
\end{align}
$$

where $\mathbf{x}(t)\in \mathcal{X} \subseteq \mathbb{R}^m$,
$\mathbf{u}(t)\in \mathcal{U} \subseteq \mathbb{R}^n$, and
$\mathbf{y}(t)\in \mathcal{Y} \subseteq \mathbb{R}^p$ are the state, input, and
output vectors, respectively, and $\mathcal{X}, \mathcal{U}, \mathcal{Y}$ are
smooth manifolds. The _flow_ of $\mathbf{f}$, denoted by
$\phi : \mathcal{X} \times \mathcal{U} \to \mathcal{X}$, is a map that describes
the evolution of the state over time. It takes an initial state $\mathbf{x}_0$
an input function $\mathbf{u} : [0, t] \to \mathcal{U}$ to the state at time.
\begin{equation} \mathbf{x}(t) = \phi(t; \mathbf{x}\_0, \mathbf{u}(\cdot)).
\label{eq:flow} \end{equation}

By fixing $t$ and $\mathbf{u}(\cdot)$ in Eq. \eqref{eq:flow}, we obtain the
_flow map_ $\phi_t^{\mathbf{u}(\cdot)} : \mathcal{X}\to \mathcal{X}$, which maps
initial conditions (IC) to the corresponding state at time $t$ under the input
$\mathbf{u}(\cdot)$:

$$
\begin{equation}
\label{eq:flowMap} \phi_t^{\mathbf{u}(\cdot)}(\mathbf{x}_0) =
\phi(t; \mathbf{x}_0, \mathbf{u}(\cdot)).
\end{equation}
$$

This map emphasizes the dependence of a future state at time $t$ on the IC. For
brevity, we omit the explicit dependence on $\mathbf{u}(\cdot)$ and write
$\phi_t(\mathbf{x}_0)$ when the input is clear from context or its specific form
is not the focus of the discussion.

### Numerical solver integration

The dependence of the flow on the input $\mathbf{u}(\cdot)$ is crucial, though
it is sometimes notationally suppressed. This dependence becomes explicit in
numerical simulations. For example, `MATLAB`'s `ode45` requires the dynamics
function $\mathbf{f}$ to be parameterized by time $t$ to incorporate
time-varying inputs.

```
[t, x] = ode45(@(t, x) f(x, get_u(t)), [0, T], x0);
```

Here, `get_u` retrieves the input at time $t$, which could be sampled from a
pre-defined trajectory or computed by a feedback controller. This numerical
procedure highlights the fact that the flow is properly understood as
$\phi(t; \mathbf{x}_0, \mathbf{u}(\cdot))$, a functional of the input signal
over time.

As in the linear case, the goal is to determine whether the $\mathbf{x}_0$ can
be uniquely inferred from the input--output history
$(\mathbf{u}(\cdot), \mathbf{y}(\cdot))$ over some finite horizon.

## The Local Observability Gramian {#sub:log}

For a nonlinear system, we define the system response map to be:

$$
\mathcal{A}_t: \mathbf{x}_0 \mapsto \mathbf{y}(t) = \mathbf{h}(\phi_t(\mathbf{x}_0,\mathbf{u})),
$$

$\mathcal{A}\_t$ is an _operator_, meaning that it maps an IC to a function
composed by the output map $\mathbf{h}$ and $\phi_t$. It has no closed-form
expression in general, making it impossible to analyze the similarity of
long-term system responses directly. Therefore, observability analysis in
nonlinear systems focus on responses in a small neighborhood around the IC, and
the result is known as _local observability_.

Instead of $\mathcal{A}_t$, we focus on its linearized version $\mathscr{A}$,
which describes how small perturbations of the initial condition affect the
output trajectory. Formally:

$$
\mathscr{A} = D\mathcal{A} : T_{\mathbf{x}_0}\mathcal{X} \to
\mathcal{L}([0,T], \mathbb{R}^p),
$$

where the range is a function space of output trajectories.

To derive an expression for $\mathscr{A}$, we linearize the flow map $\phi\_t$
and the output map $\mathbf{h}$ separately, and then compose them. Let
$\delta\mathbf{x}_0 \in T\_{\mathbf{x}\_0}\mathcal{X}$ be a small perturbation
of the initial condition ($T\_{\mathbf{x}\_0}\mathcal{X} \cong \mathbb{R}^m$
denotes the tangent space of $\mathcal{X}$ at $\mathbf{x}_0$). Using
Eq. \eqref{eq:flowMap} , we obtain the perturbed trajectory:

$$
\begin{equation}
\label{eq:perturbedTrajectory} \delta\mathbf{x}(t) =
D\phi_t(\mathbf{x}_0)\,\delta\mathbf{x}_0.
\end{equation}
$$

Differentiating Eq. \eqref{eq:perturbedTrajectory} with respect to time gives
the variational equation:

$$
\begin{equation}
\label{eq:variationalEquation} \dot{\delta\mathbf{x}}(t) =
\frac{\mathrm{d}}{\mathrm{d}t}D(\phi*t(\mathbf{x}_0))\,\delta\mathbf{x}_0 =
D\mathbf{f}(\phi*{t}(\mathbf{x}_0))\,\delta\mathbf{x}_0 = D
\mathbf{f}(\phi_{t}(\mathbf{x}_0)) D\phi_{t}(\mathbf{x}_0)\,\delta\mathbf{x}_0
= \mathbf{F}(t) \delta\mathbf{x}(t),
\end{equation}
$$

where $\mathbf{F}(t) = D_{\mathbf{x}} \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t))$
is the Jacobian of the system dynamics in Eq. \eqref{eq:systemDynamics} ,
serving as the linearized system matrix The solution of
Eq. \eqref{eq:variationalEquation} can be expressed as:

$$
\begin{equation}
\label{eq:variationalSolution} \delta\mathbf{x}(t) =
\boldsymbol{\Phi}_t\delta\mathbf{x}_0,
\end{equation}
$$

where $\boldsymbol{\Phi}_t$ is the state transition matrix, satisfying the
matrix ODE:

$$
\dot{\boldsymbol{\Phi}}(t) = \mathbf{F}(t)\,\boldsymbol{\Phi}_t, \qquad
\boldsymbol{\Phi}(0) = \mathbf{1}.
$$

While Eq. \eqref{eq:variationalSolution} gives the linearization of the flow
map, the linearization of the output map is given by the standard linearized
observation matrix: $$\mathbf{H}(t) = Dh(\mathbf{x}(t)),$$ satisfying
$\delta\mathbf{y}(t) = \mathbf{H}(t)\,\delta\mathbf{x}(t)$. Finally, the
linearized system response is obtained by composing $\mathbf{H}$ with
$\boldsymbol{\Phi}$:

$$
\begin{equation}
\mathscr{A}(t) = \mathbf{H}(t)\,\boldsymbol{\Phi}_t, \label{eq:variationalResponse}
\end{equation}
$$

and it acts on a perturbation of the initial condition by
$\delta\mathbf{x}_0 \;\mapsto\; \delta\mathbf{y}(t) = \mathbf{H}(t)\,\boldsymbol{\Phi}_t\,\delta\mathbf{x}_0$.

Mirroring the linear case, $\mathscr{A}$ can be used to quantify whether
$\delta \mathbf{x}\_0$ can be uniquely determined from $\delta \mathbf{y}(t)$
over a time internal $[0, T]$. By analogous reasoning based on inner products of
two output _perturbations_, we arrive at the definition of the Local
Observability Gramian (LOG):

$$
\begin{equation}
\mathbf{W}(\mathbf{x}_0, \mathbf{u}, T) = \int_0^T \boldsymbol{\Phi}_t^\top
{\mathbf{H}(t)}^\top \mathbf{H}(t) \boldsymbol{\Phi}_t\,\mathrm{d}t.
\label{eq:LOG}
\end{equation}
$$

Following the same logic as in the LTI case, the positive-definiteness of the
LOG is a sufficient condition for the nonlinear system to be _locally
observable_ at $\mathbf{x}_0$ under the input $\mathbf{u}(\cdot)$. This means
that for the specific input signal used, the initial state $\mathbf{x}_0$ can be
distinguished from its neighbors based on the output on $[0, T]$.

## The Nonlinear Observability Rank Test

The LOG provides a complete theoretical characterization of observability of a
system around a specific state and input trajectory, but it generally cannot be
computed at all, as $\boldsymbol{\Phi}_t$ is generally not available in closed
form. Nevertheless, the LOG is the basis for quantitative measurement of
observability, and frameworks to numerically approximate the LOG will be
introduced in Sec. [3](#sec:measuresOfObservability).

A more practical, qualitative approach examines the inherent ability of the
nonlinear system itself to distinguish states, regardless of a particular input
trajectory. This leads to the concept of _local weak observability_, proposed by
Hermann and Krener [^1] for autonomous systems of the form:

$$
\begin{align} \dot{\mathbf{x}} & = \mathbf{f}(\mathbf{x}), \\ \mathbf{y} & =
\mathbf{h}(\mathbf{x}), \end{align}
$$

[^1]:
    R. Hermann and A. Krener, “Nonlinear controllability and observability,”
    IEEE Transactions on automatic control, vol. 22, no. 5, pp. 728–740, 2003

In this setting, local weak observability at $\mathbf{x}\_0$ is determined by an
_observability codistribution_ spanned by the differentials of the Lie
derivatives of $\mathbf{h}$ along the $\mathbf{f}$. Formally, the Lie derivative
of a scalar function $g : \mathcal{X} \to \mathbb{R}$ along a vector field
$\mathbf{f} : \mathcal{X} \to T\mathcal{X}$ is defined recursively as a new
scalar function:

$$
\begin{aligned} L_{\mathbf{f}}^0 g & = g, \\ L_{\mathbf{f}}^r g & =
L*{\mathbf{f}} (L_{\mathbf{f}}^{r-1} g) = D(L_{\mathbf{f}}^{r-1} g) \cdot
\mathbf{f}, \quad \text{for } r \geq 1. \end{aligned}
$$

The differentials that form the codistribution are then evaluated at the point
of interest, e.g., $D(L_{\mathbf{f}}^r \mathbf{h})(\mathbf{x}_0)$.

With this notation, the observability codistribution is:

$$
d\mathscr{O}(\mathbf{x}) =
\mathop{\mathrm{span}}\left\{DL^{(r)}_{\mathbf{f}}\mathbf{h}(\mathbf{x}) : r =
0, 1, 2, \ldots\right\}.
$$

and the system is locally weakly observable at $\mathbf{x}_0$ if $d\mathscr{O}$
has full rank $m$.

This autonomous formulation highlights the structural observability properties
of the system, abstracting away from explicit control inputs. However, most
engineering systems are controlled, and in practice the structure of the control
inputs matters. A particularly important subclass is that of _control-affine_
systems, where the dynamics depend linearly on the inputs:

$$
\dot{\mathbf{x}} = \mathbf{f}_0(\mathbf{x}) + \sum_{i=1}^n \mathbf{f}_i(\mathbf{x}) u_i,
$$

For control-affine systems, the control inputs
$\mathbf{u} = \begin{bmatrix}u_1 & u_2 & \cdots & u_m\end{bmatrix}^\top$
contribute to the observability codistribution through the control vector fields
$\mathbf{f}_i$, in addition to the drift vector field $\mathbf{f}_0$. In this
case, the observability codistribution can be generated by Lie derivatives along
this finite family of vector fields according to:

$$
d\mathscr{O}(\mathbf{x}) = \mathop{\mathrm{span}}\{ D(L_{\mathbf{v}_1}
L_{\mathbf{v}_2} \cdots L_{\mathbf{v}_k} \mathbf{h})(\mathbf{x}) \mid k \geq 0;
\mathbf{v}_q \in \{ \mathbf{f}_0, \mathbf{f}_1, \ldots, \mathbf{f}_n \} \},
$$

where $L\_{\mathbf{v}\_1} L\_{\mathbf{v}_2} \cdots L\_{\mathbf{v}_k} \mathbf{h}$
denotes a mixed Lie derivative of total order $k$, taken along the sequence of
vector fields $\mathbf{v}_1, \mathbf{v}_2, \ldots, \mathbf{v}_k$ each taken from
$\{ \mathbf{f}_0, \mathbf{f}_1, \ldots, \mathbf{f}_m \}$.

These differentials are stacked-rowwise, forming the _nonlinear observability
matrix_. The rows correspond to all Lie derivatives up to a certain order, taken
along all possible sequences of the vector fields:

$$
\begin{equation}
\label{eq:nonlinearObservabilityMatrix} \boldsymbol{\mathcal{O}}(\mathbf{x}) =
\begin{bmatrix} D\mathbf{h}(\mathbf{x}) \\ D(L_{\mathbf{f}_0}
\mathbf{h})(\mathbf{x}) \\ \vdots \\ D(L_{\mathbf{f}_{n}}
\mathbf{h})(\mathbf{x}) \\ D(L_{\mathbf{f}_0}^2 \mathbf{h})(\mathbf{x}) \\
D(L_{\mathbf{f}_0}L_{\mathbf{f}_1} \mathbf{h})(\mathbf{x}) \\ \vdots
\end{bmatrix}.
\end{equation}
$$

The system is locally weakly observable at $\mathbf{x}_0$ if
$\operatorname{rank}(\boldsymbol{\mathcal{O}}(\mathbf{x}_0)) = m$. In practice,
the matrix is constructed incrementally: Lie derivatives are taken along various
combinations of drift and control vector fields, up to some order, until either
the rank condition is satisfied or no new independent rows can be generated.
Illustrative examples of this procedure can be found
in [@martinelli2005observability; @trawny2010interrobot].

### Exercise 1

Apply the nonlinear observability rank test to the LTI system in
Eqs. \eqref{eq:ltiState} and \eqref{eq:ltiObservation}, and show that
$\boldsymbol{\mathcal{O}}^{(r)}$ reduces to the linear observability matrix
$\boldsymbol{\mathcal{O}}$ when $r = m-1$. :::

### Exercise 2

Consider a unicycle robot whose state is
$\mathbf{x} = \begin{bmatrix}x& y& \theta\end{bmatrix}^\top$ and control input
is $\mathbf{u} = \begin{bmatrix}v & \omega\end{bmatrix}^\top$, where $(x, y)$ is
the position of the robot in the plane, $\theta$ is its heading angle, and $v$
and $\omega$ are the linear and angular velocities, respectively. This robot
measures its distance to a fixed beacon at the origin.

$$
\begin{align} \dot{\mathbf{x}} & = \mathbf{f}(\mathbf{x}, \mathbf{u}) \triangleq
\begin{bmatrix} v \cos\theta \\ v \sin\theta \\ \omega \end{bmatrix}, \\ y & =
h(\mathbf{x}) \triangleq \sqrt{x^2 + y^2}, \end{align}
$$

Conduct the nonlinear observability rank test at an arbitrary state
$\mathbf{x}_0$ and derive the conditions under which the system fails to be
locally weakly observable.

## Measures of Observability {#sec:measuresOfObservability}

The preceding sections have established a binary notion of observability: a
linear time-invariant (LTI) system is either globally observable or not, while a
nonlinear system may be locally weakly observable in some regions of its state
space and unobservable in others. This is useful for validating system designs
and sensor configurations.

However, recent works on *perception-aware control* [^2] revealed a need for
motion strategies that bring about ideal conditions for sensing and perception.
While optimizing sensor-specific operating conditions (e.g., maximizing feature
visibility in vision-based control) is effective, a more general approach is to
optimize the system's _observability_ itself.

[^2]:
    D. Falanga et al., “Pampc: Perception-aware model predictive control for
    quadrotors,” in 2018 IEEE/RSJ International Conference on Intelligent Robots
    and Systems (IROS), IEEE, 2018, pp. 1–8

To this end, the LOG becomes a crucial tool for measuring observability
quantitatively. The LOG's positive-definiteness is a sufficient condition for
local observability, but its eigenvalues $\lambda_i, i \in [0, m]$ also provide
a measure of the _quality of observability_ along the directions of their
corresponding eigenvectors in the state space. A large eigenvalue indicates a
highly observable direction, while a small eigenvalue indicates a weakly
observable one. This allows us to define scalar metrics $\psi(\mathbf{W})$ that
aggregate the eigenvalues into a single number quantifying the overall
observability quality. Common choices for $\psi$ include:

1.  The trace, equivalent to the sum of eigenvalues:

    $$
    \psi_{\operatorname{tr}}(\mathbf{W}) = \operatorname{tr}{\mathbf{W}} = \sum_{i=1}^m \lambda_i,
    $$

    which measures the overall observability quality. Maximizing
    $\psi_{\operatorname{tr}}$ improves average observability, but may neglect
    weak directions.

2.  The determinant, equivalent to the product of eigenvalues:

    $$
        \psi_{\det}(\mathbf{W}) = \det{\mathbf{W}} = \prod_{i=1}^m \lambda_i,
    $$

    which measures the volume of the observability ellipsoid. Maximizing
    $\psi_{\det}$ balances improving all directions.

3.  The minimum eigenvalue:

    $$
        \psi_{\min}(\mathbf{W}) = \lambda_{\min}(\mathbf{W}),
    $$

    which measures the worst-case observability direction. Maximizing
    $\psi_{\min}$ ensures no direction is excessively unobservable.

### Optimality metrics

In the adjacent area of optimal experiment design [@pukelsheim2006optimal], the
trace, determinant, and minimum eigenvalue criteria correspond to the A-, D-,
and E-optimality criteria, respectively. This terminology are often seen in
information- or uncertainty-aware optimization literature.

## Approximation of the LOG

Once it is established that metrics of the LOG can be used to quantify
observability, the next challenge is to compute or approximate the LOG itself.

Krener and Ide [^3] proposed the _Empirical Local Observability Gramian_ (ELOG),
a numerical method to approximate the LOG. the ELOG empirically evaluates the
sensitivity of outputs to perturbations of the initial condition along each
direction using finite differences. Its definition is:

$$
\begin{gathered} \mathbf{W}_E(T) : \mathbf{W}_{E, ij} =
\frac{1}{4\epsilon^2}\int_0^T{(\mathbf{y}^{+i} -
\mathbf{y}^{-i})}^\top(\mathbf{y}^{+j} - \mathbf{y}^{-j})dt, \end{gathered}
$$

where
$\mathbf{y}^{\pm i}(t) = \mathbf{h}(\boldsymbol{\phi}_t(\mathbf{x}_0\pm\epsilon\mathbf{1}_i))$.

[^3]:
    A. J. Krener and K. Ide, “Measures of unobservability,” in Proceedings of
    the 48h IEEE Conference on Decision and Control (CDC) held jointly with 2009
    28th Chinese Control Conference, IEEE, 2009, pp. 6401–6406

This approach is general and straightforward to implement, as it only requires
the ability to simulate the system dynamics and evaluate the output map. It can
potentially be applied to any system, including those with complex, black-box
dynamics and observation models. This comes at the cost of computational
efficiency, as it requires $2m$ simulations of the system dynamics over the time
horizon $[0, T]$, to compute the ELOG at a single state $\mathbf{x}_0$. It also
inherits the numerical sensitivity and ill-conditioning of finite-difference
methods, and introduces an additional hyperparameter $\epsilon$ that must be
tuned.

Haussman [^4] proposed the _Expanded Empirical Local Observability Gramian_
(E^2^LOG), an alternative approximation of the LOG based on analytical
approximations. The core idea is to approximate the variational system response
$\mathscr{A}(t) = \mathbf{H}(t)\boldsymbol{\Phi}_t$ (from which we derived the
LOG in Sec. [2.1](#sub:log)) using a truncated series expansion based on Lie
derivatives.

[^4]:
    K. Hausman et al., “Observability-aware trajectory optimization for self-
    calibration with application to uavs,” IEEE Robotics and Automation Letters,
    vol. 2, no. 3, pp. 1770–1777, 2017

The ELOG is constructed by first building a finite-dimensional approximation of
the nonlinear observability matrix $\boldsymbol{\mathcal{O}}^{(r)}(\mathbf{x})$
up to some order $r$. This observability matrix is a _distinct_ variant of
Eq. \eqref{eq:nonlinearObservabilityMatrix} , and is defined as:

$$
\boldsymbol{\mathcal{O}}^{(r)}(\mathbf{x}) = \begin{bmatrix}
D\mathbf{h}(\mathbf{x}) \\ D(L_{\mathbf{f}} \mathbf{h})(\mathbf{x}) \\
D(L_{\mathbf{f}}^2 \mathbf{h})(\mathbf{x}) \\ \vdots \\ D(L_{\mathbf{f}}^r
\mathbf{h})(\mathbf{x}) \end{bmatrix},
$$

without requiring the system to be factored into a control-affine form. The
E^2^LOG is then defined as:

$$
\mathbf{W}_{E^2} = {\left(\boldsymbol{\mathcal{O}}^{(r)}\right)}^\top
\boldsymbol{\Lambda} \,
\boldsymbol{\mathcal{O}}^{(r)},
$$

where $\boldsymbol{\Lambda}$ is a weighting matrix with blockwise entries

$$
\boldsymbol{\Lambda}_{ij}
= \frac{T^{i+j+1}}{(i+j+1)!i!j!}\mathbf{1},\qquad i, j = 1, \ldots, r,
$$

which accounts for the time horizon and the scaling of the different derivative
orders in the series expansion. This formulation effectively approximates the
integral in the LOG definition by an algebraic computation on the Lie
derivatives, bypassing the need for numerical simulation of perturbed
trajectories.

This approach is often more efficient than the ELOG, as it only requires the
computation of Lie derivatives of the output map along the vector fields, which
can frequently be obtained analytically or via symbolic computation. However,
its accuracy depends critically on the choice of the truncation order $r$ and
the weights $\boldsymbol{\Lambda}$, and it is most effective for systems with
simple polynomial or trigonometric nonlinearities.

https://arxiv.org/abs/1604.07905
