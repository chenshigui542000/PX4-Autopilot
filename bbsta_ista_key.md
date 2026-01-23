# BBSTA ("The implicit discretization of the super-twisting sliding-mode control algorithm")

## ISTA / implicit STA discretization: key equations & algorithm (Codex-friendly)

Source: BBSTA.pdf, Section III (Implicit discretization).

---

## Variables
At discrete time `t_k` (sample index `k`):

- Known data at time `t_k`:
  - `x_{1,k}` (sliding variable / measured state used by the controller)
  - `ν_k` (internal controller state)
  - parameters: `h`, `λ1`, `λ2`

- Intermediate (virtual / nominal) variable computed implicitly:
  - `\tilde x_{1,k+1}`

- Selection variable:
  - `ξ_{k+1} ∈ sgn(\tilde x_{1,k+1})` with `sgn(0) = [-1,1]`

---

## Implicit discrete controller (paper Eq. (5), (7)–(10))

Virtual nominal plant used for the implicit step:

(7)
\[
\tilde x_{1,k+1} = x_{1,k} + h u_k
\]

Implicit controller law (5):
\[
\begin{cases}
 u_k = -\lambda_1 \sqrt{|\tilde x_{1,k+1}|}\,\mathrm{sgn}(\tilde x_{1,k+1}) + \nu_{k+1}\\
 \nu_{k+1} \in \nu_k - \lambda_2 h\,\mathrm{sgn}(\tilde x_{1,k+1})
\end{cases}
\]

Using `ξ_{k+1} ∈ sgn(\tilde x_{1,k+1})`, rewrite (10):
\[
\begin{cases}
 u_k = -\lambda_1 \sqrt{|\tilde x_{1,k+1}|}\,\xi_{k+1} + \nu_{k+1}\\
 \nu_{k+1} = \nu_k - \lambda_2 h\,\xi_{k+1}
\end{cases}
\]

Define (paper Eq. (11)–(13) helper quantities):
\[
 a = h\lambda_1 > 0,\qquad b_k = -x_{1,k} - h\nu_k
\]

The implicit step solves a generalized equation whose solution can be expressed by cases below.

---

## Closed-form case split (paper text below Fig. 2)

Let
\[
 b_k = -x_{1,k} - h\nu_k
\]

### Case 1
If
\[
 b_k < -h^2\lambda_2
\]
then
- `ξ_{k+1} = 1`
- `\tilde x_{1,k+1} > 0`

Compute
\[
 \sqrt{|\tilde x_{1,k+1}|} = \frac{-a + \sqrt{a^2 - 4\,(b_k + \lambda_2 h^2)}}{2}
\]
Update
\[
 \nu_{k+1} = \nu_k - h\lambda_2
\]
Control
\[
 u_k = -\lambda_1\sqrt{|\tilde x_{1,k+1}|} + \nu_{k+1}
\]

### Case 2
If
\[
 b_k \in [-h^2\lambda_2,\; h^2\lambda_2]
\]
then
- `\tilde x_{1,k+1} = 0`
- equivalently `ξ_{k+1} = -\dfrac{b_k}{h^2\lambda_2} ∈ [-1,1]`

Control (paper gives the simplest closed-form):
\[
 u_k = \nu_{k+1} = -\frac{x_{1,k}}{h}
\]

(And this implies `\nu_{k+1} = \nu_k - h\lambda_2 ξ_{k+1}`.)

### Case 3
If
\[
 b_k > h^2\lambda_2
\]
then
- `ξ_{k+1} = -1`
- `\tilde x_{1,k+1} < 0`

Compute
\[
 \sqrt{|\tilde x_{1,k+1}|} = \frac{-a + \sqrt{a^2 + 4\,(b_k - \lambda_2 h^2)}}{2}
\]
Update
\[
 \nu_{k+1} = \nu_k + h\lambda_2
\]
Control
\[
 u_k = \lambda_1\sqrt{|\tilde x_{1,k+1}|} + \nu_{k+1}
\]

---

## Implementation notes (for reviewers)

- The computed control `u_k` is **non-anticipative**: all terms depend only on `x_{1,k}`, `ν_k`, parameters, and the implicit solution determined at time `t_k`.
- Boundary behavior uses Case 2 when `b_k = ±h^2 λ2`.

