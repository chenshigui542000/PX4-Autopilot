# ISTA.cpp vs BBSTA Section III — quick notes for Codex

This file is optional context to speed up a review.

## Direct mapping (paper -> code)
- `x_{1,k}` -> function parameter `x` in `ISTA::update(float x, float h)`
- `h` -> function parameter `h` (a.k.a `dt`)
- `ν_k` -> member `_nu`
- `λ1` -> member `_lambda1`
- `λ2` -> member `_lambda2`
- `b_k = -x_{1,k} - h ν_k` -> `b = -x - h * _nu`
- `a = h λ1` -> `a = h * _lambda1`

## Case split
- Case 1: `b < -h*h*λ2` -> first `if`
- Case 2: `|b| <= h*h*λ2` -> `else if (fabsf(b) <= h*h*λ2)`
- Case 3: `b >  h*h*λ2` -> final `else`

## Closed-form sqrt term
- Case 1 uses: `sqrt(|\tilde x|) = (-a + sqrt(a^2 - 4(b + λ2 h^2)))/2`
- Case 3 uses: `sqrt(|\tilde x|) = (-a + sqrt(a^2 + 4(b - λ2 h^2)))/2`

## Known divergence from paper
In Case 2, BBSTA gives **exact** `u_k = ν_{k+1} = -x/h`.

Current code does:
```cpp
_nu = -x / h;
_nu = clamp(_nu, -nu_max, nu_max); // nu_max = 3
u = _nu;
```
The clamp/saturation is **not** in the paper and makes the implementation no longer strictly equivalent to BBSTA in Case 2 when `|x/h| > nu_max`.

If your goal is *paper-equivalent verification*, treat saturation as a feature flag and disable it for equivalence tests.
