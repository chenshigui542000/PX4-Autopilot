# Prompt for Codex (VSCode) — ISTA in PX4: paper-match review

You have access to this repository/workspace (PX4 + my ISTA implementation).

## Goal
Audit whether my implementation of the **implicit super-twisting (ISTA)** matches BBSTA Section III.

## References
- Paper excerpt (normalized): `bbsta_ista_key.md`
- My core implementation: `ISTA.hpp`, `ISTA.cpp`
- PX4 integration files: (search for where ISTA is used and list those files in your answer)

## Tasks
1) **Variable mapping table** between paper and code:
   - `x_{1,k}` -> ?
   - `ν_k` -> ?
   - `h` -> ? (where does dt come from?)
   - `λ1, λ2` -> ? (params)
   - `b_k, a` -> ?
   - `ξ_{k+1}` -> ?
   - `\tilde x_{1,k+1}` / `sqrt(|\tilde x|)` -> ?

2) **Case-by-case correctness**
   - Case 1: `b_k < -h^2 λ2`
   - Case 2: `|b_k| <= h^2 λ2`
   - Case 3: `b_k >  h^2 λ2`

   For each case, confirm:
   - the computed `sqrt(|\tilde x|)` formula
   - `ν_{k+1}` update
   - `u_k` formula
   - signs (note `ξ=±1`) and boundary points `b_k = ±h^2 λ2`

3) **Identify deviations from the paper**
   - e.g., any saturation/limit, filtering, different update order, different sign conventions, etc.
   - Explain the likely impact (paper-level guarantees vs engineering behavior).

4) **PX4 integration sanity checks**
   - Units/meaning of `x` (is it the same as paper’s `x_{1,k}`?)
   - Persistence/reset of `ν` across arming/disarming, mode changes, estimator resets.
   - Timing: ensure `h` equals the controller loop dt used where ISTA runs.

5) **Minimal verification tests**
   Provide 3 minimal numeric test vectors `(x, ν, h, λ1, λ2)` that hit:
   - Case 1, Case 2, Case 3
   and show expected `(u, ν_next)` from the formulas in `bbsta_ista_key.md`.

## Output format
- Start with a short verdict: **match / match with deviations / mismatch**.
- Then the mapping table.
- Then the case-by-case findings.
- Then deviations list.
- Then integration risks.
- Then the 3 numeric tests.

## What to do
1) **Build a symbol-to-code mapping table**
   - Map paper variables to code variables and units:
     - `x_{1,k}` -> ?
     - `ν_k` -> ?
     - `h` (dt) -> ?
     - `λ1`, `λ2` -> ?
     - `b_k = -x_{1,k} - h ν_k` -> code expression
     - `a = h λ1` -> code expression

2) **Case-by-case equivalence proof (or counterexample)**
   For each case in `bbsta_ista_key.md`:
   - identify the exact code branch (file + function + line range)
   - confirm the branch condition is equivalent (including boundaries)
   - confirm computed values match the paper formula:
     - `ξ_{k+1}`
     - `sqrt(|\tilde x_{1,k+1}|)` expression
     - `ν_{k+1}` update
     - `u_k` expression and whether it uses updated `ν_{k+1}`

3) **Check for non-paper modifications**
   - Identify any saturation/limits/filters/resets not present in paper.
   - For each, explain how it changes behavior (e.g., violates `u_k=-x/h` in Case 2).

4) **Boundary & numerical robustness**
   - Evaluate behavior at `b_k = ±h^2 λ2`.
   - Check discriminants under sqrt are non-negative for each branch.
   - Check potential `h=0` or very small `h` behavior.

5) **Provide minimal numeric spot-checks**
   Give at least 3 numeric examples (choose `h, λ1, λ2, x_{1,k}, ν_k`) that hit:
   - Case 1
   - Case 2
   - Case 3
   And compute expected `(u_k, ν_{k+1})` from `bbsta_ista_key.md`, then compare to code outputs.

6) **PX4 integration review (if present)**
   - Confirm `ν` state persistence and reset conditions (arming/disarming, mode switch).
   - Confirm `h` is the controller loop dt.
   - Confirm unit consistency for the variable used as `x_{1,k}`.

## Output format
- Start with a short verdict: **MATCH / PARTIAL MATCH / MISMATCH**
- Then provide:
  - Mapping table
  - Case 1/2/3 comparison sections
  - List of differences / risks
  - Suggested patches (minimal diffs)
  - Suggested unit tests / SITL checks
   - confirm the computed `ξ`, `ν_{k+1}`, `u_k`, and `sqrt(|\tilde x|)` match the formulas.
   - explicitly check boundary points `b_k = ± h^2 λ2` (which branch is taken, and whether output matches the paper).

3) **Identify non-paper modifications**
   - Flag anything not in the paper (e.g., saturation/limits on `ν` or `u`, filtering, different sign conventions, epsilons, float vs double choices).
   - For each modification, state the behavioral impact (e.g., changes the Case 2 closed form; introduces steady-state error; alters finite-step convergence claims).

4) **PX4 integration correctness**
   - verify `h` really equals the controller update period (dt) for the loop where ISTA runs.
   - verify `ν_k` state is persisted across iterations and properly reset on arming/disarming, mode changes, estimator resets, etc.
   - verify the physical meaning/units of `x_{1,k}` match the paper assumption (sliding variable) and that scaling is consistent.

5) **Add a minimal test harness (if missing)**
   - create a small unit test (or simple main) that feeds 3–5 deterministic test cases that cover:
     - Case 1, Case 2, Case 3
     - two boundary cases `b = ± h^2 λ2`
   - In the test, compute expected outputs using the formulas from `bbsta_ista_key.md` and compare with implementation.

## Output format
- A short summary (pass/fail + biggest risk)
- Mapping table
- Case-by-case checklist with references to file/line ranges
- Any recommended code changes as a patch/diff
- The test cases you propose (exact numbers) and what they should produce

   - For each modification, state whether it changes algorithm equivalence, and what behavior it can change (steady-state error, chattering, convergence, etc.).

4) **PX4 integration correctness checks**
   - Verify `h` equals the control loop `dt` actually used.
   - Verify state `ν_k` persists across timesteps and is reset appropriately (arming/disarming, mode switch, estimator reset, etc.).
   - Verify units for `x_{1,k}` and `u_k` are consistent with the actuator path.
   - Check thread-safety / reentrancy if controller can be called from multiple contexts.

5) **Provide tests**
   - Unit tests: 3 numeric vectors that hit Case1/2/3 and compute expected `u_k` and `ν_{k+1}` from formulas in `bbsta_ista_key.md`, comparing to code.
   - A boundary test for each of `b_k = ± h^2 λ2`.
   - Optional: random property test (generate random inputs, compute expected by formulas, assert close).

## Output format
- Start with the mapping table.
- Then a section per Case (1/2/3) with: formula -> code -> verdict.
- Then a section “Differences vs paper” (must be explicit).
- Then “PX4 integration risks” + recommended logging fields.
- Then tests (code snippets).
   - Verify any saturation/limits are consistent with actuator limits and do not break expected stability behavior.

5) **Produce a minimal numeric test suite**
   Create 3–6 deterministic tests that:
   - cover all three cases and both boundaries
   - compare the code output vs the closed-form formulas in `bbsta_ista_key.md`
   - include at least one test with large `|x/h|` to reveal any saturation.

## Output format
- A short summary (Pass / Needs changes)
- The mapping table
- A section per case (Case 1/2/3 + boundary)
- A list of deviations from paper + risk assessment
- Proposed patches (if needed)
- Proposed unit tests
- A short summary (pass/fail with reasons)
- The mapping table
- A case-by-case checklist with file/line references
- A list of divergences & their implications
- The proposed tests (code snippets or exact inputs/expected outputs)
