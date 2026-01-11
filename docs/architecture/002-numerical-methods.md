# ADR-002: Numerical Root-Finding Methods

**Status:** Accepted

## Context

Skillshot interception requires solving:

```
|D + V·t| = s·(t - d) + r
```

where `t` (intercept time) is the unknown. This is a nonlinear equation that may have 0, 1, or 2 solutions.

## Decision

Use a **three-stage root-finding approach** with MathNet.Numerics:

1. **Quadratic Approximation**: Initial estimate via closed-form solution (fastest, ~80% of cases)
2. **Brent's Method**: Bracketed root-finding for robust convergence (handles edge cases)
3. **Newton-Raphson with Bisection fallback**: Final refinement to machine precision

## Implementation

See `InterceptSolver.cs`:
- `SolveQuadratic()` - Closed-form initial solution
- Uses `Brent.FindRoot()` from MathNet.Numerics for path-based solving
- `Bisection.FindRoot()` for guaranteed convergence when Brent fails
- Newton iterations for sub-microsecond refinement

## Consequences

**Positive:**
- Handles all edge cases (parallel motion, tangent paths, path corners)
- Precision to `1e-15` units (sub-pixel accuracy)
- Average ~4-8μs per prediction

**Negative:**
- MathNet.Numerics dependency (~500KB)
- Complex fallback logic increases code complexity

**Tradeoffs:**
- Brent requires brackets (slower but guaranteed)
- Newton is faster but can diverge without good initial guess
- Combination provides best of both: speed for easy cases, reliability for hard cases
