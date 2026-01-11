# ADR-001: Adaptive Behind-Target Strategy

**Status:** Accepted

## Context

Skillshot prediction can use different aiming strategies:

1. **DirectBehind**: Aims directly behind target's center relative to movement direction
2. **Tangent**: Aims at the tangent point of target hitbox from caster's perspective
3. **Adaptive**: Blends both approaches

Each has tradeoffs between psychological advantage and geometric precision.

## Decision

Use **Adaptive** as the default strategy (`BehindEdgeStrategy.Adaptive`).

The Adaptive strategy:
1. Computes both DirectBehind and Tangent aim points
2. Finds the midpoint between them
3. Projects this midpoint back onto the target's hitbox boundary

This provides:
- The "behind-target" psychological advantage (target can't see projectile approaching)
- Optimal geometric collision window (centered between extremes)
- Robust behavior across varying target hitbox sizes and caster positions

## Implementation

See `Ultimate.cs`:
- `ComputeAdaptiveOffset()` - Blends DirectBehind and Tangent offsets
- `BehindEdgeStrategy` enum in `PredictionInput.cs`

## Consequences

**Positive:**
- Higher hit probability than pure DirectBehind on large hitboxes
- More psychologically effective than pure Tangent
- Graceful degradation to DirectBehind when Tangent is invalid

**Negative:**
- Slightly more computation than DirectBehind alone (~2 vector operations)
- May be suboptimal for extreme edge cases (very wide skillshots, very small targets)
