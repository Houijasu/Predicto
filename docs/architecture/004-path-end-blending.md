# ADR-004: Path-End Blending with Smoothstep

**Status:** Accepted

## Context

When a target approaches the end of their movement path, the "behind-target" strategy can fail:
- Target will stop or reverse at path end
- Aiming behind current direction misses if target stops
- Abrupt aim transitions look jittery and are exploitable

## Decision

Use **smoothstep blending** to transition from trailing-edge aim to center aim as target approaches path end.

The blend factor uses the smoothstep function:
```
smoothstep(t) = 3t² - 2t³  (for t ∈ [0,1])
```

This provides C1-continuous transition (no velocity discontinuity in aim point).

## Implementation

See `Ultimate.cs`:
- `Constants.PathEndBlendThreshold = 0.5s` - Time threshold for blending
- `ApplyPathEndBlending()` - Interpolates between trailing-edge and center aim
- Blend starts when `timeToPathEnd < PathEndBlendThreshold`

```
blendFactor = smoothstep(1 - timeToPathEnd / threshold)
finalAim = lerp(trailingEdgeAim, centerAim, blendFactor)
```

## Consequences

**Positive:**
- Smooth aim transitions (no jitter)
- Higher hit probability at path endpoints
- Psychologically harder to exploit (gradual change)

**Negative:**
- Slightly reduced "behind" advantage near path end
- Adds ~2 floating-point operations per prediction
- Threshold (0.5s) is tuned for LoL; may need adjustment for other games
