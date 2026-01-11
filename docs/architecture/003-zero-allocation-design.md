# ADR-003: Zero-Allocation Performance Design

**Status:** Accepted

## Context

Skillshot prediction runs in game loops at 30+ Hz. Each tick may evaluate multiple targets. GC pauses are unacceptable for real-time gameplay.

## Decision

Use **stack-allocated structs** exclusively in hot paths:

1. All model types are `readonly record struct` (not classes)
2. All intermediate calculations use value types
3. No LINQ in prediction paths (avoids iterator allocations)
4. Results are value types (`PredictionResult` is a discriminated union of structs)

## Implementation

Key patterns:
- `PredictionInput` - readonly record struct
- `PredictionResult.Hit/OutOfRange/Unreachable` - readonly records
- `Point2D`, `Vector2D` from MathNet.Spatial - structs
- `TargetPath` stores `IReadOnlyList<Point2D>` (allocated once, reused)

See `Models/PredictionInput.cs`, `Models/PredictionResult.cs`.

## Consequences

**Positive:**
- Zero GC pressure per prediction
- ~8μs average prediction time
- Cache-friendly memory layout
- No heap allocations in steady state

**Negative:**
- Structs copied on assignment (mitigated by `in` parameters)
- Cannot use inheritance for result types (using discriminated union pattern instead)
- `TargetPath` must own its waypoint list (slight API friction)

**Measurements:**
- 1000 predictions: ~8ms total, 0 bytes allocated
- No GC collections during benchmark runs
