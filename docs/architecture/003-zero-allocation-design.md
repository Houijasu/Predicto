# ADR-003: Low-Allocation Performance Design

**Status:** Accepted (Revised)

## Context

Skillshot prediction runs in game loops at 30+ Hz. Each tick may evaluate multiple targets. GC pauses are unacceptable for real-time gameplay.

## Decision

Use **stack-allocated structs** for inputs and intermediates in hot paths:

1. All input model types are `readonly record struct` (not classes)
2. All intermediate calculations use value types
3. No LINQ in prediction paths (avoids iterator allocations)
4. `PredictionResult` uses an abstract record class with sealed subtypes (`Hit`, `OutOfRange`, `Unreachable`) — these allocate on the heap but are small, short-lived objects
5. `TargetPath` is a sealed class (heap-allocated) to own its waypoint list

**Note:** The design minimizes allocations but is not strictly zero-allocation.
`PredictionResult` and `TargetPath` allocate on the heap by design (discriminated union
pattern requires reference types for inheritance in C#).

## Implementation

Key patterns:
- `PredictionInput` - readonly record struct
- `PredictionResult.Hit/OutOfRange/Unreachable` - sealed record classes (heap-allocated)
- `Point2D`, `Vector2D` from MathNet.Spatial - structs
- `TargetPath` - sealed class storing `IReadOnlyList<Point2D>` (allocated once, reused)

See `Models/PredictionInput.cs`, `Models/PredictionResult.cs`.

## Consequences

**Positive:**
- Minimal GC pressure per prediction (only result object allocated)
- ~8μs average prediction time
- Cache-friendly memory layout for inputs and intermediates
- No heap allocations for computation-heavy solver paths

**Negative:**
- Structs copied on assignment (mitigated by `in` parameters)
- `PredictionResult` requires heap allocation (C# discriminated unions need reference types)
- `TargetPath` must own its waypoint list (slight API friction)
- Delegate-based callbacks (`Func<double, double>`) for root-finders allocate closures on heap

**Measurements:**
- 1000 predictions: ~8ms total
- Primary allocations: PredictionResult per call, TargetPath per path input
- Solver internals (quadratic, Newton, bisection) are allocation-free
