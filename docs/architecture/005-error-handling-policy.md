# ADR-005: Layered Error Handling Policy

**Status:** Accepted

## Context

The library has two layers:
1. `Ultimate` - Public API consumed by game code
2. `InterceptSolver` - Internal mathematical solver

Each layer has different error handling requirements:
- Game code runs in hot loops; exceptions are expensive
- Internal solver needs fast-fail for programming errors

## Decision

Use **different error handling strategies per layer**:

### Layer 1: Ultimate.cs (Public API)
- **All failures return `PredictionResult.Unreachable`**
- No exceptions thrown during prediction
- Includes both invalid inputs and unsolvable scenarios

### Layer 2: InterceptSolver.cs (Internal)
- **Throws `ArgumentException`** for programming errors (invalid inputs)
- **Returns `null`** for unsolvable scenarios (no valid solution)
- `Ultimate` converts `null` to `Unreachable`

## Implementation

```csharp
// InterceptSolver.cs - throws for programming errors
if (skillshotSpeed <= 0)
    throw new ArgumentException("Speed must be positive");

// Returns null if no mathematical solution exists
return null;

// Ultimate.cs - returns Unreachable for all failures
if (input.Skillshot.Speed <= 0)
    return new PredictionResult.Unreachable("Speed must be positive");
```

## Consequences

**Positive:**
- Zero exception overhead in game loops
- Internal solver catches bugs early during development
- Clear separation of "programmer error" vs "runtime condition"

**Negative:**
- Dual validation (some checks duplicated at both layers)
- `Unreachable` conflates invalid inputs with unsolvable scenarios at API level

**Rationale:**
For a game AI library, the cost of throwing exceptions in hot paths outweighs the benefit of distinguishing error types. Callers typically only care "can I hit?" not "why can't I hit?"
