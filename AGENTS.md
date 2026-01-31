# Predicto - Agent Guidelines

## Quick Reference

| Command | Purpose |
|---------|---------|
| `dotnet build` | Build entire solution |
| `dotnet test` | Run all tests (185 tests) |
| `dotnet run --project src/Predicto.Simulation` | Visual simulation |

## Project Overview

High-precision ballistic prediction engine for game AI (League of Legends style).
C# .NET 10.0 with MathNet.Numerics/Spatial. Zero-allocation design, ~8 microseconds per prediction.

## Code Style

- Use **record types** for immutable data (PredictionInput, PredictionResult)
- Use **stack-allocated structs** for performance-critical code
- Add **XML documentation** (`///`) on all public members
- Only allowed magic numbers: epsilon, 0.5, 1, 2

## Architecture

See `docs/architecture/` for detailed design decisions:
- `001-adaptive-strategy.md` - Targeting strategy selection
- `002-numerical-methods.md` - MathNet integration
- `003-zero-allocation-design.md` - Performance patterns

## Testing

- Use xUnit with `[Fact]` and `[Theory]` attributes
- Test naming: `MethodName_Scenario` or `Scenario_ExpectedBehavior`
- All edge cases require regression tests in `EdgeCaseRegressionTests.cs`
