# Contributing to Predicto

## Getting Started

```bash
git clone <repo-url>
cd Predicto
dotnet build
dotnet test
```

## Development

### Prerequisites
- .NET 10.0 SDK

### Build & Test
```bash
dotnet build              # Build entire solution
dotnet test               # Run all tests (213 tests)
dotnet format             # Apply code formatting
dotnet format --verify-no-changes  # Check formatting without modifying
```

### Running the Simulation
```bash
dotnet run --project src/Predicto.Simulation
```

### Running Benchmarks
```bash
dotnet run --project tests/Predicto.Benchmarks -c Release
```

## Code Style

- Use **record types** for immutable data (`PredictionInput`, `PredictionResult`)
- Use **stack-allocated structs** for performance-critical code
- Add **XML documentation** (`///`) on all public members
- Only allowed magic numbers: `epsilon`, `0.5`, `1`, `2` (game data values are exempt)
- Run `dotnet format` before committing

## Testing

- Use xUnit with `[Fact]` and `[Theory]` attributes
- Test naming: `MethodName_Scenario` or `Scenario_ExpectedBehavior`
- All edge cases require regression tests in `EdgeCaseRegressionTests.cs`

## Architecture

See `docs/architecture/` for detailed design decisions:
- `001-adaptive-strategy.md` - Targeting strategy selection
- `002-numerical-methods.md` - MathNet integration
- `003-zero-allocation-design.md` - Performance patterns

## Pull Requests

1. Create a feature branch from `main`
2. Make your changes
3. Ensure `dotnet build` passes with 0 warnings
4. Ensure all 213+ tests pass
5. Run `dotnet format` to fix formatting
6. Open a PR with a clear description of changes
