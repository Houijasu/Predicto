# Predicto

A high-precision ballistic prediction engine for skillshot interception in game AI. Designed for League of Legends-style gameplay mechanics.

## Features

- **Behind-Target Strategy**: Aims BEHIND the target relative to their movement direction - target cannot see the projectile coming and must completely reverse to dodge
- **Path-End Blending**: Smoothly transitions from trailing-edge to center aim when target approaches the end of their movement path (C1-continuous smoothstep)
- **Sub-Pixel Precision**: Refinement system using MathNet.Numerics (RobustNewtonRaphson + Bisection) achieves ~1e-15 precision
- **Linear & Circular Skillshots**: Full support for both projectile-based and ground-targeted abilities
- **Multi-Target Priority Selection**: Rank and select optimal targets in team fights
- **Reaction Time Modeling**: Confidence adjustment based on human reaction time limitations
- **Multi-Path Prediction**: Handles complex movement patterns with multiple waypoints
- **Adaptive Confidence**: Scoring system accounts for distance, target speed, approach angle, and prediction reliability
- **Cast Delay Compensation**: Properly accounts for skillshot wind-up time
- **Performance Optimized**: ~8 microseconds per prediction, zero GC pressure with stack-allocated structs
- **30+ Built-in Presets**: Pre-configured skillshots for popular League of Legends abilities

## How It Works

### The Behind-Target Strategy

Unlike traditional prediction that aims at target center or leading edge, Predicto aims **behind** the target:

```
Target moving: ↑ (north)
Predicted position: ●
Aim point (behind): ○ ← projectile arrives from behind
```

Why this is optimal:
- Target cannot see projectile approaching from their movement direction
- Must completely REVERSE direction to dodge
- Projectile "catches up" from behind

### Path-End Blending

When a target is following a path and approaching the final waypoint (about to stop), pure trailing-edge aim becomes suboptimal. Predicto smoothly blends from trailing-edge to center aim:

```
Path remaining > 0.5s: Pure trailing-edge (aim behind target)
Path remaining < 0.5s: Smooth blend toward center
Path remaining = 0:    Center aim (fastest intercept for stopped target)
```

The transition uses a C1-continuous smoothstep function for visually smooth aim adjustment.

### Mathematical Model

The engine solves the interception equation:

```
|D + V·t| = s·(t - d) + r
```

Where:
- `D` = displacement from caster to target
- `V` = target velocity
- `t` = intercept time
- `s` = skillshot speed
- `d` = cast delay
- `r` = effective radius (target hitbox + skillshot width/2)

### Triple Refinement Pipeline

1. **Quadratic Formula**: O(1) analytical solution for initial estimate
2. **RobustNewtonRaphson**: Combines Newton-Raphson with bisection fallback for robust convergence
3. **Bisection Refinement**: Guaranteed convergence to ~1e-15 (sub-pixel precision)

## Project Structure

```
src/
  Predicto/                    # Core prediction library
    Models/
      LinearSkillshot.cs       # Linear skillshot parameters
      CircularSkillshot.cs     # Circular skillshot parameters (with presets)
      PredictionInput.cs       # Input configuration
      PredictionResult.cs      # Result types (Hit/OutOfRange/Unreachable)
      TargetPath.cs            # Multi-waypoint path handling
    Solvers/
      InterceptSolver.cs       # Mathematical solving engine (~2800 lines)
    Ultimate.cs                # Main prediction API
    IPrediction.cs             # Prediction interface
    Constants.cs               # Game constants
  
  Predicto.Simulation/         # Visual testing simulation
    Program.cs                 # Raylib-based visualization

tests/
  Predicto.Tests/              # Unit tests (144 tests)
    InterceptSolverTests.cs    # Solver unit tests
    UltimateTests.cs           # Integration tests
    EdgeCaseRegressionTests.cs # Regression tests for bug fixes
```

## Usage

### Linear Skillshots (Projectiles)

```csharp
var prediction = new Ultimate();

var input = new PredictionInput(
    CasterPosition: new Point2D(300, 400),
    TargetPosition: new Point2D(800, 400),
    TargetVelocity: new Vector2D(0, -350),
    Skillshot: new LinearSkillshot(
        Speed: 2000,
        Range: 2000,
        Width: 40,
        Delay: 0.25
    ),
    TargetHitboxRadius: 65
);

var result = prediction.Predict(input);

switch (result)
{
    case PredictionResult.Hit hit:
        Console.WriteLine($"Aim at: {hit.CastPosition}");
        Console.WriteLine($"Target will be at: {hit.PredictedTargetPosition}");
        Console.WriteLine($"Intercept time: {hit.InterceptTime:F3}s");
        Console.WriteLine($"Confidence: {hit.Confidence:P0}");
        break;
    case PredictionResult.OutOfRange oor:
        Console.WriteLine($"Target at {oor.Distance:F0} exceeds range {oor.MaxRange:F0}");
        break;
    case PredictionResult.Unreachable u:
        Console.WriteLine($"Cannot intercept: {u.Reason}");
        break;
}
```

### Circular Skillshots (Ground-Targeted)

```csharp
var input = new CircularPredictionInput(
    CasterPosition: new Point2D(0, 0),
    TargetPosition: new Point2D(400, 0),
    TargetVelocity: new Vector2D(0, 300),
    Skillshot: CircularSkillshot.XerathW, // Built-in preset
    TargetHitboxRadius: 65
);

var result = prediction.PredictCircular(input);
```

### Built-in Skillshot Presets

#### Linear Skillshots (20+ presets)
```csharp
// Hooks/Pulls
LinearSkillshot.BlitzcrankQ  // Speed: 1800, Range: 1150, Width: 70, Delay: 0.25
LinearSkillshot.ThreshQ      // Speed: 1900, Range: 1100, Width: 70, Delay: 0.5
LinearSkillshot.NautilusQ    // Speed: 2000, Range: 1100, Width: 90, Delay: 0.25
LinearSkillshot.PykeQ        // Speed: 2500, Range: 1100, Width: 70, Delay: 0.2

// Binds/Roots
LinearSkillshot.MorganaQ     // Speed: 1200, Range: 1300, Width: 70, Delay: 0.25
LinearSkillshot.LuxQ         // Speed: 1200, Range: 1300, Width: 70, Delay: 0.25
LinearSkillshot.NeekoE       // Speed: 1300, Range: 1000, Width: 70, Delay: 0.25
LinearSkillshot.ZyraE        // Speed: 1150, Range: 1100, Width: 70, Delay: 0.25

// Damage/Poke
LinearSkillshot.EzrealQ      // Speed: 2000, Range: 1200, Width: 60, Delay: 0.25
LinearSkillshot.NidaleeQ     // Speed: 1300, Range: 1500, Width: 40, Delay: 0.25
LinearSkillshot.JinxW        // Speed: 3300, Range: 1500, Width: 60, Delay: 0.6
LinearSkillshot.CaitlynQ     // Speed: 2200, Range: 1300, Width: 90, Delay: 0.625
LinearSkillshot.KaiSaW       // Speed: 1750, Range: 3000, Width: 100, Delay: 0.4
LinearSkillshot.ZoeE         // Speed: 1850, Range: 800, Width: 50, Delay: 0.25

// Ultimates
LinearSkillshot.AsheR        // Speed: 1600, Range: Global, Width: 130, Delay: 0.25
LinearSkillshot.EzrealR      // Speed: 2000, Range: Global, Width: 160, Delay: 1.0
LinearSkillshot.JinxR        // Speed: 2150, Range: Global, Width: 140, Delay: 0.6
LinearSkillshot.LuxR         // Speed: Instant, Range: 3400, Width: 100, Delay: 1.0
LinearSkillshot.SennaR       // Speed: 20000, Range: Global, Width: 180, Delay: 1.0
```

#### Circular Skillshots (20+ presets)
```csharp
// Mage Abilities
CircularSkillshot.XerathW       // Radius: 200, Range: 1100, Delay: 0.5
CircularSkillshot.XerathWCenter // Radius: 50 (inner circle for bonus damage)
CircularSkillshot.VeigarW       // Radius: 112, Range: 950, Delay: 1.2
CircularSkillshot.BrandW        // Radius: 260, Range: 900, Delay: 0.625
CircularSkillshot.SyndraQ       // Radius: 200, Range: 800, Delay: 0.6
CircularSkillshot.SyndraW       // Radius: 225, Range: 925, Delay: 0.25
CircularSkillshot.LuxE          // Radius: 310, Range: 1100, Delay: 0.25

// Tank/Fighter Abilities
CircularSkillshot.ChoGathQ      // Radius: 175, Range: 950, Delay: 0.5
CircularSkillshot.GragasQ       // Radius: 250, Range: 850, Delay: 0.5
CircularSkillshot.ZacE          // Radius: 300, Range: 1800, Delay: 0.9
CircularSkillshot.SionQ         // Radius: 200, Range: 750, Delay: 0.6

// Support/Control
CircularSkillshot.NamiQ         // Radius: 180, Range: 875, Delay: 0.85
CircularSkillshot.VeigarE       // Radius: 375, Range: 725, Delay: 0.5
CircularSkillshot.SorakaE       // Radius: 260, Range: 925, Delay: 1.5

// Ultimates
CircularSkillshot.ZiggsR        // Radius: 550, Range: 5300, Delay: 1.5
CircularSkillshot.XerathR       // Radius: 200, Range: 5000, Delay: 0.5
CircularSkillshot.GangplankR    // Radius: 600, Range: Global, Delay: 0.67
```

### Multi-Waypoint Path Prediction

```csharp
// Target following L-shaped path
var path = new TargetPath(
    waypoints: new[] { new Point2D(200, 300), new Point2D(500, 300) },
    currentPosition: new Point2D(200, 0),
    currentWaypointIndex: 0,
    speed: 350
);

var input = PredictionInput.WithPath(
    casterPosition: new Point2D(0, 0),
    path: path,
    skillshot: new LinearSkillshot(Speed: 1500, Range: 800, Width: 70, Delay: 0.25)
);

var result = prediction.Predict(input);
```

### Multi-Target Priority Selection

Rank multiple targets in team fights to find the optimal target:

```csharp
var prediction = new Ultimate();
var casterPos = new Point2D(0, 0);
var skillshot = LinearSkillshot.BlitzcrankQ;

// Create target candidates with priority weights
var targets = new TargetCandidate[]
{
    new(new Point2D(500, 0), new Vector2D(0, 200), PriorityWeight: 2.0, Tag: "ADC"),
    new(new Point2D(400, 100), new Vector2D(100, 0), PriorityWeight: 0.5, Tag: "Tank"),
    new(new Point2D(600, -50), new Vector2D(-50, 100), PriorityWeight: 1.5, Tag: "Mage"),
};

// Get all targets ranked by priority score
var ranked = prediction.RankTargets(casterPos, skillshot, targets);
foreach (var target in ranked)
{
    Console.WriteLine($"{target.Tag}: Score={target.PriorityScore:F2}, Hittable={target.IsHittable}");
}

// Or get just the best hittable target
var best = prediction.GetBestTarget(casterPos, skillshot, targets);
if (best.HasValue)
{
    var hit = best.Value.HitResult!;
    Console.WriteLine($"Best target: {best.Value.Tag} at {hit.CastPosition}");
}
```

Priority scoring factors:
- **Confidence** (0-1): Base hit probability
- **Priority Weight**: User-defined importance (e.g., ADC = 2.0, Tank = 0.5)
- **Range Efficiency**: Closer targets preferred when confidence is similar

## Running the Simulation

```bash
dotnet run --project src/Predicto.Simulation
```

Controls:
- **Left Click**: Set caster position
- **Right Click**: Set target position (clears waypoints)
- **Shift+Right Click**: Add waypoint (path mode)
- **Space**: Fire skillshot
- **Delete/Backspace**: Clear waypoints
- **Tab**: Cycle through presets
- **` (Backtick)**: Toggle Linear/Circular mode
- **Q/W**: Decrease/Increase target speed
- **E/R**: Decrease/Increase projectile speed
- **1-5**: Set time multiplier
- **Mouse Wheel**: Zoom

### Visualization Legend (Path Mode)

When in path mode with linear skillshots, two aim points are displayed:
- **Green circle**: Current prediction (with path-end blending)
- **Blue circle**: Pure trailing-edge prediction (no blending)

This allows you to see the difference between blended and pure trailing-edge aim, especially when the target is near the end of their path.

## Running Tests

```bash
dotnet test
```

Current test coverage: **144 tests** covering:
- Basic interception scenarios
- Edge cases (boundary conditions, numerical precision)
- Multi-target priority selection
- Path-end blending behavior
- Regression tests for bug fixes
- Performance benchmarks

## Performance

| Metric | Value |
|--------|-------|
| Average prediction time | ~8 μs |
| 1000 predictions | ~8 ms |
| Memory allocations | Zero (stack-allocated structs) |
| Easy cases (close, slow) | ~4 μs |
| Hard cases (far, fast) | ~12 μs |

## Constants

Key parameters in `Constants.cs`:

| Constant | Value | Description |
|----------|-------|-------------|
| `ServerTickRate` | 30 Hz | LoL server tick rate |
| `DefaultHitboxRadius` | 65 units | Standard champion hitbox |
| `Epsilon` | 1e-9 | Numerical comparison tolerance |
| `MaxReasonableVelocity` | 2000 units/s | Maximum target speed |
| `MaxReasonableSkillshotSpeed` | 5000 units/s | Maximum skillshot speed |
| `TrailingEdgeMargin` | 1.0 unit | Pixel margin for behind-target aim |
| `PathEndBlendThreshold` | 0.5s | Time threshold for path-end blending |
| `AverageReactionTime` | 0.25s | Human average reaction time |
| `MinReactionTime` | 0.15s | Fast human reaction time |
| `MaxReactionTime` | 0.5s | Slow human reaction time |

## API Reference

### `Ultimate` Class

Main prediction engine implementing `IPrediction`.

| Method | Description |
|--------|-------------|
| `Predict(PredictionInput)` | Predicts intercept for linear skillshot |
| `PredictPureTrailingEdge(PredictionInput)` | Predicts without path-end blending (always trailing-edge) |
| `PredictCircular(CircularPredictionInput)` | Predicts intercept for circular skillshot |
| `RankTargets(casterPos, skillshot, targets)` | Ranks multiple targets by hit probability |
| `RankTargetsCircular(casterPos, skillshot, targets)` | Ranks targets for circular skillshot |
| `GetBestTarget(casterPos, skillshot, targets)` | Gets single best hittable target |
| `GetBestTargetCircular(casterPos, skillshot, targets)` | Gets best target for circular |

### `InterceptSolver` Class

Low-level mathematical solver with multiple precision levels.

| Method | Description |
|--------|-------------|
| `SolveInterceptTime` | Basic center-to-center intercept |
| `SolveEdgeInterceptTime` | Edge-to-edge collision detection |
| `SolveBehindTarget` | Behind-target strategy (quadratic only) |
| `SolveBehindTargetWithSecantRefinement` | + Secant refinement |
| `SolveBehindTargetWithFullRefinement` | + Bisection for max precision |
| `SolvePathBehindTargetIntercept` | Multi-waypoint path support |
| `CalculateConfidence` | Prediction reliability score |

### Model Types

```csharp
// Target candidate for multi-target selection
TargetCandidate(Position, Velocity, HitboxRadius, PriorityWeight, Path?, Tag?)

// Ranked result
RankedTarget(Candidate, Result, PriorityScore)
  .IsHittable          // bool - can target be hit
  .HitResult           // Hit? - the hit result if hittable
  .Tag                 // object? - user data for identification

// Result types
PredictionResult.Hit           // Success - CastPosition, PredictedTargetPosition, InterceptTime, Confidence
PredictionResult.OutOfRange    // Target beyond skillshot range
PredictionResult.Unreachable   // No valid intercept (target too fast, etc.)
```

## License

MIT
