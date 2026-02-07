# Predicto

A high-precision ballistic prediction engine for skillshot interception in game AI. Designed for League of Legends-style gameplay mechanics.

## Features

- **Adaptive Prediction Strategy**: Centered between **Trailing-Edge** and **Tangent** methods - provides the most reliable hit geometry by staying in the optimal center of the target's movement window.
- **Absolute Minimal Interception Time**: Mathematical model accounting for **Rectangular Geometry** and **Caster Hitbox Offset** to find the absolute earliest possible collision.
- **MathNet Integration**: Leverages high-precision `MathNet.Numerics` for root-finding (Brent, Newton-Raphson) and `MathNet.Spatial` for robust geometric operations.
- **Path-End Blending**: Smoothly transitions from adaptive to center aim when target approaches the end of their movement path (C1-continuous smoothstep).
- **Hitscan Support**: Instant beam prediction for abilities like Lux R and Xerath Q.
- **Linear & Circular Skillshots**: Full support for both projectile-based and ground-targeted abilities.
- **Multi-Target Priority Selection**: Rank and select optimal targets in team fights based on confidence, priority, and range.
- **Multi-Path Prediction**: Global path root-finder handles complex movement patterns with multiple waypoints.
- **Performance Optimized**: ~8 microseconds per prediction, minimal GC pressure with stack-allocated structs for inputs and value-type intermediates.

## How It Works

### Adaptive Prediction Strategy

Predicto defaults to the **Adaptive** strategy, which combines several targeting methods to maximize hit reliability:

1. **Direct Behind**: Aims at the trailing edge of the target path. Forces the target to reverse direction to dodge.
2. **Tangent Point**: Aims at the tangent of the target's hitbox from the caster's perspective. Optimized for wide projectiles.
3. **Adaptive (Default)**: Centers the aim point between the Trailing-Edge and Tangent points, then projects it back to the hitbox boundary. This maintains the "behind-target" psychological advantage while optimizing for the largest possible collision window.

### Absolute Minimal Interception

When the `MinimizeTime` flag is set, the engine bypasses strategic heuristics to find the absolute earliest collision:

- **Rectangular Geometry**: Models the projectile as a flat-front rectangle rather than a circle, saving $Width/2$ in travel distance.
- **Caster Hitbox Offset**: Projectile originates from the edge of the caster's hitbox ($R_{caster}$ offset), further reducing travel time.
- **Global Path Root-Finding**: Uses Brent's method to find the first root of the collision function across all path segments simultaneously.

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

### Mathematical Core

The engine leverages **MathNet.Numerics** and **MathNet.Spatial** for high-precision vector and root-finding operations:

1. **Quadratic Solver**: Uses library-level polynomial root finding for stable kinematic estimates.
2. **Brent's Method**: Employed for global path root-finding and off-path iterative refinement (Continuous Collision Detection).
3. **Bisection**: Final stage refinement ensuring precision down to `1e-15` units.

## Project Structure

```
src/
  Predicto/                    # Core prediction library
    Models/
      LinearSkillshot.cs       # Linear skillshot parameters (with 20+ presets)
      CircularSkillshot.cs     # Circular skillshot parameters (with 20+ presets)
      PredictionInput.cs       # Input configuration
      PredictionResult.cs      # Result types (Hit/OutOfRange/Unreachable)
      TargetPath.cs            # Multi-waypoint path handling
    Solvers/
      InterceptSolver.cs       # Mathematical solving engine
      OffPathAimPointSolver.cs # Off-path aim point strategies (DirectBehind/Tangent/Adaptive)
    Physics/
      DangerFields.cs          # Danger field potentials (experimental)
      FastMath.cs              # Inlined math utilities (Lorentzian, etc.)
      StructOdeSolver.cs       # Zero-allocation ODE integrators (RK4/Euler/Verlet)
    Ultimate.cs                # Main prediction API
    IPrediction.cs             # Prediction interface
    Constants.cs               # Game constants and helper methods
  
  Predicto.Simulation/         # Visual testing simulation
    Program.cs                 # Raylib-based visualization (with --bench CLI mode)

tests/
  Predicto.Tests/              # Unit tests (185 tests)
    InterceptSolverTests.cs    # Solver unit tests
    UltimateTests.cs           # Integration tests
    EdgeCaseRegressionTests.cs # Regression tests for bug fixes
    OffPathAimPointSolverTests.cs # Off-path aim strategy tests
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
    TargetHitboxRadius: 65,
    CasterHitboxRadius: 65,
    MinimizeTime: false,
    Strategy: BehindEdgeStrategy.Adaptive
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
- **M**: Cycle aim strategies (DirectBehind/Tangent/Adaptive/Gagong)
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

Current test coverage: **185 tests** covering:
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
| Memory allocations | Minimal (inputs are stack-allocated structs; results are heap-allocated records) |
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
| `DefaultCasterHitboxRadius` | 65 units | Standard caster hitbox size |
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
| `PredictCircular(CircularPredictionInput)` | Predicts intercept for circular skillshot |
| `PredictReactive(PredictionInput)` | **[Experimental/Obsolete]** Physics-based dodge prediction |
| `RankTargets(casterPos, skillshot, targets)` | Ranks multiple targets by hit probability |
| `RankTargetsCircular(casterPos, skillshot, targets)` | Ranks targets for circular skillshot |
| `GetBestTarget(casterPos, skillshot, targets)` | Gets single best hittable target |

### `InterceptSolver` Class

Low-level mathematical solver using MathNet.

| Method | Description |
|--------|-------------|
| `SolveMinimalInterceptDirect` | Absolute earliest hit (velocity) |
| `SolvePathMinimalIntercept` | Absolute earliest hit (path-based) |
| `SolveBehindTargetDirect` | Strategy-based intercept (velocity) |
| `SolvePathBehindTargetIntercept` | Strategy-based intercept (path) |
| `SolveHitscanBehindTarget` | Instant beam interception |
| `CalculateConfidence` | Prediction reliability score |

### `OffPathAimPointSolver` Class

Calculates off-path aim points using behind-target strategies.

| Method | Description |
|--------|-------------|
| `SolveOffPathIntercept` | Iterative CCD refinement using Brent's method |
| `CalculateDirectBehindPoint` | Aim behind target relative to movement direction |
| `CalculateTangentPoint` | Aim at tangent of hitbox from caster's perspective |
| `CalculateAdaptivePoint` | Blend of DirectBehind and Tangent (default) |

### Model Types

```csharp
// Configuration strategy for behind-edge aim
enum BehindEdgeStrategy { DirectBehind, Tangent, Adaptive, Gagong }

// Prediction input parameters
PredictionInput(CasterPosition, TargetPosition, TargetVelocity, Skillshot, 
                TargetHitboxRadius, TargetPath, CasterHitboxRadius, 
                MinimizeTime, Strategy)

// Target candidate for multi-target selection
TargetCandidate(Position, Velocity, HitboxRadius, PriorityWeight, Path, Tag, Strategy)
```

## License

MIT
