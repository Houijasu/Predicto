# Predicto

A high-precision ballistic prediction engine for skillshot interception in game AI. Designed for League of Legends-style gameplay mechanics.

## Features

- **Trailing Edge Targeting**: Aims at the back of the target's hitbox relative to their movement direction, making dodging nearly impossible
- **Maximum Aggressiveness**: Aims just 0.1 pixels inside the collision zone (99.9% edge utilization)
- **Multi-Path Prediction**: Handles complex movement patterns with multiple waypoints
- **Sub-Pixel Precision**: Triple refinement system (Quadratic → Secant → Bisection) achieves ~1e-15 precision
- **Cast Delay Compensation**: Properly accounts for skillshot wind-up time

## How It Works

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

Instead of aiming at the target's center or leading edge, Predicto aims at the **trailing edge** - the point where the skillshot just barely clips the back of the hitbox. This makes the prediction extremely difficult to dodge since the target would need to reverse direction.

## Project Structure

```
src/
  Predicto/                    # Core prediction library
    Models/
      LinearSkillshot.cs       # Skillshot parameters
      PredictionInput.cs       # Input configuration
      PredictionResult.cs      # Result types (Hit/Miss/OutOfRange)
      TargetPath.cs            # Multi-waypoint path handling
    Solvers/
      InterceptSolver.cs       # Mathematical solving engine
    Ultimate.cs                # Main prediction API
    Constants.cs               # Game constants (tick rate, etc.)
  
  Predicto.Simulation/         # Visual testing simulation
    Program.cs                 # Raylib-based visualization

tests/
  Predicto.Tests/              # Unit tests (80 tests)
```

## Usage

```csharp
var ultimate = new Ultimate();

var input = new PredictionInput(
    CasterPosition: new Point2D(300, 400),
    TargetPosition: new Point2D(800, 400),
    TargetVelocity: new Vector2D(0, -350),
    TargetHitboxRadius: 65,
    Skillshot: new LinearSkillshot(
        Speed: 2000,
        Range: 2000,
        Width: 40,
        Delay: 0.25
    )
);

var result = ultimate.Predict(input);

if (result is PredictionResult.Hit hit)
{
    Console.WriteLine($"Aim at: {hit.CastPosition}");
    Console.WriteLine($"Target will be at: {hit.PredictedTargetPosition}");
    Console.WriteLine($"Intercept time: {hit.InterceptTime}s");
    Console.WriteLine($"Confidence: {hit.Confidence:P0}");
}
```

## Running the Simulation

```bash
dotnet run --project src/Predicto.Simulation
```

Controls:
- **Left Click**: Set caster position
- **Right Click**: Set target position / add waypoints (path mode)
- **Space**: Fire skillshot
- **P**: Toggle path mode (multi-waypoint)
- **C**: Start continuous firing (path mode)
- **R**: Reset
- **Mouse Wheel**: Zoom

## Running Tests

```bash
dotnet test
```

## Constants

Key parameters in `Constants.cs`:

| Constant | Value | Description |
|----------|-------|-------------|
| `ServerTickRate` | 30 Hz | LoL server tick rate |
| `DefaultHitboxRadius` | 65 units | Standard champion hitbox |
| `MaxPredictionTime` | 3.0 s | Maximum prediction horizon |

## License

MIT
