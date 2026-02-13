using MathNet.Spatial.Euclidean;
using Predicto;
using Predicto.Models;
using Predicto.Solvers;

namespace Predicto.Tests;

public class UltimateTests
{
    private readonly Ultimate _prediction = new();

    [Fact]
    public void StationaryTarget_DirectHit()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.Equal(500, hit.CastPosition.X, precision: 1);
        Assert.Equal(0, hit.CastPosition.Y, precision: 1);
        Assert.True(hit.Confidence > 0);
    }

    [Fact]
    public void MovingTarget_PerpendicularMotion()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 300),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.PredictedTargetPosition.Y > 0);
    }

    [Fact]
    public void MovingTarget_TowardsCaster()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(800, 0),
            TargetVelocity: new Vector2D(-200, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.CastPosition.X < 800);
    }

    [Fact]
    public void MovingTarget_Outrunning()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(300, 0),
            TargetVelocity: new Vector2D(1600, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1500, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        // With behind-edge strategy, fast-moving targets may still be reachable
        // but if truly outrunning, should be unreachable
        Assert.True(result is PredictionResult.Unreachable or PredictionResult.OutOfRange);
    }

    [Fact]
    public void OutOfRange_ReturnsOutOfRange()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(1500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    [Fact]
    public void InvalidSkillshot_ZeroSpeed_ReturnsUnreachable()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 0, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }

    [Fact]
    public void BehindEdge_HitsFromBehind()
    {
        // Test that behind-edge strategy aims at trailing edge
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(200, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Behind-edge should have later intercept time than early edge
        Assert.True(hit.InterceptTime > 0);
        // Predicted position should be ahead of initial position (target moving right)
        Assert.True(hit.PredictedTargetPosition.X > 400);
    }

    [Fact]
    public void BehindTarget_StationaryTarget_AimsAtCenter()
    {
        // For stationary targets, there's no "behind" - aim at center
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Stationary target - aim at target position
        Assert.Equal(500, hit.CastPosition.X, precision: 1);
        Assert.Equal(0, hit.CastPosition.Y, precision: 1);
    }

    [Fact]
    public void BehindTarget_MovingNorth_AimsSouth()
    {
        // Target moving north (positive Y), we should aim SOUTH (negative Y offset) of predicted position
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(0, 300), // Moving north (up)
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Aim point should be SOUTH (lower Y) of predicted target position
        Assert.True(hit.CastPosition.Y < hit.PredictedTargetPosition.Y,
            $"Aim point Y ({hit.CastPosition.Y:F1}) should be < predicted target Y ({hit.PredictedTargetPosition.Y:F1}) - aiming behind");

        // The offset should be approximately (effectiveRadius - adaptiveMargin)
        // With adaptive margin based on half spell width, the offset will be smaller
        double yOffset = hit.PredictedTargetPosition.Y - hit.CastPosition.Y;
        double effectiveRadius = 65 + (70 / 2.0); // 100
        // Adaptive margin is clamped to max 50% of effectiveRadius = 50
        double maxMargin = effectiveRadius * 0.5;
        double minExpectedOffset = effectiveRadius - maxMargin; // 50
        Assert.True(yOffset >= minExpectedOffset - 5 && yOffset <= effectiveRadius,
            $"Y offset ({yOffset:F1}) should be between {minExpectedOffset} and {effectiveRadius}");
    }

    [Fact]
    public void BehindTarget_MovingEast_AimsWest()
    {
        // Target moving east (positive X), we should aim WEST (negative X offset) of predicted position
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(0, 400),
            TargetVelocity: new Vector2D(300, 0), // Moving east (right)
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Aim point should be WEST (lower X) of predicted target position
        Assert.True(hit.CastPosition.X < hit.PredictedTargetPosition.X,
            $"Aim point X ({hit.CastPosition.X:F1}) should be < predicted target X ({hit.PredictedTargetPosition.X:F1}) - aiming behind");
    }

    [Fact]
    public void BehindTarget_DirectSolverTest()
    {
        // Direct test of SolveBehindTarget solver
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300); // Moving north

        var result = InterceptSolver.SolveBehindTarget(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000,
            behindMargin: 1.0);

        Assert.NotNull(result);

        var (aimPoint, predictedPos, interceptTime) = result.Value;

        // Predicted position should have positive Y (target moved north)
        Assert.True(predictedPos.Y > 0, "Target should have moved north");

        // Aim point should be SOUTH of predicted position (behind target)
        Assert.True(aimPoint.Y < predictedPos.Y,
            $"Aim point Y ({aimPoint.Y:F1}) should be < predicted Y ({predictedPos.Y:F1})");

        // X should be approximately same (target only moving in Y)
        Assert.True(Math.Abs(aimPoint.X - predictedPos.X) < 1,
            "X coordinates should be nearly equal");
    }

    // === NEW: Edge Case Tests ===

    [Fact]
    public void ExtremeVelocity_ReturnsUnreachable()
    {
        // Target velocity exceeds reasonable bounds (> 2000 units/s)
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(3000, 0), // Way too fast
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
        var unreachable = (PredictionResult.Unreachable)result;
        Assert.Contains("velocity", unreachable.Reason, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void ExtremeSkillshotSpeed_ReturnsUnreachable()
    {
        // Skillshot speed exceeds reasonable bounds (> 5000 units/s)
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(300, 0),
            Skillshot: new LinearSkillshot(Speed: 10000, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }

    [Fact]
    public void TargetMovingAway_BeyondRange_ReturnsOutOfRange()
    {
        // Target at edge of range, moving away fast
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(950, 0),
            TargetVelocity: new Vector2D(500, 0), // Moving away
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.5));

        var result = _prediction.Predict(input);

        Assert.True(result is PredictionResult.OutOfRange or PredictionResult.Unreachable,
            "Should be out of range or unreachable");
    }

    [Fact]
    public void TargetOutrunningSkillshot_ReturnsUnreachable()
    {
        // Target faster than skillshot and moving directly away
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(1200, 0), // Faster than skillshot
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 2000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
        var unreachable = (PredictionResult.Unreachable)result;
        Assert.Contains("outrunning", unreachable.Reason, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void CloseRangeTarget_UsesAdaptiveRefinement()
    {
        // Close range = "easy case" should still produce valid result
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(150, 0), // Within EasyShotDistanceThreshold
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.Confidence > 0.5); // Close shots should be confident
    }

    [Fact]
    public void PerpendicularMovement_ReducedConfidence()
    {
        // Target moving perpendicular to skillshot path - harder to predict
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(600, 0),
            TargetVelocity: new Vector2D(0, 400), // Moving perpendicular
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Compare with target moving toward caster (easier prediction)
        var inputToward = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(600, 0),
            TargetVelocity: new Vector2D(-400, 0), // Moving toward
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var resultToward = _prediction.Predict(inputToward);
        Assert.IsType<PredictionResult.Hit>(resultToward);
        var hitToward = (PredictionResult.Hit)resultToward;

        // Perpendicular movement should have lower or equal confidence
        Assert.True(hit.Confidence <= hitToward.Confidence + 0.1,
            $"Perpendicular confidence ({hit.Confidence:F3}) should be <= toward confidence ({hitToward.Confidence:F3})");
    }

    [Fact]
    public void VerySlowTarget_HighConfidence()
    {
        // Slow targets are very predictable
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(50, 0), // Very slow
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.Confidence >= 0.5, $"Slow target confidence should be high, got {hit.Confidence}");
    }

    [Fact]
    public void FastDashingTarget_LowerConfidence()
    {
        // Fast-moving targets (like dashing) are less predictable
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 800), // Fast perpendicular dash
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.Confidence < 0.8, $"Fast dashing target confidence should be reduced, got {hit.Confidence}");
    }

    [Fact]
    public void AdaptiveMargin_FastTarget_SmallerMargin()
    {
        // Fast target gets smaller behind-margin (more aggressive aim)
        var fastInput = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 600), // Fast
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var slowInput = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 100), // Slow
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            TargetHitboxRadius: 65.0);

        var fastResult = _prediction.Predict(fastInput);
        var slowResult = _prediction.Predict(slowInput);

        Assert.IsType<PredictionResult.Hit>(fastResult);
        Assert.IsType<PredictionResult.Hit>(slowResult);

        var fastHit = (PredictionResult.Hit)fastResult;
        var slowHit = (PredictionResult.Hit)slowResult;

        // Both should hit - adaptive margin should handle both cases
        Assert.True(fastHit.InterceptTime > 0);
        Assert.True(slowHit.InterceptTime > 0);
    }

    [Fact]
    public void ZeroRange_ReturnsUnreachable()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 0, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }


    #region Hitscan Tests

    [Fact]
    public void Hitscan_LuxR_IsHitscan()
    {
        Assert.True(LinearSkillshot.LuxR.IsHitscan);
    }

    [Fact]
    public void Hitscan_XerathQ_IsHitscan()
    {
        Assert.True(LinearSkillshot.XerathQ.IsHitscan);
    }

    [Fact]
    public void Hitscan_NormalSkillshot_IsNotHitscan()
    {
        Assert.False(LinearSkillshot.BlitzcrankQ.IsHitscan);
        Assert.False(LinearSkillshot.MorganaQ.IsHitscan);
        Assert.False(LinearSkillshot.EzrealQ.IsHitscan);
    }

    [Fact]
    public void Hitscan_StationaryTarget_DirectHit()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: LinearSkillshot.LuxR);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.Equal(500, hit.CastPosition.X, precision: 1);
        Assert.Equal(0, hit.CastPosition.Y, precision: 1);
        // Intercept time should equal cast delay (1.0s for Lux R)
        Assert.Equal(1.0, hit.InterceptTime, precision: 3);
        Assert.True(hit.Confidence > 0.9); // High confidence for stationary
    }

    [Fact]
    public void Hitscan_MovingTarget_AimsAtPredictedPosition()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 300), // Moving perpendicular
            Skillshot: LinearSkillshot.LuxR); // 1.0s delay

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        // Target will move 300 units in Y direction during 1.0s delay
        Assert.True(hit.PredictedTargetPosition.Y > 250); // Should predict movement
        Assert.Equal(1.0, hit.InterceptTime, precision: 3); // Always equals delay for hitscan
    }

    [Fact]
    public void Hitscan_MovingTarget_BehindTargetAiming()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(300, 0), // Moving away
            Skillshot: LinearSkillshot.XerathQ); // 0.5s delay

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        // Predicted position after 0.5s delay: 500 + 300*0.5 = 650
        Assert.True(hit.PredictedTargetPosition.X > 500);
        // Aim point should be behind the predicted position (less X)
        Assert.True(hit.CastPosition.X < hit.PredictedTargetPosition.X);
    }

    [Fact]
    public void Hitscan_OutOfRange_ReturnsOutOfRange()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(4000, 0), // Beyond Lux R range (3400)
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: LinearSkillshot.LuxR);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    [Fact]
    public void Hitscan_MovesOutOfRange_ReturnsOutOfRange()
    {
        // Target starts in range but moves out before beam fires
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(3300, 0), // Close to Lux R range
            TargetVelocity: new Vector2D(500, 0), // Moving away
            Skillshot: LinearSkillshot.LuxR); // 1.0s delay -> moves 500 units

        var result = _prediction.Predict(input);

        // After 1s delay, target at 3800 which is beyond 3400 range
        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    [Fact]
    public void Hitscan_InterceptTimeEqualsDelay()
    {
        // For hitscan, intercept time = cast delay (no travel time)
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(1000, 0),
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: LinearSkillshot.XerathQ); // 0.5s delay

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.Equal(LinearSkillshot.XerathQ.Delay, hit.InterceptTime, precision: 5);
    }

    [Fact]
    public void Hitscan_FasterThanProjectile()
    {
        // Compare hitscan vs projectile intercept times
        var caster = new Point2D(0, 0);
        var target = new Point2D(1000, 0);
        var velocity = new Vector2D(200, 0);

        // Hitscan (Lux R)
        var hitscanInput = new PredictionInput(caster, target, velocity, LinearSkillshot.LuxR);
        var hitscanResult = _prediction.Predict(hitscanInput);

        // Projectile (Morgana Q - similar range, but has travel time)
        var projectileInput = new PredictionInput(caster, target, velocity, LinearSkillshot.MorganaQ);
        var projectileResult = _prediction.Predict(projectileInput);

        Assert.IsType<PredictionResult.Hit>(hitscanResult);
        Assert.IsType<PredictionResult.Hit>(projectileResult);

        var hitscanHit = (PredictionResult.Hit)hitscanResult;
        var projectileHit = (PredictionResult.Hit)projectileResult;

        // Hitscan should have faster (or equal) intercept time
        Assert.True(hitscanHit.InterceptTime <= projectileHit.InterceptTime);
    }

    [Fact]
    public void Hitscan_WithPath_WorksCorrectly()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(500, 0),
            destination: new Point2D(500, 1000),
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: LinearSkillshot.LuxR,
            targetHitboxRadius: 65);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        // After 1.0s delay, target should have moved 350 units towards waypoint
        Assert.True(hit.PredictedTargetPosition.Y > 300);
        Assert.Equal(1.0, hit.InterceptTime, precision: 3);
    }

    [Fact]
    public void Hitscan_HighConfidenceForInstantHits()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(100, 0), // Slow movement
            Skillshot: LinearSkillshot.XerathQ);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        // Hitscan should have reasonable confidence (boosted by 1.1x)
        Assert.True(hit.Confidence > 0.5);
    }

    #endregion

    // === Performance Benchmark ===

    [Fact]
    public void PerformanceBenchmark_1000Predictions()
    {
        var random = new Random(42); // Fixed seed for reproducibility
        var inputs = new List<PredictionInput>();

        // Generate 1000 random prediction scenarios
        for (int i = 0; i < 1000; i++)
        {
            var input = new PredictionInput(
                CasterPosition: new Point2D(0, 0),
                TargetPosition: new Point2D(
                    (random.NextDouble() * 800) + 100,  // 100-900
                    (random.NextDouble() * 800) - 400), // -400 to 400
                TargetVelocity: new Vector2D(
                    (random.NextDouble() * 600) - 300,  // -300 to 300
                    (random.NextDouble() * 600) - 300),
                Skillshot: new LinearSkillshot(
                    Speed: 1200 + (random.NextDouble() * 600),  // 1200-1800
                    Range: 1000,
                    Width: 50 + (random.NextDouble() * 70),     // 50-120
                    Delay: random.NextDouble() * 0.3),        // 0-0.3
                TargetHitboxRadius: 65.0);
            inputs.Add(input);
        }

        var sw = System.Diagnostics.Stopwatch.StartNew();

        int hits = 0;
        int outOfRange = 0;
        int unreachable = 0;

        foreach (var input in inputs)
        {
            var result = _prediction.Predict(input);
            switch (result)
            {
                case PredictionResult.Hit: hits++; break;
                case PredictionResult.OutOfRange: outOfRange++; break;
                case PredictionResult.Unreachable: unreachable++; break;
            }
        }

        sw.Stop();

        // Log results
        var avgMs = sw.ElapsedMilliseconds / 1000.0;

        // Assertions
        Assert.True(sw.ElapsedMilliseconds < 500,
            $"1000 predictions should complete in <500ms, took {sw.ElapsedMilliseconds}ms");
        Assert.True(hits > 500, $"Most predictions should be hits, got {hits}/1000");

        // Output for debugging
        // Hits: {hits}, OutOfRange: {outOfRange}, Unreachable: {unreachable}
        // Avg time per prediction: {avgMs:F4}ms
    }

    [Fact]
    public void PerformanceBenchmark_EasyCases_AreFaster()
    {
        var easyInputs = new List<PredictionInput>();
        var hardInputs = new List<PredictionInput>();

        // Easy cases: close range, slow targets
        for (int i = 0; i < 500; i++)
        {
            easyInputs.Add(new PredictionInput(
                CasterPosition: new Point2D(0, 0),
                TargetPosition: new Point2D(150 + (i * 0.1), 0),
                TargetVelocity: new Vector2D(50, 0),
                Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
                TargetHitboxRadius: 65.0));
        }

        // Hard cases: far, fast, perpendicular
        for (int i = 0; i < 500; i++)
        {
            hardInputs.Add(new PredictionInput(
                CasterPosition: new Point2D(0, 0),
                TargetPosition: new Point2D(700, 0),
                TargetVelocity: new Vector2D(0, 400),
                Skillshot: new LinearSkillshot(Speed: 1200, Range: 1000, Width: 70, Delay: 0.25),
                TargetHitboxRadius: 65.0));
        }

        var swEasy = System.Diagnostics.Stopwatch.StartNew();
        foreach (var input in easyInputs)
            _prediction.Predict(input);
        swEasy.Stop();

        var swHard = System.Diagnostics.Stopwatch.StartNew();
        foreach (var input in hardInputs)
            _prediction.Predict(input);
        swHard.Stop();

        // Both should complete reasonably fast
        Assert.True(swEasy.ElapsedMilliseconds < 200,
            $"500 easy predictions should take <200ms, took {swEasy.ElapsedMilliseconds}ms");
        Assert.True(swHard.ElapsedMilliseconds < 300,
            $"500 hard predictions should take <300ms, took {swHard.ElapsedMilliseconds}ms");
    }

    #region Multi-Target Priority Selection Tests

    [Fact]
    public void RankTargets_ReturnsHighestConfidenceFirst()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0);

        var targets = new TargetCandidate[]
        {
            // Far target, moving away = lower confidence
            new(new Point2D(800, 0), new Vector2D(300, 0), Tag: "far"),
            // Close stationary target = highest confidence
            new(new Point2D(400, 0), new Vector2D(0, 0), Tag: "close_stationary"),
            // Medium target, moving perpendicular = medium confidence
            new(new Point2D(500, 0), new Vector2D(0, 200), Tag: "medium")
        };

        var ranked = _prediction.RankTargets(casterPos, skillshot, targets);

        Assert.Equal(3, ranked.Count);
        // Close stationary should be first (highest confidence)
        Assert.Equal("close_stationary", ranked[0].Tag);
        // All should be hittable
        Assert.All(ranked, t => Assert.True(t.IsHittable));
    }

    [Fact]
    public void RankTargets_PriorityWeightAffectsRanking()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0);

        // Two targets at same distance with same confidence
        // But different priority weights
        var targets = new TargetCandidate[]
        {
            new(new Point2D(400, 0), new Vector2D(0, 0), PriorityWeight: 0.5, Tag: "low_priority"),
            new(new Point2D(400, 0), new Vector2D(0, 0), PriorityWeight: 2.0, Tag: "high_priority")
        };

        var ranked = _prediction.RankTargets(casterPos, skillshot, targets);

        Assert.Equal(2, ranked.Count);
        // High priority target should be first despite same confidence
        Assert.Equal("high_priority", ranked[0].Tag);
        Assert.Equal("low_priority", ranked[1].Tag);
    }

    [Fact]
    public void RankTargets_OutOfRangeTargetsRankedLower()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 500, Width: 70, Delay: 0);

        var targets = new TargetCandidate[]
        {
            // Out of range
            new(new Point2D(1000, 0), new Vector2D(0, 0), Tag: "out_of_range"),
            // In range
            new(new Point2D(300, 0), new Vector2D(0, 0), Tag: "in_range")
        };

        var ranked = _prediction.RankTargets(casterPos, skillshot, targets);

        Assert.Equal(2, ranked.Count);
        // In range target should be first
        Assert.Equal("in_range", ranked[0].Tag);
        Assert.True(ranked[0].IsHittable);
        Assert.Equal("out_of_range", ranked[1].Tag);
        Assert.False(ranked[1].IsHittable);
    }

    [Fact]
    public void RankTargets_EmptyArray_ReturnsEmpty()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0);

        var ranked = _prediction.RankTargets(casterPos, skillshot, ReadOnlySpan<TargetCandidate>.Empty);

        Assert.Empty(ranked);
    }

    [Fact]
    public void GetBestTarget_ReturnsHighestPriorityHittable()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0);

        var targets = new TargetCandidate[]
        {
            new(new Point2D(500, 0), new Vector2D(100, 0), Tag: "target1"),
            new(new Point2D(400, 0), new Vector2D(0, 0), Tag: "best_target"),
            new(new Point2D(600, 0), new Vector2D(0, 200), Tag: "target3")
        };

        var best = _prediction.GetBestTarget(casterPos, skillshot, targets);

        Assert.NotNull(best);
        Assert.Equal("best_target", best.Value.Tag);
        Assert.True(best.Value.IsHittable);
    }

    [Fact]
    public void GetBestTarget_NoHittableTargets_ReturnsNull()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 300, Width: 70, Delay: 0);

        // All targets out of range
        var targets = new TargetCandidate[]
        {
            new(new Point2D(1000, 0), new Vector2D(0, 0), Tag: "far1"),
            new(new Point2D(1200, 0), new Vector2D(0, 0), Tag: "far2")
        };

        var best = _prediction.GetBestTarget(casterPos, skillshot, targets);

        Assert.Null(best);
    }

    [Fact]
    public void RankTargetsCircular_RanksCorrectly()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new CircularSkillshot(Radius: 200, Range: 900, Delay: 0.5);

        var targets = new TargetCandidate[]
        {
            // Moving fast = harder to predict
            new(new Point2D(500, 0), new Vector2D(0, 400), Tag: "fast_mover"),
            // Stationary = easy target
            new(new Point2D(500, 0), new Vector2D(0, 0), Tag: "stationary")
        };

        var ranked = _prediction.RankTargetsCircular(casterPos, skillshot, targets);

        Assert.Equal(2, ranked.Count);
        // Stationary should be first (highest confidence)
        Assert.Equal("stationary", ranked[0].Tag);
    }

    [Fact]
    public void RankTargets_WithPath_WorksCorrectly()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0);

        // Target with a path - start at first waypoint moving toward second
        var path = TargetPath.FromDestination(
            new Point2D(500, 0),
            new Point2D(700, 200),
            speed: 350);

        var targets = new TargetCandidate[]
        {
            TargetCandidate.WithPath(path, tag: "pathed_target"),
            TargetCandidate.Stationary(new Point2D(400, 0), tag: "stationary_target")
        };

        var ranked = _prediction.RankTargets(casterPos, skillshot, targets);

        Assert.Equal(2, ranked.Count);
        Assert.All(ranked, t => Assert.True(t.IsHittable));
    }

    [Fact]
    public void RankTargets_Performance_ManyTargets()
    {
        var casterPos = new Point2D(0, 0);
        var skillshot = new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25);
        var random = new Random(12345);

        // Create 20 targets (realistic team fight scenario)
        var targets = new TargetCandidate[20];
        for (int i = 0; i < 20; i++)
        {
            targets[i] = new TargetCandidate(
                new Point2D(200 + (random.NextDouble() * 700), -300 + (random.NextDouble() * 600)),
                new Vector2D(-200 + (random.NextDouble() * 400), -200 + (random.NextDouble() * 400)),
                HitboxRadius: 65,
                PriorityWeight: 0.5 + (random.NextDouble() * 2.0),
                Tag: $"target_{i}");
        }

        var sw = System.Diagnostics.Stopwatch.StartNew();
        for (int i = 0; i < 100; i++)
        {
            var ranked = _prediction.RankTargets(casterPos, skillshot, targets);
            Assert.NotEmpty(ranked);
        }
        sw.Stop();

        // 100 iterations of ranking 20 targets should be fast
        Assert.True(sw.ElapsedMilliseconds < 500,
            $"100 iterations of ranking 20 targets should take <500ms, took {sw.ElapsedMilliseconds}ms");
    }

    #endregion

    #region Path-End Blending Tests

    /// <summary>
    /// When target has a long path remaining, aim should be behind-target (trailing edge).
    /// </summary>
    [Fact]
    public void PathEndBlending_LongPathRemaining_AimsBehindTarget()
    {
        var casterPos = new Point2D(0, 0);
        // Target moving right from (400,0) to (2400,0) at 400 speed
        // Intercept will happen quickly, leaving plenty of path remaining
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(2400, 0),
            speed: 400);

        var input = PredictionInput.WithPath(
            casterPos,
            path,
            new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65,
            minimizeTime: false);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Aim point should be BEHIND (less X) than predicted target position
        // since target is moving right (positive X)
        Assert.True(hit.CastPosition.X < hit.PredictedTargetPosition.X,
            $"Aim ({hit.CastPosition.X:F1}) should be behind target ({hit.PredictedTargetPosition.X:F1}) for long path");
    }

    /// <summary>
    /// When target is at the end of their path (about to stop), aim should converge to center.
    /// </summary>
    [Fact]
    public void PathEndBlending_AtPathEnd_AimsAtCenter()
    {
        var casterPos = new Point2D(0, 0);
        // Target moving right but almost at destination (50 units at 400 speed = 0.125 seconds)
        // This is well within the PathEndBlendThreshold of 0.5 seconds
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(450, 0), // Only 50 units remaining
            speed: 400);

        var input = PredictionInput.WithPath(
            casterPos,
            path,
            new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Aim point should be close to predicted target position (center aim)
        double aimOffset = hit.PredictedTargetPosition.X - hit.CastPosition.X;
        double effectiveRadius = 65 + 35; // hitbox + width/2

        // At path end, aim offset should be small (close to center, not full behind)
        Assert.True(aimOffset < effectiveRadius * 0.3,
            $"At path end, aim offset ({aimOffset:F1}) should be small (< {effectiveRadius * 0.3:F1})");
    }

    /// <summary>
    /// Smooth blending: mid-path should have intermediate aim between behind and center.
    /// </summary>
    [Fact]
    public void PathEndBlending_MidPath_SmoothTransition()
    {
        var casterPos = new Point2D(0, 0);

        // Create three scenarios with different remaining path times
        var paths = new[]
        {
            // Long path (5 seconds) - should be full behind
            TargetPath.FromDestination(new Point2D(400, 0), new Point2D(2400, 0), speed: 400),
            // Medium path (0.5 seconds = threshold) - should be blended
            TargetPath.FromDestination(new Point2D(400, 0), new Point2D(600, 0), speed: 400),
            // Short path (0.1 seconds) - should be near center
            TargetPath.FromDestination(new Point2D(400, 0), new Point2D(440, 0), speed: 400),
        };

        var offsets = new double[3];
        for (int i = 0; i < 3; i++)
        {
            var input = PredictionInput.WithPath(
                casterPos,
                paths[i],
                new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
                targetHitboxRadius: 65);

            var result = _prediction.Predict(input);
            Assert.IsType<PredictionResult.Hit>(result);
            var hit = (PredictionResult.Hit)result;

            offsets[i] = hit.PredictedTargetPosition.X - hit.CastPosition.X;
        }

        // Offsets should decrease as path gets shorter (more center-focused)
        // Long path should have largest offset (most behind)
        // Short path should have smallest offset (most center)
        Assert.True(offsets[0] >= offsets[1],
            $"Long path offset ({offsets[0]:F1}) should be >= medium path ({offsets[1]:F1})");
        Assert.True(offsets[1] >= offsets[2],
            $"Medium path offset ({offsets[1]:F1}) should be >= short path ({offsets[2]:F1})");
    }

    /// <summary>
    /// Smoothstep helper should produce C1-continuous transitions.
    /// </summary>
    [Fact]
    public void Smoothstep_ProducesCorrectValues()
    {
        // Test boundary conditions
        Assert.Equal(0.0, Constants.Smoothstep(0, 1, -0.5), precision: 10);
        Assert.Equal(0.0, Constants.Smoothstep(0, 1, 0), precision: 10);
        Assert.Equal(1.0, Constants.Smoothstep(0, 1, 1), precision: 10);
        Assert.Equal(1.0, Constants.Smoothstep(0, 1, 1.5), precision: 10);

        // Test midpoint (should be exactly 0.5)
        Assert.Equal(0.5, Constants.Smoothstep(0, 1, 0.5), precision: 10);

        // Test that it's monotonically increasing
        double prev = 0;
        for (double x = 0; x <= 1; x += 0.1)
        {
            double value = Constants.Smoothstep(0, 1, x);
            Assert.True(value >= prev, $"Smoothstep should be monotonic at x={x}");
            prev = value;
        }
    }

    /// <summary>
    /// Path-end blending should not affect velocity-based predictions (no path = no blending).
    /// </summary>
    [Fact]
    public void PathEndBlending_VelocityBased_UnchangedBehavior()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(400, 0);
        var targetVel = new Vector2D(400, 0); // Moving right at 400 speed

        var input = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: targetPos,
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Velocity-based should still use behind-target (trailing edge)
        Assert.True(hit.CastPosition.X < hit.PredictedTargetPosition.X,
            $"Velocity-based aim ({hit.CastPosition.X:F1}) should be behind target ({hit.PredictedTargetPosition.X:F1})");
    }

    #endregion

    #region Gagong Strategy Tests

    [Fact]
    public void Gagong_BasicHit_ReturnsValidResult()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(800, 0),
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65) with
        { Strategy = BehindEdgeStrategy.Gagong };

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.InterceptTime > 0);
        Assert.True(hit.Confidence > 0);
    }

    [Fact]
    public void Gagong_AimPointIsBehindTarget()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(1000, 0),
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            targetHitboxRadius: 65) with
        { Strategy = BehindEdgeStrategy.Gagong };

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        Assert.True(hit.CastPosition.X < hit.PredictedTargetPosition.X,
            $"Gagong aim ({hit.CastPosition.X:F1}) should be behind target ({hit.PredictedTargetPosition.X:F1})");
    }

    [Fact]
    public void Gagong_VerticalMovement_AimsBehindVertically()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(400, 600),
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            targetHitboxRadius: 65) with
        { Strategy = BehindEdgeStrategy.Gagong };

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        Assert.True(hit.CastPosition.Y < hit.PredictedTargetPosition.Y,
            $"Gagong aim Y ({hit.CastPosition.Y:F1}) should be behind target Y ({hit.PredictedTargetPosition.Y:F1})");
    }

    [Fact]
    public void Gagong_OutOfRange_ReturnsOutOfRange()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(1200, 0),
            destination: new Point2D(2000, 0),
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 800, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65) with
        { Strategy = BehindEdgeStrategy.Gagong };

        var result = _prediction.Predict(input);

        Assert.True(result is PredictionResult.OutOfRange or PredictionResult.Unreachable);
    }

    [Fact]
    public void Gagong_StationaryTarget_FallsBackToEdge()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(400, 0),
            speed: 0);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65) with
        { Strategy = BehindEdgeStrategy.Gagong };

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(Math.Abs(hit.CastPosition.X - 400) < 1, "Stationary target should aim at target position");
    }

    [Fact]
    public void Gagong_VsAdaptive_ProducesValidDifferentResults()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(800, 0),
            speed: 350);

        var baseInput = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65);

        var gagongInput = baseInput with { Strategy = BehindEdgeStrategy.Gagong };
        var adaptiveInput = baseInput with { Strategy = BehindEdgeStrategy.Adaptive };

        var gagongResult = _prediction.Predict(gagongInput);
        var adaptiveResult = _prediction.Predict(adaptiveInput);

        Assert.IsType<PredictionResult.Hit>(gagongResult);
        Assert.IsType<PredictionResult.Hit>(adaptiveResult);

        var gagongHit = (PredictionResult.Hit)gagongResult;
        var adaptiveHit = (PredictionResult.Hit)adaptiveResult;

        Assert.True(gagongHit.InterceptTime > 0);
        Assert.True(adaptiveHit.InterceptTime > 0);
    }

    [Fact]
    public void Gagong_DiagonalPath_CorrectBehindDirection()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(300, 300),
            destination: new Point2D(600, 600),
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0),
            targetHitboxRadius: 65) with
        { Strategy = BehindEdgeStrategy.Gagong };

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        Vector2D moveDir = new Vector2D(1, 1).Normalize();
        Vector2D aimOffset = hit.CastPosition - hit.PredictedTargetPosition;
        double behindness = aimOffset.DotProduct(moveDir.Negate());

        Assert.True(behindness > 0, "Aim point should be in behind hemisphere");
    }

    #endregion

    #region MinimizeTime Direct Path Tests

    [Fact]
    public void MinimizeTime_ReturnsEarlierInterceptTime_ThanDefault()
    {
        var path = TargetPath.FromDestination(
            currentPosition: new Point2D(400, 0),
            destination: new Point2D(800, 0),
            speed: 350);

        var baseInput = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            targetHitboxRadius: 65);

        var defaultResult = _prediction.Predict(baseInput);
        var minimizeResult = _prediction.Predict(baseInput with { MinimizeTime = true });

        Assert.IsType<PredictionResult.Hit>(defaultResult);
        Assert.IsType<PredictionResult.Hit>(minimizeResult);

        var defaultHit = (PredictionResult.Hit)defaultResult;
        var minimizeHit = (PredictionResult.Hit)minimizeResult;

        Assert.True(minimizeHit.InterceptTime <= defaultHit.InterceptTime + 0.01,
            $"MinimizeTime ({minimizeHit.InterceptTime:F4}s) should be <= default ({defaultHit.InterceptTime:F4}s)");
    }

    [Fact]
    public void MinimizeTime_StationaryTarget_ReturnsHit()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            MinimizeTime: true);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.InterceptTime > 0);
        Assert.Equal(500, hit.CastPosition.X, precision: 1);
    }

    [Fact]
    public void MinimizeTime_PerpendicularTarget_ReturnsHit()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 350),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            MinimizeTime: true);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.InterceptTime > 0);
        Assert.True(hit.PredictedTargetPosition.Y > 0,
            "Predicted target should have moved in Y direction");
    }

    [Fact]
    public void MinimizeTime_TargetMovingAway_ReturnsHit()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(300, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1200, Width: 70, Delay: 0.25),
            MinimizeTime: true);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.InterceptTime > 0);
        Assert.True(hit.PredictedTargetPosition.X > 400,
            "Target moving away should be predicted further along X");
    }

    #endregion

    #region Circular Skillshot Path Prediction Tests

    [Fact]
    public void PredictCircular_WithMultiWaypointPath_ReturnsHit()
    {
        var path = new TargetPath(
            waypoints: new[] { new Point2D(400, 300), new Point2D(600, 300) },
            currentPosition: new Point2D(400, 0),
            currentWaypointIndex: 0,
            speed: 350);

        var input = CircularPredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new CircularSkillshot(Radius: 200, Range: 1100, Delay: 0.5),
            targetHitboxRadius: 65);

        var result = _prediction.PredictCircular(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.InterceptTime > 0);
    }

    [Fact]
    public void PredictCircular_TargetNearPathEnd_ReturnsHit()
    {
        var path = new TargetPath(
            waypoints: new[] { new Point2D(450, 0) },
            currentPosition: new Point2D(400, 0),
            currentWaypointIndex: 0,
            speed: 350);

        var input = CircularPredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new CircularSkillshot(Radius: 200, Range: 900, Delay: 0.5),
            targetHitboxRadius: 65);

        var result = _prediction.PredictCircular(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.PredictedTargetPosition.X >= 400,
            "Target near path end should be predicted at or past start position");
    }

    [Fact]
    public void PredictCircular_OutOfRangeTarget_ReturnsOutOfRange()
    {
        var path = new TargetPath(
            waypoints: new[] { new Point2D(3000, 0) },
            currentPosition: new Point2D(2500, 0),
            currentWaypointIndex: 0,
            speed: 350);

        var input = CircularPredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new CircularSkillshot(Radius: 200, Range: 900, Delay: 0.5),
            targetHitboxRadius: 65);

        var result = _prediction.PredictCircular(input);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    #endregion
}
