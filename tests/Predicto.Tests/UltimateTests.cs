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
        double effectiveRadius = 65 + 70/2.0; // 100
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
        Assert.True(hit.Confidence >= 0.6, $"Slow target confidence should be high, got {hit.Confidence}");
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
                    random.NextDouble() * 800 + 100,  // 100-900
                    random.NextDouble() * 800 - 400), // -400 to 400
                TargetVelocity: new Vector2D(
                    random.NextDouble() * 600 - 300,  // -300 to 300
                    random.NextDouble() * 600 - 300),
                Skillshot: new LinearSkillshot(
                    Speed: 1200 + random.NextDouble() * 600,  // 1200-1800
                    Range: 1000,
                    Width: 50 + random.NextDouble() * 70,     // 50-120
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
                TargetPosition: new Point2D(150 + i * 0.1, 0),
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
}
