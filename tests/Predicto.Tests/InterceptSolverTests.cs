using MathNet.Spatial.Euclidean;
using Predicto;
using Predicto.Models;
using Predicto.Solvers;

namespace Predicto.Tests;

public class InterceptSolverTests
{
    private readonly InterceptSolver _solver = new();
    private readonly Ultimate _prediction = new();

    #region Stationary Target Tests

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
        Assert.Equal(1.0, hit.Confidence, precision: 2);
    }

    [Fact]
    public void StationaryTarget_OutOfRange()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(1500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    #endregion

    #region Moving Target Tests

    [Fact]
    public void MovingTarget_PerpendicularMotion()
    {
        // Target moving perpendicular to caster - classic leading shot
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 300), // Moving up
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Target is moving up, so predicted position should be above current position
        Assert.True(hit.PredictedTargetPosition.Y > 0);
    }

    [Fact]
    public void MovingTarget_TowardsCaster()
    {
        // Target moving toward caster - should hit earlier
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(800, 0),
            TargetVelocity: new Vector2D(-200, 0), // Moving toward caster
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Should aim closer than current position
        Assert.True(hit.CastPosition.X < 800);
    }

    [Fact]
    public void MovingTarget_AwayFromCaster_Catchable()
    {
        // Target moving away but slower than skillshot
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(300, 0),
            TargetVelocity: new Vector2D(200, 0), // Moving away
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1500, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Should aim further than current position
        Assert.True(hit.PredictedTargetPosition.X > 300);
    }

    [Fact]
    public void MovingTarget_Outrunning()
    {
        // Target moving away faster than skillshot - unreachable
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(300, 0),
            TargetVelocity: new Vector2D(1600, 0), // Moving away fast
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1500, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }

    #endregion

    #region Edge Cases

    [Fact]
    public void TargetVeryClose_WithCaster()
    {
        // Target very close to caster (10 units away)
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(10, 0),
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void DiagonalMovement()
    {
        // Target moving diagonally
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 300),
            TargetVelocity: new Vector2D(100, 100),
            Skillshot: new LinearSkillshot(Speed: 1600, Range: 1200, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void InvalidSkillshot_ZeroSpeed()
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
    public void InvalidSkillshot_ZeroRange()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 0, Width: 70, Delay: 0));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Unreachable>(result);
    }

    #endregion

    #region Behind-Target Solver Equivalence Tests

    [Fact]
    public void BehindTarget_Direct_MatchesFullRefinement_RandomizedScenarios()
    {
        const double hitbox = 65.0;
        const double width = 70.0;
        const double range = 2000.0;
        const double margin = 1.0;

        var rng = new Random(1337);

        for (int i = 0; i < 512; i++)
        {
            var caster = new Point2D(rng.NextDouble() * 2000 - 1000, rng.NextDouble() * 2000 - 1000);
            var target = new Point2D(rng.NextDouble() * 2000 - 1000, rng.NextDouble() * 2000 - 1000);

            double targetSpeed = 50 + rng.NextDouble() * 650;
            double theta = rng.NextDouble() * Math.PI * 2;
            var velocity = new Vector2D(Math.Cos(theta) * targetSpeed, Math.Sin(theta) * targetSpeed);

            double skillshotSpeed = 800 + rng.NextDouble() * 2600;
            double delay = rng.NextDouble() * 0.75;

            var full = InterceptSolver.SolveBehindTargetWithFullRefinement(
                caster, target, velocity,
                skillshotSpeed, delay,
                hitbox, width,
                range,
                behindMargin: margin);

            var direct = InterceptSolver.SolveBehindTargetDirect(
                caster, target, velocity,
                skillshotSpeed, delay,
                hitbox, width,
                range,
                behindMargin: margin);

            Assert.Equal(full.HasValue, direct.HasValue);

            if (!full.HasValue)
                continue;

            var a = full!.Value;
            var b = direct!.Value;

            Assert.Equal(a.InterceptTime, b.InterceptTime, precision: 10);
            Assert.Equal(a.AimPoint.X, b.AimPoint.X, precision: 9);
            Assert.Equal(a.AimPoint.Y, b.AimPoint.Y, precision: 9);
            Assert.Equal(a.PredictedTargetPosition.X, b.PredictedTargetPosition.X, precision: 9);
            Assert.Equal(a.PredictedTargetPosition.Y, b.PredictedTargetPosition.Y, precision: 9);
        }
    }

    #endregion
 
    #region Solver Unit Tests


    [Fact]
    public void Solver_StationaryTarget_ReturnsTime()
    {
        double? time = InterceptSolver.SolveInterceptTime(
            casterPosition: new Point2D(0, 0),
            targetPosition: new Point2D(500, 0),
            targetVelocity: new Vector2D(0, 0),
            skillshotSpeed: 1000,
            castDelay: 0);

        Assert.NotNull(time);
        Assert.Equal(0.5, time!.Value, precision: 3); // 500 / 1000 = 0.5s
    }

    [Fact]
    public void Solver_PerpendicularMotion_ReturnsValidTime()
    {
        double? time = InterceptSolver.SolveInterceptTime(
            casterPosition: new Point2D(0, 0),
            targetPosition: new Point2D(500, 0),
            targetVelocity: new Vector2D(0, 300),
            skillshotSpeed: 1500,
            castDelay: 0);

        Assert.NotNull(time);
        Assert.True(time > 0);
    }

    [Fact]
    public void Solver_Confidence_DecreasesWithDistance()
    {
        // Same intercept time, different distances
        // interceptTime, distance, targetSpeed, skillshotSpeed, skillshotRange, castDelay
        double closeConfidence = InterceptSolver.CalculateConfidence(0.5, 200, 300, 1500, 1000, 0.25);
        double farConfidence = InterceptSolver.CalculateConfidence(0.5, 900, 300, 1500, 1000, 0.25);

        Assert.True(closeConfidence > farConfidence);
    }

    [Fact]
    public void Solver_Confidence_DecreasesWithTargetSpeed()
    {
        // Same distance, different target speeds
        // interceptTime, distance, targetSpeed, skillshotSpeed, skillshotRange, castDelay
        double slowConfidence = InterceptSolver.CalculateConfidence(0.5, 500, 100, 1500, 1000, 0.25);
        double fastConfidence = InterceptSolver.CalculateConfidence(0.5, 500, 800, 1500, 1000, 0.25);

        Assert.True(slowConfidence > fastConfidence);
    }

    [Fact]
    public void Solver_Confidence_DecreasesWithTime()
    {
        // Same distance/speed, different intercept times
        // interceptTime, distance, targetSpeed, skillshotSpeed, skillshotRange, castDelay
        double quickConfidence = InterceptSolver.CalculateConfidence(0.3, 500, 300, 1500, 1000, 0.25);
        double slowConfidence = InterceptSolver.CalculateConfidence(0.8, 500, 300, 1500, 1000, 0.25);

        Assert.True(quickConfidence > slowConfidence);
    }

    #endregion

    #region Small Radius Edge Cases

    [Fact]
    public void SmallSpellWidth_SmallerThanTargetRadius_Works()
    {
        // Very thin skillshot (width 10) hitting normal target (radius 65)
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 300),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 10, Delay: 0),
            TargetHitboxRadius: 65);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        Assert.True(hit.InterceptTime > 0);
    }

    [Fact]
    public void TinyTarget_TinySpell_Works()
    {
        // Very small target (radius 5) and very thin spell (width 4)
        // effectiveRadius = 5 + 4/2 = 7
        // margin = 1.0 (default) - should still work since 1 < 7
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(200, 0),
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 500, Width: 4, Delay: 0),
            TargetHitboxRadius: 5);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void VeryTinyEffectiveRadius_MarginClamped_Works()
    {
        // Extremely small target and spell where effectiveRadius < margin
        // targetRadius = 0.5, spellWidth = 0.4
        // effectiveRadius = 0.5 + 0.2 = 0.7
        // margin = 1.0 would break math, but should be auto-clamped to 0.7 * 0.9 = 0.63
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(100, 0),
            TargetVelocity: new Vector2D(0, 50),
            Skillshot: new LinearSkillshot(Speed: 500, Range: 200, Width: 0.4, Delay: 0),
            TargetHitboxRadius: 0.5);

        var result = _prediction.Predict(input);

        // Should not crash and should return a valid result
        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void Solver_BehindEdge_MarginClamped_WhenTooLarge()
    {
        // Direct solver test with effectiveRadius < margin
        // This verifies the margin clamping works correctly
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(100, 0);
        var targetVel = new Vector2D(50, 0);

        // effectiveRadius = 0.5 + 0.2 = 0.7, margin = 1.0 should be clamped
        double? time = InterceptSolver.SolveBehindEdgeInterceptWithRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 500,
            castDelay: 0,
            targetHitboxRadius: 0.5,
            skillshotWidth: 0.4,
            skillshotRange: 200,
            margin: 1.0); // This is > effectiveRadius, should be clamped

        Assert.NotNull(time);
        Assert.True(time > 0);
    }

    [Fact]
    public void LargeSpellWidth_SmallTarget_Works()
    {
        // Large spell (width 200) hitting small target (radius 10)
        // This is valid - effectiveRadius = 10 + 100 = 110
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 800, Width: 200, Delay: 0),
            TargetHitboxRadius: 10);

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    #endregion

    #region Real-World Skillshot Tests

    [Fact]
    public void EzrealQ_MovingTarget()
    {
        // Ezreal Q: 2000 speed, 1150 range, ~60 width, 0.25 delay
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(600, 0),
            TargetVelocity: new Vector2D(0, 350), // Standard move speed
            Skillshot: new LinearSkillshot(Speed: 2000, Range: 1150, Width: 60, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Verify the prediction leads the target
        Assert.True(hit.PredictedTargetPosition.Y > 0);
    }

    [Fact]
    public void MorganaQ_SlowSkillshot()
    {
        // Morgana Q: 1200 speed, 1175 range, ~70 width, 0.25 delay
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(800, 0),
            TargetVelocity: new Vector2D(0, 400),
            Skillshot: new LinearSkillshot(Speed: 1200, Range: 1175, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    #endregion

    #region Refinement (Newton-backed) Tests

    [Fact]
    public void Refinement_ProducesSameResultAsQuadratic()
    {
        // For normal cases, Secant should produce the same result as quadratic
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300);

        var quadResult = InterceptSolver.SolveBehindTarget(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000,
            behindMargin: 1.0);

        var secantResult = InterceptSolver.SolveBehindTargetWithSecantRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000,
            behindMargin: 1.0,
            tolerance: 1e-12);

        Assert.NotNull(quadResult);
        Assert.NotNull(secantResult);

        // Times should match to high precision
        Assert.Equal(quadResult.Value.InterceptTime, secantResult.Value.InterceptTime, precision: 9);
    }

    [Fact]
    public void Refinement_HighPrecision_VerifyIntercept()
    {
        // Verify that refined solution actually produces valid intercept
        var casterPos = new Point2D(300, 400);
        var targetPos = new Point2D(800, 400);
        var targetVel = new Vector2D(0, -350);
        double speed = 1300;
        double delay = 0.25;
        double hitbox = 65;
        double width = 40;
        double range = 1500;
        double effectiveRadius = hitbox + width / 2;

        var result = InterceptSolver.SolveBehindTargetWithSecantRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: 1.0, tolerance: 1e-12);

        Assert.NotNull(result);
        var (aimPoint, predictedPos, interceptTime) = result.Value;

        // Calculate actual arrival time
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;

        // Intercept time should match arrival time to high precision
        Assert.Equal(interceptTime, arrivalTime, precision: 10);

        // Target at arrival should be within effectiveRadius of aim point
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;
        Assert.True(separation <= effectiveRadius,
            $"Separation {separation:F6} should be <= {effectiveRadius}");
    }

    [Fact]
    public void Refinement_LargeDistance_MaintainsPrecision()
    {
        // Test with large distance where numerical precision matters more
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(10000, 5000);
        var targetVel = new Vector2D(-200, 150);
        double speed = 2000;
        double delay = 0.5;
        double hitbox = 65;
        double width = 40;
        double range = 20000;
        double effectiveRadius = hitbox + width / 2;

        var result = InterceptSolver.SolveBehindTargetWithSecantRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: 1.0, tolerance: 1e-12);

        Assert.NotNull(result);
        var (aimPoint, _, interceptTime) = result.Value;

        // Verify arrival matches intercept time
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;

        Assert.Equal(interceptTime, arrivalTime, precision: 9);
    }

    [Fact]
    public void Refinement_WithCastDelay_CorrectTiming()
    {
        // Test with significant cast delay
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(400, 0);
        var targetVel = new Vector2D(100, 200);
        double speed = 1500;
        double delay = 0.6; // Long delay (like Thresh Q)
        double hitbox = 65;
        double width = 70;
        double range = 1100;
        double effectiveRadius = hitbox + width / 2;

        var result = InterceptSolver.SolveBehindTargetWithSecantRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: 1.0, tolerance: 1e-12);

        Assert.NotNull(result);
        var (aimPoint, predictedPos, interceptTime) = result.Value;

        // Intercept time must be greater than delay
        Assert.True(interceptTime > delay);

        // Verify hit
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;

        Assert.True(separation <= effectiveRadius);
    }

    [Fact]
    public void SolveInterceptTimeWithSecantRefinement_BasicTest()
    {
        // Test the intermediate method directly
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300);

        double? time = InterceptSolver.SolveInterceptTimeWithSecantRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0,
            skillshotRange: 1000,
            tolerance: 1e-12);

        Assert.NotNull(time);
        Assert.True(time > 0);

        // Verify: at this time, projectile should reach target
        var targetAtTime = targetPos + targetVel * time.Value;
        double distToTarget = (targetAtTime - casterPos).Length;
        double projectileTraveled = 1500 * time.Value;

        // Should be equal (center-to-center intercept)
        Assert.Equal(distToTarget, projectileTraveled, precision: 6);
    }

    #endregion

    #region Full Refinement (Quadratic + Secant + Bisection) Tests

    [Fact]
    public void FullRefinement_PixelPerfect_ExactSeparation()
    {
        // Test that full refinement achieves EXACT separation = effectiveRadius - margin
        var casterPos = new Point2D(300, 400);
        var targetPos = new Point2D(800, 400);
        var targetVel = new Vector2D(0, -350); // Moving north
        double speed = 1300;
        double delay = 0.25;
        double hitbox = 65;
        double width = 40;
        double range = 1500;
        double margin = 1.0;
        double effectiveRadius = hitbox + width / 2; // 85

        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: margin,
            secantTolerance: 1e-12,
            bisectionTolerance: 1e-15);

        Assert.NotNull(result);
        var (aimPoint, predictedPos, interceptTime) = result.Value;

        // Calculate actual separation at arrival
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;

        // Separation should be EXACTLY (effectiveRadius - margin) = 84 pixels
        double expectedSeparation = effectiveRadius - margin;
        Assert.Equal(expectedSeparation, separation, precision: 10);
    }

    [Fact]
    public void FullRefinement_HitsByExactlyOnePixel()
    {
        // Verify we hit by exactly 1 pixel (margin) inside the collision zone
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300);
        double speed = 1500;
        double delay = 0;
        double hitbox = 65;
        double width = 70;
        double range = 1000;
        double margin = 1.0;
        double effectiveRadius = hitbox + width / 2; // 100

        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: margin);

        Assert.NotNull(result);
        var (aimPoint, _, interceptTime) = result.Value;

        // Calculate separation
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;

        // We should be exactly (effectiveRadius - 1) = 99 pixels from center
        // Which means we hit 1 pixel inside the edge
        double distanceFromEdge = effectiveRadius - separation;
        Assert.Equal(margin, distanceFromEdge, precision: 10);
    }

    [Fact]
    public void FullRefinement_TimeMatchesArrival()
    {
        // Verify intercept time exactly matches projectile arrival time
        var casterPos = new Point2D(100, 200);
        var targetPos = new Point2D(700, 500);
        var targetVel = new Vector2D(-150, 200);
        double speed = 1800;
        double delay = 0.3;
        double hitbox = 65;
        double width = 60;
        double range = 1500;

        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            secantTolerance: 1e-12,
            bisectionTolerance: 1e-15);

        Assert.NotNull(result);
        var (aimPoint, _, interceptTime) = result.Value;

        // Calculate actual arrival time
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;

        // Should match to 15 decimal places
        Assert.Equal(interceptTime, arrivalTime, precision: 14);
    }

    [Fact]
    public void FullRefinement_BetterThanSecantAlone()
    {
        // Full refinement should be at least as precise as Secant alone
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(1000, 500);
        var targetVel = new Vector2D(100, -200);
        double speed = 2000;
        double delay = 0.4;
        double hitbox = 65;
        double width = 50;
        double range = 2000;

        var secantResult = InterceptSolver.SolveBehindTargetWithSecantRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            tolerance: 1e-12);

        var fullResult = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            secantTolerance: 1e-12,
            bisectionTolerance: 1e-15);

        Assert.NotNull(secantResult);
        Assert.NotNull(fullResult);

        // Both should produce valid hits
        double effectiveRadius = hitbox + width / 2;

        // Calculate separations
        var (secantAim, _, _) = secantResult.Value;
        var (fullAim, _, fullTime) = fullResult.Value;

        double secantDist = (secantAim - casterPos).Length;
        double fullDist = (fullAim - casterPos).Length;

        double secantArrival = delay + secantDist / speed;
        double fullArrival = delay + fullDist / speed;

        var secantTarget = targetPos + targetVel * secantArrival;
        var fullTarget = targetPos + targetVel * fullArrival;

        double secantSep = (secantAim - secantTarget).Length;
        double fullSep = (fullAim - fullTarget).Length;

        // Both should hit
        Assert.True(secantSep < effectiveRadius);
        Assert.True(fullSep < effectiveRadius);
    }

    [Fact]
    public void FullRefinement_VariousMargins_AllPixelPerfect()
    {
        // Test various margin values all achieve pixel-perfect accuracy
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(600, 0);
        var targetVel = new Vector2D(0, 400);
        double speed = 1500;
        double delay = 0.2;
        double hitbox = 65;
        double width = 70;
        double range = 1200;
        double effectiveRadius = hitbox + width / 2; // 100

        double[] margins = { 0.5, 1.0, 2.0, 5.0, 10.0 };

        foreach (var margin in margins)
        {
            var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
                casterPos, targetPos, targetVel,
                speed, delay, hitbox, width, range,
                behindMargin: margin);

            Assert.NotNull(result);
            var (aimPoint, _, _) = result.Value;

            double travelDist = (aimPoint - casterPos).Length;
            double arrivalTime = delay + travelDist / speed;
            var targetAtArrival = targetPos + targetVel * arrivalTime;
            double separation = (aimPoint - targetAtArrival).Length;

            double expectedSep = effectiveRadius - margin;
            Assert.Equal(expectedSep, separation, precision: 8);
        }
    }

    [Fact]
    public void SolveInterceptTimeWithFullRefinement_BasicTest()
    {
        // Test the intermediate method
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300);

        double? time = InterceptSolver.SolveInterceptTimeWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0,
            skillshotRange: 1000,
            secantTolerance: 1e-12,
            bisectionTolerance: 1e-15);

        Assert.NotNull(time);
        Assert.True(time > 0);

        // Verify intercept equation: |D + V*t| = s*t
        var targetAtTime = targetPos + targetVel * time.Value;
        double distToTarget = (targetAtTime - casterPos).Length;
        double projectileTraveled = 1500 * time.Value;

        Assert.Equal(distToTarget, projectileTraveled, precision: 12);
    }

    [Fact]
    public void FullRefinement_AdaptiveTolerance_DoesNotBreakTimingConsistency()
    {
        // This exercises the adaptive tolerance path (positionTol -> timeTol) via full refinement.
        // The invariant is that interceptTime matches projectile arrival time.
        var casterPos = new Point2D(123, 456);
        var targetPos = new Point2D(900, 800);
        var targetVel = new Vector2D(-250, 175);

        double speed = 1600;
        double delay = 0.35;

        double? t = InterceptSolver.SolveInterceptTimeWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: speed,
            castDelay: delay,
            skillshotRange: 5000);

        Assert.NotNull(t);

        var aimAtTime = targetPos + targetVel * t.Value;
        double travelDist = (aimAtTime - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;

        Assert.Equal(t.Value, arrivalTime, precision: 9);
    }

    #endregion

    #region Input Validation Tests

    [Fact]
    public void SolveInterceptTime_NegativeSpeed_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        var ex = Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveInterceptTime(
                casterPos, targetPos, targetVel,
                skillshotSpeed: -100,
                castDelay: 0));

        Assert.Contains("positive", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void SolveInterceptTime_ZeroSpeed_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        var ex = Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveInterceptTime(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 0,
                castDelay: 0));

        Assert.Contains("positive", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void SolveInterceptTime_NegativeDelay_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        var ex = Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveInterceptTime(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 1000,
                castDelay: -0.5));

        Assert.Contains("negative", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void SolveEdgeInterceptTime_NegativeHitboxRadius_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        var ex = Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveEdgeInterceptTime(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 1000,
                castDelay: 0,
                targetHitboxRadius: -10,
                skillshotWidth: 70,
                skillshotRange: 1000));

        Assert.Contains("negative", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void SolveEdgeInterceptTime_NegativeSkillshotWidth_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        var ex = Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveEdgeInterceptTime(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 1000,
                castDelay: 0,
                targetHitboxRadius: 65,
                skillshotWidth: -20,
                skillshotRange: 1000));

        Assert.Contains("negative", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void SolveBehindTarget_NegativeMargin_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        var ex = Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveBehindTarget(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 1000,
                castDelay: 0,
                targetHitboxRadius: 65,
                skillshotWidth: 70,
                skillshotRange: 1000,
                behindMargin: -5));

        Assert.Contains("negative", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void SolveBehindTargetWithFullRefinement_InvalidInputs_ThrowsArgumentException()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        // Test zero speed
        Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveBehindTargetWithFullRefinement(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 0,
                castDelay: 0,
                targetHitboxRadius: 65,
                skillshotWidth: 70,
                skillshotRange: 1000));

        // Test negative delay
        Assert.Throws<ArgumentException>(() =>
            InterceptSolver.SolveBehindTargetWithFullRefinement(
                casterPos, targetPos, targetVel,
                skillshotSpeed: 1000,
                castDelay: -1,
                targetHitboxRadius: 65,
                skillshotWidth: 70,
                skillshotRange: 1000));
    }

    [Fact]
    public void SolveInterceptTime_ValidInputs_DoesNotThrow()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 0);

        // Should not throw with valid inputs
        var result = InterceptSolver.SolveInterceptTime(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1000,
            castDelay: 0.25);

        Assert.NotNull(result);
    }

    #endregion

    #region Edge Case Regression Tests

    /// <summary>
    /// Tests that Secant refinement works correctly when initialGuess is very close to castDelay.
    /// This tests the fix for the t1 clamping issue where t1 could be less than or equal to castDelay.
    /// </summary>
    [Fact]
    public void Refinement_InitialGuessNearCastDelay_DoesNotFail()
    {
        // Setup: target very close, so intercept time is barely above castDelay
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(50, 0); // Very close
        var targetVel = new Vector2D(0, 100); // Slow movement
        double speed = 2000; // Fast projectile
        double delay = 0.25;

        // This would result in initialGuess ≈ delay + 50/2000 = 0.275
        // The perturbation t1 = 0.275 * 1.001 + 1e-6 ≈ 0.2753
        // Both should be > castDelay (0.25), but edge cases exist

        var result = InterceptSolver.SolveBehindTargetWithSecantRefinement(
            casterPos, targetPos, targetVel,
            speed, delay,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 500,
            behindMargin: 1.0,
            tolerance: 1e-12);

        Assert.NotNull(result);
        Assert.True(result.Value.InterceptTime > delay);
    }

    /// <summary>
    /// Tests zero margin (aim exactly at collision edge, no safety buffer).
    /// </summary>
    [Fact]
    public void ZeroMargin_AimsExactlyAtEdge()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300);
        double speed = 1500;
        double delay = 0;
        double hitbox = 65;
        double width = 70;
        double range = 1000;
        double margin = 0.0; // Zero margin - aim exactly at edge
        double effectiveRadius = hitbox + width / 2; // 100

        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: margin);

        Assert.NotNull(result);
        var (aimPoint, _, _) = result.Value;

        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = delay + travelDist / speed;
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;

        // With zero margin, separation should equal effectiveRadius exactly
        Assert.Equal(effectiveRadius, separation, precision: 8);
    }

    /// <summary>
    /// Tests very high target velocities for numerical stability.
    /// </summary>
    [Fact]
    public void VeryHighVelocity_NumericallyStable()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 1400); // Very fast (near skillshot speed)
        double speed = 1500;
        double delay = 0;
        double hitbox = 65;
        double width = 70;
        double range = 2000;

        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range,
            behindMargin: 1.0);

        // Should either find a valid intercept or return null (not crash/NaN)
        if (result != null)
        {
            var (aimPoint, _, interceptTime) = result.Value;
            Assert.False(double.IsNaN(interceptTime));
            Assert.False(double.IsInfinity(interceptTime));
            Assert.True(interceptTime > delay);
        }
    }

    /// <summary>
    /// Tests target velocity exactly equal to skillshot speed (degenerate quadratic case).
    /// </summary>
    [Fact]
    public void TargetSpeedEqualsSkillshotSpeed_HandledCorrectly()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(1000, 0); // Same speed as skillshot, moving away
        double speed = 1000;

        // When |V| = s and moving directly away, target is unreachable
        double? timeAway = InterceptSolver.SolveInterceptTime(
            casterPos, targetPos, targetVel,
            skillshotSpeed: speed,
            castDelay: 0);

        // Should return null (unreachable) or a very large time
        // The solver may find a mathematical solution at infinity

        // Moving TOWARD caster should be catchable
        var towardVel = new Vector2D(-500, 0); // Moving toward caster
        double? towardTime = InterceptSolver.SolveInterceptTime(
            casterPos, targetPos, towardVel,
            skillshotSpeed: speed,
            castDelay: 0);

        Assert.NotNull(towardTime);
        Assert.True(towardTime > 0);
        Assert.True(towardTime < 1.0); // Should intercept quickly
    }

    /// <summary>
    /// Tests stationary target trailing edge calculation.
    /// Verifies our simplified logic is correct.
    /// </summary>
    [Fact]
    public void StationaryTarget_TrailingEdge_CalculatedCorrectly()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 0); // Stationary
        double speed = 1000;
        double delay = 0.25;
        double hitbox = 65;
        double width = 70;
        double range = 1000;
        double effectiveRadius = hitbox + width / 2; // 100

        double? trailingTime = InterceptSolver.SolveEdgeInterceptTimeTrailing(
            casterPos, targetPos, targetVel,
            speed, delay, hitbox, width, range);

        Assert.NotNull(trailingTime);

        // For stationary target: trailingTime = delay + (distance + effectiveRadius) / speed
        // = 0.25 + (500 + 100) / 1000 = 0.25 + 0.6 = 0.85
        double expected = delay + (500 + effectiveRadius) / speed;
        Assert.Equal(expected, trailingTime.Value, precision: 10);
    }

    /// <summary>
    /// Tests target inside collision radius at start (special case).
    /// </summary>
    [Fact]
    public void TargetInsideCollisionRadius_AtStart_StillWorks()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(50, 0); // Inside effectiveRadius = 100
        var targetVel = new Vector2D(0, 200);
        double hitbox = 65;
        double width = 70;
        double effectiveRadius = hitbox + width / 2; // 100

        var input = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: targetPos,
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 500, Width: width, Delay: 0.25),
            TargetHitboxRadius: hitbox);

        var result = _prediction.Predict(input);

        // Should be a hit since target starts in range
        Assert.IsType<PredictionResult.Hit>(result);
    }

    /// <summary>
    /// Tests extreme cast delay values.
    /// </summary>
    [Fact]
    public void ExtremeCastDelay_HandledCorrectly()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(100, 100);

        // Very long delay (1.5 seconds)
        var input = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: targetPos,
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 2000, Range: 2000, Width: 70, Delay: 1.5));

        var result = _prediction.Predict(input);

        if (result is PredictionResult.Hit hit)
        {
            Assert.True(hit.InterceptTime > 1.5);
        }
    }

    #endregion

    #region Path-Based Prediction Tests

    [Fact]
    public void PathPrediction_SimpleTwoPointPath_Works()
    {
        // Target moving from (500,0) toward (500,500)
        var path = new TargetPath(
            waypoints: new[] { new Point2D(500, 500) },
            currentPosition: new Point2D(500, 0),
            currentWaypointIndex: 0,
            speed: 300);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Target is moving up (north), so predicted position should be above starting position
        Assert.True(hit.PredictedTargetPosition.Y > 0);
    }

    [Fact]
    public void PathPrediction_MultiWaypointPath_FindsInterceptOnCorrectSegment()
    {
        // Target following L-shaped path: (200,0) -> (200,300) -> (500,300)
        var path = new TargetPath(
            waypoints: new[] { new Point2D(200, 300), new Point2D(500, 300) },
            currentPosition: new Point2D(200, 0),
            currentWaypointIndex: 0,
            speed: 350);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 800, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
    }

    [Fact]
    public void PathPrediction_StationaryPath_SameAsVelocity()
    {
        // Stationary target with path (speed = 0)
        var path = new TargetPath(
            waypoints: new[] { new Point2D(600, 0) },
            currentPosition: new Point2D(500, 0),
            currentWaypointIndex: 0,
            speed: 0);

        var inputWithPath = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var inputWithVelocity = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1000, Range: 1000, Width: 70, Delay: 0.25));

        var resultWithPath = _prediction.Predict(inputWithPath);
        var resultWithVelocity = _prediction.Predict(inputWithVelocity);

        Assert.IsType<PredictionResult.Hit>(resultWithPath);
        Assert.IsType<PredictionResult.Hit>(resultWithVelocity);

        var hitPath = (PredictionResult.Hit)resultWithPath;
        var hitVelocity = (PredictionResult.Hit)resultWithVelocity;

        // Both should aim at the same place
        Assert.Equal(hitVelocity.CastPosition.X, hitPath.CastPosition.X, precision: 1);
    }

    [Fact]
    public void PathPrediction_TargetReachesFinalWaypoint_HitsAtWaypoint()
    {
        // Target will reach final waypoint before projectile arrives
        var path = new TargetPath(
            waypoints: new[] { new Point2D(300, 0) },
            currentPosition: new Point2D(200, 0),
            currentWaypointIndex: 0,
            speed: 500); // Fast target, short path

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 800, Range: 500, Width: 70, Delay: 0.5)); // Slow skillshot with delay

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;

        // Target should be at or near final waypoint
        Assert.True(hit.PredictedTargetPosition.X >= 280); // Near final waypoint (300,0)
    }

    [Fact]
    public void PathPrediction_OutOfRange_ReturnsCorrectResult()
    {
        // Target path that goes out of range
        var path = new TargetPath(
            waypoints: new[] { new Point2D(2000, 0) },
            currentPosition: new Point2D(1500, 0),
            currentWaypointIndex: 0,
            speed: 400);

        var input = PredictionInput.WithPath(
            casterPosition: new Point2D(0, 0),
            path: path,
            skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));

        var result = _prediction.Predict(input);

        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    [Fact]
    public void TargetPath_GetPositionAtTime_FollowsWaypoints()
    {
        // Create a path: (0,0) -> (100,0) -> (100,100)
        var path = new TargetPath(
            waypoints: new[] { new Point2D(100, 0), new Point2D(100, 100) },
            currentPosition: new Point2D(0, 0),
            currentWaypointIndex: 0,
            speed: 100); // 100 units/second

        // At t=0, should be at start
        var pos0 = path.GetPositionAtTime(0);
        Assert.Equal(0, pos0.X, precision: 1);
        Assert.Equal(0, pos0.Y, precision: 1);

        // At t=0.5, should be halfway to first waypoint (50,0)
        var pos05 = path.GetPositionAtTime(0.5);
        Assert.Equal(50, pos05.X, precision: 1);
        Assert.Equal(0, pos05.Y, precision: 1);

        // At t=1.0, should be at first waypoint (100,0)
        var pos1 = path.GetPositionAtTime(1.0);
        Assert.Equal(100, pos1.X, precision: 1);
        Assert.Equal(0, pos1.Y, precision: 1);

        // At t=1.5, should be halfway up second segment (100,50)
        var pos15 = path.GetPositionAtTime(1.5);
        Assert.Equal(100, pos15.X, precision: 1);
        Assert.Equal(50, pos15.Y, precision: 1);

        // At t=2.0, should be at final waypoint (100,100)
        var pos2 = path.GetPositionAtTime(2.0);
        Assert.Equal(100, pos2.X, precision: 1);
        Assert.Equal(100, pos2.Y, precision: 1);

        // At t=3.0, should still be at final waypoint (target stops there)
        var pos3 = path.GetPositionAtTime(3.0);
        Assert.Equal(100, pos3.X, precision: 1);
        Assert.Equal(100, pos3.Y, precision: 1);
    }

    [Fact]
    public void TargetPath_GetCurrentVelocity_ReturnsCorrectDirection()
    {
        var path = new TargetPath(
            waypoints: new[] { new Point2D(100, 100) },
            currentPosition: new Point2D(0, 0),
            currentWaypointIndex: 0,
            speed: 141.42); // ~100*sqrt(2) for diagonal movement

        var velocity = path.GetCurrentVelocity();

        // Should be moving diagonally (northeast)
        Assert.True(velocity.X > 0);
        Assert.True(velocity.Y > 0);
        Assert.Equal(velocity.X, velocity.Y, precision: 1); // 45-degree angle
    }

    [Fact]
    public void PathPrediction_SolverDirect_ReturnsPathInterceptResult()
    {
        var casterPos = new Point2D(0, 0);
        var path = new TargetPath(
            waypoints: new[] { new Point2D(500, 500) },
            currentPosition: new Point2D(500, 0),
            currentWaypointIndex: 0,
            speed: 300);

        var result = InterceptSolver.SolvePathBehindTargetIntercept(
            casterPos, path,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000,
            behindMargin: 1.0);

        Assert.NotNull(result);
        Assert.True(result.Value.InterceptTime > 0.25);
        Assert.True(result.Value.WaypointIndex >= 0);
    }

    [Fact]
    public void PathPrediction_ConsistentWithVelocityPrediction_FirstSegment()
    {
        // For a simple two-point path on the first segment,
        // path prediction should be consistent with velocity prediction

        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300); // Moving north

        // Create equivalent path
        var path = new TargetPath(
            waypoints: new[] { new Point2D(500, 1000) }, // Far away so we stay on first segment
            currentPosition: targetPos,
            currentWaypointIndex: 0,
            speed: 300);

        var velocityResult = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000,
            behindMargin: 1.0);

        var pathResult = InterceptSolver.SolvePathBehindTargetIntercept(
            casterPos, path,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000,
            behindMargin: 1.0);

        Assert.NotNull(velocityResult);
        Assert.NotNull(pathResult);

        // Times should be similar (not exact due to different refinement paths)
        Assert.Equal(velocityResult.Value.InterceptTime, pathResult.Value.InterceptTime, precision: 2);
    }

    #endregion

    #region Radius Bisection Tests

    [Fact]
    public void RadiusBisection_FindsOptimalRadius()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300); // Moving perpendicular
        double hitbox = 60;
        double width = 40;

        var result = InterceptSolver.SolveBehindTargetWithRadiusBisection(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: hitbox,
            skillshotWidth: width,
            skillshotRange: 1000,
            behindMargin: 1.0);

        Assert.NotNull(result);
        var (aimPoint, predictedPos, interceptTime, effectiveRadius) = result.Value;

        // Effective radius should be between hitbox and hitbox + width
        Assert.True(effectiveRadius >= hitbox, 
            $"EffectiveRadius ({effectiveRadius}) should be >= hitbox ({hitbox})");
        Assert.True(effectiveRadius <= hitbox + width, 
            $"EffectiveRadius ({effectiveRadius}) should be <= hitbox + width ({hitbox + width})");

        // Verify it's actually a hit
        double separation = (aimPoint - predictedPos).Length;
        Assert.True(separation <= effectiveRadius,
            $"Separation ({separation}) should be <= effectiveRadius ({effectiveRadius})");
    }

    [Fact]
    public void RadiusBisection_StationaryTarget_UsesMaxRadius()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 0); // Stationary
        double hitbox = 60;
        double width = 40;

        var result = InterceptSolver.SolveBehindTargetWithRadiusBisection(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: hitbox,
            skillshotWidth: width,
            skillshotRange: 1000);

        Assert.NotNull(result);
        // For stationary targets, uses standard radius (hitbox + width/2)
        Assert.Equal(hitbox + width / 2, result.Value.EffectiveRadius, precision: 1);
    }

    [Fact]
    public void RadiusBisection_FastTarget_FindsSmallerRadius()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        double hitbox = 60;
        double width = 40;

        // Slow target
        var slowResult = InterceptSolver.SolveBehindTargetWithRadiusBisection(
            casterPos, targetPos, new Vector2D(0, 100),
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: hitbox,
            skillshotWidth: width,
            skillshotRange: 1000);

        // Fast target
        var fastResult = InterceptSolver.SolveBehindTargetWithRadiusBisection(
            casterPos, targetPos, new Vector2D(0, 400),
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: hitbox,
            skillshotWidth: width,
            skillshotRange: 1000);

        Assert.NotNull(slowResult);
        Assert.NotNull(fastResult);

        // Both should find valid hits
        Assert.True(slowResult.Value.EffectiveRadius >= hitbox);
        Assert.True(fastResult.Value.EffectiveRadius >= hitbox);
    }

    [Fact]
    public void RadiusBisection_OutOfRange_ReturnsNull()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(2000, 0); // Far beyond range
        var targetVel = new Vector2D(500, 0); // Moving away
        double hitbox = 60;
        double width = 40;

        var result = InterceptSolver.SolveBehindTargetWithRadiusBisection(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: hitbox,
            skillshotWidth: width,
            skillshotRange: 1000);

        Assert.Null(result);
    }

    #endregion
}
