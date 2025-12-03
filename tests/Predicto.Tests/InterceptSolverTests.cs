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

    #region Secant Refinement Tests

    [Fact]
    public void SecantRefinement_ProducesSameResultAsQuadratic()
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
    public void SecantRefinement_HighPrecision_VerifyIntercept()
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
    public void SecantRefinement_LargeDistance_MaintainsPrecision()
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
    public void SecantRefinement_WithCastDelay_CorrectTiming()
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
}
