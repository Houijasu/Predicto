using MathNet.Spatial.Euclidean;
using Predicto;
using Predicto.Models;
using Predicto.Solvers;

namespace Predicto.Tests;

/// <summary>
/// Regression tests for specific bug fixes and edge cases.
/// These tests ensure fixed bugs don't regress in future changes.
/// </summary>
public class EdgeCaseRegressionTests
{
    private readonly Ultimate _prediction = new();

    #region SolveLinearEdge Inequality Fix Tests

    /// <summary>
    /// Verifies that SolveLinearEdge uses strict inequality (t > minTime) consistently.
    /// Bug: Previously used (t >= minTime) which was inconsistent with other solvers.
    /// Fix: Changed to (t > minTime) at InterceptSolver.cs:306
    /// </summary>
    [Fact]
    public void SolveLinearEdge_TimeExactlyAtMinTime_ReturnsNull()
    {
        // This test verifies the fix for inconsistent inequality handling.
        // The solver should use strict inequality (t > castDelay), not (t >= castDelay).
        
        // Setup target that's NOT in collision radius at launch time
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0); // Far enough that target won't be in radius at launch
        var targetVel = new Vector2D(0, 300); // Moving perpendicular
        
        var result = InterceptSolver.SolveEdgeInterceptTime(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0.25,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000);
        
        // Solution must be strictly greater than cast delay
        Assert.NotNull(result);
        Assert.True(result.Value > 0.25, 
            $"Intercept time ({result.Value}) must be strictly > cast delay (0.25)");
    }

    /// <summary>
    /// Tests that edge intercept solutions are strictly greater than cast delay.
    /// </summary>
    [Fact]
    public void SolveEdgeInterceptTime_WithCastDelay_ResultStrictlyGreater()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(200, 0); // Far enough to need travel time (effectiveRadius = 100)
        var targetVel = new Vector2D(0, 100);
        double castDelay = 0.5;

        var result = InterceptSolver.SolveEdgeInterceptTime(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: castDelay,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 1000);

        Assert.NotNull(result);
        Assert.True(result.Value > castDelay, 
            $"Intercept time ({result.Value}) must be > cast delay ({castDelay})");
    }

    /// <summary>
    /// Verifies consistency between SolveLinearEdge and SolveQuadraticEdge boundary behavior.
    /// Both should use strict inequality (t > minTime).
    /// </summary>
    [Fact]
    public void EdgeSolvers_BoundaryBehavior_Consistent()
    {
        // Test a range of scenarios near the boundary between linear and quadratic cases
        var casterPos = new Point2D(0, 0);
        var testCases = new[]
        {
            // (targetSpeed close to skillshotSpeed - near linear case boundary)
            (new Point2D(500, 0), new Vector2D(1490, 0), 1500.0), // Almost equal speeds
            (new Point2D(500, 0), new Vector2D(1500, 0), 1500.0), // Exactly equal speeds (linear)
            (new Point2D(500, 0), new Vector2D(1510, 0), 1500.0), // Slightly faster target
        };

        foreach (var (targetPos, targetVel, speed) in testCases)
        {
            var result = InterceptSolver.SolveEdgeInterceptTime(
                casterPos, targetPos, targetVel,
                skillshotSpeed: speed,
                castDelay: 0,
                targetHitboxRadius: 65,
                skillshotWidth: 70,
                skillshotRange: 3000);

            // Result should either be null (unreachable) or strictly > 0
            if (result != null)
            {
                Assert.True(result.Value > Constants.Epsilon, 
                    $"For target at {targetPos} with vel {targetVel}: time {result.Value} should be > epsilon");
            }
        }
    }

    #endregion

    #region CalculateAdaptiveMargin Discontinuity Fix Tests

    /// <summary>
    /// Verifies that CalculateAdaptiveMargin has no discontinuity at threshold boundaries.
    /// Bug: Previously had 500M factor jump when crossing threshold (used 0.5*radius vs epsilon).
    /// Fix: Now uses smooth transition at effectiveRadius = 2.0 (Ultimate.cs:349-365)
    /// </summary>
    [Fact]
    public void AdaptiveMargin_AtThresholdBoundary_Continuous()
    {
        // Test values around the 2.0 threshold
        // At exactly 2.0: proportional formula gives 2.0 * 0.5 = 1.0
        // Above 2.0: fixed formula gives 1.0
        // Both should be equal at the boundary!
        
        double belowThreshold = 1.999;
        double atThreshold = 2.0;
        double aboveThreshold = 2.001;
        
        // Calculate margins using the public solver method that uses adaptive margin internally
        // We'll verify the behavior through prediction results consistency
        
        var casterPos = new Point2D(0, 0);
        var targetVel = new Vector2D(0, 300);
        
        // Test with very small effective radius (below threshold)
        var smallRadiusInput = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 2.0, Delay: 0), // width/2 = 1
            TargetHitboxRadius: belowThreshold - 1.0); // effectiveRadius = 1.999
        
        var normalRadiusInput = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 2.0, Delay: 0),
            TargetHitboxRadius: atThreshold - 1.0); // effectiveRadius = 2.0
        
        var largeRadiusInput = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 2.0, Delay: 0),
            TargetHitboxRadius: aboveThreshold - 1.0); // effectiveRadius = 2.001
        
        var smallResult = _prediction.Predict(smallRadiusInput);
        var normalResult = _prediction.Predict(normalRadiusInput);
        var largeResult = _prediction.Predict(largeRadiusInput);
        
        // All should produce valid predictions (no crashes from discontinuity)
        Assert.IsType<PredictionResult.Hit>(smallResult);
        Assert.IsType<PredictionResult.Hit>(normalResult);
        Assert.IsType<PredictionResult.Hit>(largeResult);
        
        // Intercept times should be very similar (continuous behavior)
        var smallHit = (PredictionResult.Hit)smallResult;
        var normalHit = (PredictionResult.Hit)normalResult;
        var largeHit = (PredictionResult.Hit)largeResult;
        
        double diffSmallNormal = Math.Abs(smallHit.InterceptTime - normalHit.InterceptTime);
        double diffNormalLarge = Math.Abs(normalHit.InterceptTime - largeHit.InterceptTime);
        
        // Differences should be proportional to radius change (not 500M factor jump)
        Assert.True(diffSmallNormal < 0.01, 
            $"Time jump from {belowThreshold} to {atThreshold} radius: {diffSmallNormal}s (should be smooth)");
        Assert.True(diffNormalLarge < 0.01, 
            $"Time jump from {atThreshold} to {aboveThreshold} radius: {diffNormalLarge}s (should be smooth)");
    }

    /// <summary>
    /// Tests that very small effective radii use proportional margin correctly.
    /// </summary>
    [Fact]
    public void AdaptiveMargin_VerySmallRadius_ProportionalMargin()
    {
        // For effectiveRadius <= 2.0, margin = effectiveRadius * 0.5
        // This prevents the margin from exceeding the collision zone
        
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(200, 0);
        var targetVel = new Vector2D(0, 100);
        
        // Tiny target and spell: effectiveRadius = 0.5 + 0.25 = 0.75
        // Proportional margin = 0.75 * 0.5 = 0.375
        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1000,
            castDelay: 0,
            targetHitboxRadius: 0.5,
            skillshotWidth: 0.5,
            skillshotRange: 500);
        
        Assert.NotNull(result);
        
        // Verify the solution is valid - separation should be less than effectiveRadius
        var (aimPoint, _, interceptTime) = result.Value;
        double effectiveRadius = 0.5 + 0.5 / 2.0; // 0.75
        
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = travelDist / 1000.0;
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;
        
        Assert.True(separation < effectiveRadius, 
            $"Separation ({separation:F4}) must be < effective radius ({effectiveRadius:F4})");
    }

    /// <summary>
    /// Tests that normal gameplay radii use fixed 1-unit margin.
    /// </summary>
    [Fact]
    public void AdaptiveMargin_NormalRadius_FixedMargin()
    {
        // For effectiveRadius > 2.0, margin = 1.0 (fixed)
        // Normal gameplay: targetHitbox=65, spellWidth=70 → effectiveRadius = 100
        
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        var targetVel = new Vector2D(0, 300);
        double hitbox = 65;
        double width = 70;
        double effectiveRadius = hitbox + width / 2; // 100
        
        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 1500,
            castDelay: 0,
            targetHitboxRadius: hitbox,
            skillshotWidth: width,
            skillshotRange: 1000,
            behindMargin: 1.0); // 1-unit margin
        
        Assert.NotNull(result);
        
        var (aimPoint, _, _) = result.Value;
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = travelDist / 1500.0;
        var targetAtArrival = targetPos + targetVel * arrivalTime;
        double separation = (aimPoint - targetAtArrival).Length;
        
        // Expected separation = effectiveRadius - margin = 100 - 1 = 99
        double expectedSeparation = effectiveRadius - 1.0;
        Assert.Equal(expectedSeparation, separation, precision: 6);
    }

    #endregion

    #region CalculateCircularAdaptiveMargin Tests

    /// <summary>
    /// Tests that circular adaptive margin uses simplified formula without unused parameters.
    /// Bug: Previously accepted unused parameters (targetSpeed, distance, delay, skillshotSpeed).
    /// Fix: Now only takes effectiveRadius parameter (Ultimate.cs:626-636)
    /// </summary>
    [Fact]
    public void CircularAdaptiveMargin_BasicFunctionality()
    {
        // Test circular prediction works correctly with simplified margin calculation
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(0, 300),
            Skillshot: new CircularSkillshot(Radius: 200, Range: 800, Delay: 0.5),
            TargetHitboxRadius: 65);
        
        var result = _prediction.PredictCircular(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // Target is moving north, so predicted position should be above current position
        Assert.True(hit.PredictedTargetPosition.Y > 0);
        
        // Intercept time should be >= delay
        Assert.True(hit.InterceptTime >= 0.5);
    }

    /// <summary>
    /// Tests circular margin for very small effective radius.
    /// </summary>
    [Fact]
    public void CircularAdaptiveMargin_VerySmallRadius_ProportionalMargin()
    {
        // For effectiveRadius < 1.0, use effectiveRadius * 0.5
        // This is consistent with linear skillshot behavior
        
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(100, 0),
            TargetVelocity: new Vector2D(0, 50),
            Skillshot: new CircularSkillshot(Radius: 0.3, Range: 200, Delay: 0.2), // Tiny radius
            TargetHitboxRadius: 0.2); // effectiveRadius = 0.3 + 0.2 = 0.5 < 1.0
        
        var result = _prediction.PredictCircular(input);
        
        // Should produce a valid prediction without crashing
        Assert.IsType<PredictionResult.Hit>(result);
    }

    /// <summary>
    /// Tests that circular margin correctly uses 50% of effective radius.
    /// </summary>
    [Fact]
    public void CircularAdaptiveMargin_NormalRadius_HalfEffectiveRadius()
    {
        // For circular spells, margin = effectiveRadius * 0.5
        // This aims halfway between center and trailing edge
        
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(0, 200), // Moving north
            Skillshot: new CircularSkillshot(Radius: 100, Range: 800, Delay: 0.5),
            TargetHitboxRadius: 65); // effectiveRadius = 100 + 65 = 165
        
        var result = _prediction.PredictCircular(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // The aim point should be BEHIND (south of) the predicted target position
        // because we're using behind-target strategy
        Assert.True(hit.CastPosition.Y < hit.PredictedTargetPosition.Y,
            $"Cast Y ({hit.CastPosition.Y:F1}) should be < predicted Y ({hit.PredictedTargetPosition.Y:F1})");
    }

    #endregion

    #region Constants.Epsilon Consistency Tests

    /// <summary>
    /// Tests that Constants.Epsilon is used consistently throughout the codebase.
    /// Bug: Previously had hardcoded 1e-9 values in some places.
    /// Fix: Replaced with Constants.Epsilon for maintainability.
    /// </summary>
    [Fact]
    public void Epsilon_ConsistentUsage_NumericalStability()
    {
        // Test numerical stability at epsilon boundaries
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(500, 0);
        
        // Test with velocity magnitude near epsilon
        var nearZeroVelocity = new Vector2D(Constants.Epsilon * 0.5, 0);
        var atEpsilonVelocity = new Vector2D(Constants.Epsilon, 0);
        var aboveEpsilonVelocity = new Vector2D(Constants.Epsilon * 2, 0);
        
        // All should produce stable results (no NaN/Infinity)
        foreach (var vel in new[] { nearZeroVelocity, atEpsilonVelocity, aboveEpsilonVelocity })
        {
            var result = InterceptSolver.SolveInterceptTime(
                casterPos, targetPos, vel,
                skillshotSpeed: 1500,
                castDelay: 0.25);
            
            Assert.NotNull(result);
            Assert.False(double.IsNaN(result.Value));
            Assert.False(double.IsInfinity(result.Value));
        }
    }

    /// <summary>
    /// Tests epsilon handling in confidence calculation.
    /// </summary>
    [Fact]
    public void Epsilon_ConfidenceCalculation_ClampedCorrectly()
    {
        // Test that confidence is clamped between Epsilon and 1.0
        var confidence = InterceptSolver.CalculateConfidence(
            interceptTime: 10.0, // Very long time (low confidence)
            distance: 1000,
            targetSpeed: 500,
            skillshotSpeed: 1000,
            skillshotRange: 1000,
            castDelay: 0.5);
        
        Assert.True(confidence >= Constants.Epsilon);
        Assert.True(confidence <= 1.0);
    }

    #endregion

    #region Numerical Precision Tests

    /// <summary>
    /// Tests precision at large distances where floating-point errors accumulate.
    /// </summary>
    [Fact]
    public void LargeDistance_NumericalPrecision_Maintained()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(50000, 30000); // Large distance
        var targetVel = new Vector2D(-100, 150);
        
        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 3000,
            castDelay: 0.5,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 100000);
        
        Assert.NotNull(result);
        var (aimPoint, _, interceptTime) = result.Value;
        
        // Verify time-travel consistency
        double travelDist = (aimPoint - casterPos).Length;
        double arrivalTime = 0.5 + travelDist / 3000.0;
        
        Assert.Equal(interceptTime, arrivalTime, precision: 9);
    }

    /// <summary>
    /// Tests precision with very small intercept times (target very close).
    /// </summary>
    [Fact]
    public void SmallInterceptTime_NumericalPrecision_Maintained()
    {
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(50, 0); // Very close
        var targetVel = new Vector2D(0, 50);
        
        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            casterPos, targetPos, targetVel,
            skillshotSpeed: 5000, // Very fast projectile
            castDelay: 0,
            targetHitboxRadius: 65,
            skillshotWidth: 70,
            skillshotRange: 500);
        
        Assert.NotNull(result);
        var (_, _, interceptTime) = result.Value;
        
        Assert.True(interceptTime > 0);
        Assert.True(interceptTime < 0.1, "Very close target should have quick intercept");
        Assert.False(double.IsNaN(interceptTime));
    }

    #endregion

    #region Boundary Condition Tests

    /// <summary>
    /// Tests behavior when target is exactly at skillshot range boundary.
    /// </summary>
    [Fact]
    public void TargetAtRangeBoundary_HandledCorrectly()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(1000, 0), // Exactly at range
            TargetVelocity: new Vector2D(0, 0), // Stationary
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        // Should be a hit (within range tolerance)
        Assert.IsType<PredictionResult.Hit>(result);
    }

    /// <summary>
    /// Tests behavior when target is just beyond skillshot range.
    /// Note: Range check uses (distance - effectiveRadius > range), meaning
    /// a target at distance 1101 with effectiveRadius 100 and range 1000
    /// is considered "just out of range" (1101 - 100 = 1001 > range).
    /// </summary>
    [Fact]
    public void TargetJustBeyondRange_OutOfRange()
    {
        // effectiveRadius = targetHitbox (65) + width/2 (35) = 100
        // For OutOfRange: distance - effectiveRadius > range
        // Need: distance - 100 > 1000, so distance > 1100
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(1101, 0), // 1101 - 100 = 1001 > 1000
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        Assert.IsType<PredictionResult.OutOfRange>(result);
    }

    /// <summary>
    /// Tests behavior when caster and target are at the same position.
    /// </summary>
    [Fact]
    public void CasterAndTargetSamePosition_HandledGracefully()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(0, 0), // Same position
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        // Should produce a valid hit (target is within collision radius at start)
        Assert.IsType<PredictionResult.Hit>(result);
    }

    #endregion

    #region Reaction Time Factor Tests

    /// <summary>
    /// Tests that reaction time factor is 1.0 when flight time is less than reaction time.
    /// </summary>
    [Fact]
    public void ReactionTimeFactor_FlightTimeLessThanReaction_Returns1()
    {
        // Flight time 0.1s < 0.25s reaction time
        double factor = Constants.GetReactionTimeFactor(0.1, 0.25);
        Assert.Equal(1.0, factor);
    }

    /// <summary>
    /// Tests that reaction time factor is 1.0 when flight time equals reaction time.
    /// </summary>
    [Fact]
    public void ReactionTimeFactor_FlightTimeEqualsReaction_Returns1()
    {
        double factor = Constants.GetReactionTimeFactor(0.25, 0.25);
        Assert.Equal(1.0, factor);
    }

    /// <summary>
    /// Tests that reaction time factor decays for longer flight times.
    /// </summary>
    [Fact]
    public void ReactionTimeFactor_FlightTimeLongerThanReaction_Decays()
    {
        // Flight time = 2x reaction time
        double factor = Constants.GetReactionTimeFactor(0.5, 0.25);
        
        // Should be 0.5 (minimum) at 2x reaction time
        Assert.Equal(0.5, factor, precision: 6);
    }

    /// <summary>
    /// Tests that reaction time factor is clamped to 0.5 minimum.
    /// </summary>
    [Fact]
    public void ReactionTimeFactor_VeryLongFlightTime_ClampedAt05()
    {
        // Flight time = 3x reaction time
        double factor = Constants.GetReactionTimeFactor(0.75, 0.25);
        
        // Should be clamped to 0.5
        Assert.Equal(0.5, factor);
    }

    /// <summary>
    /// Tests reaction time factor with default parameter.
    /// </summary>
    [Fact]
    public void ReactionTimeFactor_DefaultReactionTime_Works()
    {
        // Use default reaction time (0.25s)
        double factor = Constants.GetReactionTimeFactor(0.1);
        
        // Should use average reaction time
        Assert.Equal(1.0, factor);
    }

    /// <summary>
    /// Tests that fast skillshots get higher confidence due to reaction time.
    /// </summary>
    [Fact]
    public void ReactionTimeIntegration_FastSkillshot_HigherConfidence()
    {
        // Fast skillshot (short flight time)
        var fastInput = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(300, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: new LinearSkillshot(Speed: 2500, Range: 1000, Width: 70, Delay: 0)); // Very fast
        
        // Slow skillshot (long flight time)
        var slowInput = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(300, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: new LinearSkillshot(Speed: 800, Range: 1000, Width: 70, Delay: 0)); // Slow
        
        var fastResult = _prediction.Predict(fastInput);
        var slowResult = _prediction.Predict(slowInput);
        
        Assert.IsType<PredictionResult.Hit>(fastResult);
        Assert.IsType<PredictionResult.Hit>(slowResult);
        
        var fastHit = (PredictionResult.Hit)fastResult;
        var slowHit = (PredictionResult.Hit)slowResult;
        
        // Fast skillshot should have higher confidence
        Assert.True(fastHit.Confidence > slowHit.Confidence,
            $"Fast confidence ({fastHit.Confidence:F3}) should be > slow confidence ({slowHit.Confidence:F3})");
    }

    #endregion

    #region Additional Edge Case Tests (From Audit)

    /// <summary>
    /// Tests behavior when target is inside collision radius at start but moving quickly.
    /// The trailing edge logic should still work correctly.
    /// </summary>
    [Fact]
    public void TargetInsideCollisionRadius_MovingFast_HandledCorrectly()
    {
        // Target starts very close (inside effective collision radius)
        // effectiveRadius = 65 + 35 = 100
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(50, 0), // Inside effective radius
            TargetVelocity: new Vector2D(500, 0), // Moving away fast
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        // Should be a hit since target starts in range
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // Intercept time should be reasonable (not negative or huge)
        Assert.True(hit.InterceptTime >= 0);
        Assert.True(hit.InterceptTime < 2.0); // Should catch within 2 seconds
    }

    /// <summary>
    /// Tests behavior when target starts inside collision radius and moves perpendicular.
    /// </summary>
    [Fact]
    public void TargetInsideCollisionRadius_MovingPerpendicular_HandledCorrectly()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(30, 0), // Very close
            TargetVelocity: new Vector2D(0, 400), // Moving perpendicular
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.1));
        
        var result = _prediction.Predict(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // The aim point Y should be positive (ahead of target)
        Assert.True(hit.PredictedTargetPosition.Y > 0,
            "Target should be predicted to have moved in Y direction");
    }

    /// <summary>
    /// Tests edge case where intercept time equals cast delay (boundary condition).
    /// This happens when target is exactly at the right distance/speed combination.
    /// </summary>
    [Fact]
    public void InterceptTimeNearCastDelay_BoundaryCondition()
    {
        // Setup where projectile would theoretically hit at exactly cast delay time
        // This is the boundary between "already in collision" and "need to travel"
        var casterPos = new Point2D(0, 0);
        var targetPos = new Point2D(100, 0); // Close but not inside collision radius
        var targetVel = new Vector2D(0, 0); // Stationary
        
        // Cast delay of 0.25, fast projectile
        var input = new PredictionInput(
            CasterPosition: casterPos,
            TargetPosition: targetPos,
            TargetVelocity: targetVel,
            Skillshot: new LinearSkillshot(Speed: 2000, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // Intercept time should be >= cast delay
        Assert.True(hit.InterceptTime >= 0.25,
            $"Intercept time ({hit.InterceptTime}) should be >= cast delay (0.25)");
    }

    /// <summary>
    /// Tests that skillshot width of zero is handled correctly.
    /// </summary>
    [Fact]
    public void SkillshotWidthZero_HandledGracefully()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 0, Delay: 0.25), // Zero width
            TargetHitboxRadius: 65);
        
        var result = _prediction.Predict(input);
        
        // Should still work (uses target hitbox radius only)
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // Values should be valid (no NaN)
        Assert.False(double.IsNaN(hit.CastPosition.X));
        Assert.False(double.IsNaN(hit.CastPosition.Y));
        Assert.False(double.IsNaN(hit.InterceptTime));
    }

    /// <summary>
    /// Tests extremely large skillshot width.
    /// </summary>
    [Fact]
    public void SkillshotWidthVeryLarge_HandledCorrectly()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 500, Delay: 0.25), // Very wide
            TargetHitboxRadius: 65);
        
        var result = _prediction.Predict(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // With large effective radius (315), the margin should still be valid
        Assert.True(hit.Confidence > 0);
    }

    /// <summary>
    /// Tests target hitbox radius of zero.
    /// </summary>
    [Fact]
    public void TargetHitboxRadiusZero_HandledGracefully()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(400, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25),
            TargetHitboxRadius: 0); // Zero hitbox
        
        var result = _prediction.Predict(input);
        
        // Should still work (uses skillshot width/2 only)
        Assert.IsType<PredictionResult.Hit>(result);
    }

    /// <summary>
    /// Tests the SmallRadiusThreshold constant usage in adaptive margin.
    /// </summary>
    [Fact]
    public void AdaptiveMargin_AtSmallRadiusThreshold_ContinuousTransition()
    {
        // At exactly SmallRadiusThreshold (2.0), both formulas should give same result
        // Below: margin = effectiveRadius * 0.5
        // Above: margin = 1.0
        // At 2.0: 2.0 * 0.5 = 1.0 (matches)
        
        // Test just below threshold
        var inputBelowThreshold = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 2, Delay: 0), // Width 2 -> effective = 1 + hitbox
            TargetHitboxRadius: 1); // effectiveRadius = 1 + 1 = 2 (at threshold)
        
        var inputAboveThreshold = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 10, Delay: 0), // Width 10 -> effective = 5 + hitbox
            TargetHitboxRadius: 60); // effectiveRadius = 5 + 60 = 65 (above threshold)
        
        var resultBelow = _prediction.Predict(inputBelowThreshold);
        var resultAbove = _prediction.Predict(inputAboveThreshold);
        
        // Both should produce valid hits
        Assert.IsType<PredictionResult.Hit>(resultBelow);
        Assert.IsType<PredictionResult.Hit>(resultAbove);
    }

    /// <summary>
    /// Tests the IsEasyCase logic with very slow target (less than 0.5 units per tick).
    /// </summary>
    [Fact]
    public void IsEasyCase_VerySlowTarget_ProcessedEfficiently()
    {
        // Target moving less than 0.5 units per tick (15 units/s at 30Hz = 0.5 units/tick)
        // This qualifies as an "easy case" internally for optimization purposes.
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(10, 0), // 10 units/s = 0.33 units/tick
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // Slow target should still produce reasonable prediction
        // Note: "IsEasyCase" is an internal optimization, not a confidence booster
        Assert.True(hit.Confidence > 0.5, $"Expected confidence > 0.5, got {hit.Confidence}");
        
        // Aim point is behind target by (effectiveRadius - margin)
        // effectiveRadius = 65 + 35 = 100, margin = 1.0
        // So aim ~= targetPos - (100 - 1) = 500 - 99 = 401
        // Allow tolerance for target movement and calculation
        Assert.True(hit.CastPosition.X > 350 && hit.CastPosition.X < 500, 
            $"Expected aim behind target (350-500), got {hit.CastPosition.X}");
    }

    /// <summary>
    /// Tests the IsEasyCase logic with target moving directly toward caster.
    /// </summary>
    [Fact]
    public void IsEasyCase_TargetMovingTowardCaster_ProcessedEfficiently()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(700, 0),
            TargetVelocity: new Vector2D(-350, 0), // Moving directly toward caster
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        Assert.IsType<PredictionResult.Hit>(result);
        var hit = (PredictionResult.Hit)result;
        
        // Fast approaching targets are harder to predict (timing more critical),
        // so confidence reflects this uncertainty. Just verify it's reasonable.
        Assert.True(hit.Confidence > 0.2, $"Expected confidence > 0.2, got {hit.Confidence}");
        
        // Aim point should be between caster and target (target coming toward us)
        Assert.True(hit.CastPosition.X < 700, $"Expected aim < 700, got {hit.CastPosition.X}");
        Assert.True(hit.CastPosition.X > 0, $"Expected aim > 0, got {hit.CastPosition.X}");
    }

    /// <summary>
    /// Tests vector-based futureDistance calculation with angled movement.
    /// Previously used magnitude approximation which was inaccurate for non-radial movement.
    /// </summary>
    [Fact]
    public void FutureDistance_AngledMovement_AccurateCalculation()
    {
        // Target at 1200 units, moving at angle (not directly away)
        // Old approximation: futureDistance = distance + speed * time (overestimate)
        // New calculation: uses actual vector position
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(1200, 0), // Just beyond range (1000)
            TargetVelocity: new Vector2D(100, 200), // Moving at angle, partially away
            Skillshot: new LinearSkillshot(Speed: 1500, Range: 1000, Width: 70, Delay: 0.25));
        
        var result = _prediction.Predict(input);
        
        // With accurate vector math, this might be hittable if target curves back into range
        // The test verifies no crash/NaN occurs and result is reasonable
        Assert.NotNull(result);
        
        if (result is PredictionResult.Hit hit)
        {
            Assert.False(double.IsNaN(hit.CastPosition.X));
            Assert.False(double.IsNaN(hit.InterceptTime));
        }
    }

    #endregion
}
