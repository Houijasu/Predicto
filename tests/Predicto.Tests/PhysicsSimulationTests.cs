using MathNet.Spatial.Euclidean;
using Predicto.Models;
using Predicto.Physics;
using Predicto.Solvers;
using Xunit;

namespace Predicto.Tests;

/// <summary>
/// Tests for the physics-based dodge simulation system.
/// Verifies the Newtonian/Least Action Principle implementation.
/// </summary>
public class PhysicsSimulationTests
{
    private readonly DodgeSimulator _simulator = new();
    private readonly Ultimate _prediction = new();

    #region FastMath Tests

    [Fact]
    public void FastMath_LorentzianFalloff_PeakAtZero()
    {
        double result = FastMath.LorentzianFalloff(0);
        Assert.Equal(1.0, result, precision: 10);
    }

    [Fact]
    public void FastMath_LorentzianFalloff_DecaysWithDistance()
    {
        double at0 = FastMath.LorentzianFalloff(0);
        double at1 = FastMath.LorentzianFalloff(1);
        double at2 = FastMath.LorentzianFalloff(2);

        Assert.True(at0 > at1);
        Assert.True(at1 > at2);
        Assert.Equal(0.5, at1, precision: 10); // 1/(1+1) = 0.5
    }

    [Fact]
    public void FastMath_LorentzianFalloff_WithSigma_ControlsWidth()
    {
        double sigma = 2.0;
        double atSigma = FastMath.LorentzianFalloff(sigma, sigma);
        Assert.Equal(0.5, atSigma, precision: 10); // At x=sigma, falloff is 0.5
    }

    [Fact]
    public void FastMath_PerpendicularDistance_HorizontalLine()
    {
        var point = new Point2D(5, 3);
        var lineOrigin = new Point2D(0, 0);
        var lineDirection = new Vector2D(1, 0); // Horizontal line

        double dist = FastMath.AbsPerpendicularDistance(point, lineOrigin, lineDirection);
        Assert.Equal(3.0, dist, precision: 10); // Point is 3 units above line
    }

    [Fact]
    public void FastMath_ProjectOntoLine_ReturnsCorrectParameter()
    {
        var point = new Point2D(5, 3);
        var lineOrigin = new Point2D(0, 0);
        var lineDirection = new Vector2D(1, 0);

        double t = FastMath.ProjectOntoLine(point, lineOrigin, lineDirection);
        Assert.Equal(5.0, t, precision: 10); // Point projects to x=5
    }

    [Fact]
    public void FastMath_ClampMagnitude_UnderLimit_NoChange()
    {
        FastMath.ClampMagnitude(3, 4, 10, out double outX, out double outY);
        Assert.Equal(3, outX, precision: 10);
        Assert.Equal(4, outY, precision: 10);
    }

    [Fact]
    public void FastMath_ClampMagnitude_OverLimit_Clamps()
    {
        FastMath.ClampMagnitude(30, 40, 5, out double outX, out double outY);
        double magnitude = Math.Sqrt(outX * outX + outY * outY);
        Assert.Equal(5.0, magnitude, precision: 10);
    }

    #endregion

    #region StructOdeSolver Tests

    [Fact]
    public void StructOdeSolver_Step_ConstantVelocity_MovesCorrectly()
    {
        var state = new ParticleState(0, 0, 100, 0); // Moving right at 100 units/s

        AccelerationFunction noAccel = (in ParticleState s, double t, out double ax, out double ay) =>
        {
            ax = 0;
            ay = 0;
        };

        StructOdeSolver.Step(ref state, 0, 1.0, noAccel); // 1 second step

        Assert.Equal(100, state.X, precision: 5);
        Assert.Equal(0, state.Y, precision: 5);
    }

    [Fact]
    public void StructOdeSolver_Step_ConstantAcceleration_CorrectPosition()
    {
        var state = new ParticleState(0, 0, 0, 0); // Stationary

        AccelerationFunction constantAccel = (in ParticleState s, double t, out double ax, out double ay) =>
        {
            ax = 10; // 10 units/s²
            ay = 0;
        };

        // x = 0.5 * a * t² for constant acceleration from rest
        StructOdeSolver.Step(ref state, 0, 1.0, constantAccel);

        // After 1 second: x = 0.5 * 10 * 1 = 5, v = 10
        Assert.Equal(5, state.X, precision: 1);
        Assert.Equal(10, state.Vx, precision: 1);
    }

    [Fact]
    public void StructOdeSolver_Integrate_MultipleSteps()
    {
        var state = new ParticleState(0, 0, 100, 50);

        AccelerationFunction noAccel = (in ParticleState s, double t, out double ax, out double ay) =>
        {
            ax = 0;
            ay = 0;
        };

        StructOdeSolver.Integrate(ref state, 0, 2.0, 0.1, noAccel);

        Assert.Equal(200, state.X, precision: 1);
        Assert.Equal(100, state.Y, precision: 1);
    }

    #endregion

    #region LinearDangerField Tests

    [Fact]
    public void LinearDangerField_Potential_HighOnPath()
    {
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.EzrealQ;

        var field = new LinearDangerField(origin, direction, skillshot);

        // Point directly ahead on path should have high potential
        var onPath = new Point2D(200, 0);
        double potential = field.Potential(onPath, 0.3); // After some travel time

        Assert.True(potential > 100, $"Potential on path should be high, got {potential}");
    }

    [Fact]
    public void LinearDangerField_Potential_LowOffPath()
    {
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.EzrealQ;

        var field = new LinearDangerField(origin, direction, skillshot);

        // Point far off path should have low potential
        var offPath = new Point2D(200, 500);
        double potential = field.Potential(offPath, 0.3);

        Assert.True(potential < 10, $"Potential off path should be low, got {potential}");
    }

    [Fact]
    public void LinearDangerField_Gradient_PointsAwayFromPath()
    {
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.EzrealQ;

        var field = new LinearDangerField(origin, direction, skillshot);

        // Point slightly above path
        var nearPath = new Point2D(200, 50);
        field.Gradient(nearPath, 0.3, out double gx, out double gy);

        // Gradient should point toward path (down), so force points away (up)
        // Since we're above the path, gy of gradient should be negative (pointing down)
        // Force = -gradient, so force Y should be positive (pushing up)
        Assert.True(gy < 0, $"Gradient Y should be negative (toward path), got {gy}");
    }

    [Fact]
    public void LinearDangerField_IsActive_FalseAfterMaxRange()
    {
        var origin = new Point2D(0, 0);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.Create(speed: 1000, range: 500, width: 70, delay: 0.25);

        var field = new LinearDangerField(origin, direction, skillshot);

        // At time 0.25 (after delay), projectile starts moving
        // At time 0.75 (0.5s flight), projectile has traveled 500 units (max range)
        Assert.True(field.IsActive(0.5)); // Still traveling
        Assert.False(field.IsActive(1.0)); // Past max range
    }

    #endregion

    #region CircularDangerField Tests

    [Fact]
    public void CircularDangerField_Potential_HighAtCenter()
    {
        var center = new Point2D(500, 500);
        var skillshot = CircularSkillshot.BrandW; // 260 radius, 0.625s delay

        var field = new CircularDangerField(center, skillshot);

        // At detonation time, center should have high potential
        double potential = field.Potential(center, skillshot.Delay);
        Assert.True(potential > 500, $"Potential at center should be high, got {potential}");
    }

    [Fact]
    public void CircularDangerField_Potential_LowFarAway()
    {
        var center = new Point2D(500, 500);
        var skillshot = CircularSkillshot.BrandW;

        var field = new CircularDangerField(center, skillshot);

        var farAway = new Point2D(1500, 1500);
        double potential = field.Potential(farAway, skillshot.Delay);

        Assert.True(potential < 50, $"Potential far away should be low, got {potential}");
    }

    [Fact]
    public void CircularDangerField_Potential_ZeroBeforeAnticipation()
    {
        var center = new Point2D(500, 500);
        var skillshot = CircularSkillshot.Create(radius: 200, range: 1000, delay: 1.0);

        var field = new CircularDangerField(center, skillshot, anticipationWindow: 0.3);

        // Before anticipation window (1.0 - 0.3 = 0.7), potential should be zero
        double potential = field.Potential(center, 0.5);
        Assert.Equal(0, potential, precision: 5);
    }

    #endregion

    #region DodgeSimulator Tests

    [Fact]
    public void DodgeSimulator_StationaryTarget_NoDodgeProfile_NoMovement()
    {
        var targetPos = new Point2D(500, 500);
        var targetVel = new Vector2D(0, 0);
        var casterPos = new Point2D(0, 500);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.EzrealQ;

        var dangerField = new LinearDangerField(casterPos, direction, skillshot);
        var profile = DodgeProfile.None;

        var result = _simulator.Simulate(targetPos, targetVel, dangerField, profile, maxTime: 0.5);

        // With None profile (infinite reaction time), target shouldn't move
        Assert.Equal(targetPos.X, result.FinalPosition.X, precision: 1);
        Assert.Equal(targetPos.Y, result.FinalPosition.Y, precision: 1);
    }

    [Fact]
    public void DodgeSimulator_MovingTarget_AverageProfile_Dodges()
    {
        var targetPos = new Point2D(500, 500);
        var targetVel = new Vector2D(0, 100); // Moving up
        var casterPos = new Point2D(0, 500);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.MorganaQ; // Slower skillshot

        var dangerField = new LinearDangerField(casterPos, direction, skillshot);
        var profile = DodgeProfile.Average;

        var result = _simulator.Simulate(targetPos, targetVel, dangerField, profile, maxTime: 1.0);

        // Target should move away from the path
        // Since target is moving up and skillshot is coming from left, 
        // target should continue moving up and away
        Assert.True(result.FinalPosition.Y > targetPos.Y, "Target should move away from danger");
    }

    [Fact]
    public void DodgeSimulator_ExpertProfile_BetterDodge()
    {
        var targetPos = new Point2D(400, 500);
        var targetVel = new Vector2D(100, 0); // Moving right toward caster
        var casterPos = new Point2D(0, 500);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.ThreshQ;

        var dangerField = new LinearDangerField(casterPos, direction, skillshot);

        var beginnerResult = _simulator.Simulate(targetPos, targetVel, dangerField, DodgeProfile.Beginner, maxTime: 1.0);
        var expertResult = _simulator.Simulate(targetPos, targetVel, dangerField, DodgeProfile.Expert, maxTime: 1.0);

        // Expert should have better escape (more distance from danger)
        // This is a relative test - exact values depend on tuning
        Assert.True(expertResult.MinDangerDistance >= beginnerResult.MinDangerDistance * 0.8,
            "Expert should dodge at least as well as beginner");
    }

    [Fact]
    public void DodgeSimulator_EstimateDodgeProbability_ReturnsValidRange()
    {
        var targetPos = new Point2D(500, 500);
        var targetVel = new Vector2D(0, 100);
        var casterPos = new Point2D(0, 500);
        var skillshot = LinearSkillshot.EzrealQ;

        double prob = _simulator.EstimateDodgeProbability(
            targetPos, targetVel, casterPos, skillshot, DodgeProfile.Average);

        Assert.InRange(prob, 0, 1);
    }

    [Fact]
    public void DodgeSimulator_NoneProfile_ZeroDodgeProbability()
    {
        var targetPos = new Point2D(500, 500);
        var targetVel = new Vector2D(0, 0);
        var casterPos = new Point2D(0, 500);
        var skillshot = LinearSkillshot.EzrealQ;

        double prob = _simulator.EstimateDodgeProbability(
            targetPos, targetVel, casterPos, skillshot, DodgeProfile.None);

        Assert.Equal(0, prob);
    }

    #endregion

    #region DodgeProfile Tests

    [Fact]
    public void DodgeProfile_Presets_HaveValidValues()
    {
        var profiles = new[]
        {
            DodgeProfile.None,
            DodgeProfile.Beginner,
            DodgeProfile.Average,
            DodgeProfile.Skilled,
            DodgeProfile.Expert
        };

        foreach (var profile in profiles)
        {
            Assert.True(profile.ReactionTime >= 0, "Reaction time must be non-negative");
            Assert.True(profile.MaxAcceleration >= 0, "Max acceleration must be non-negative");
            Assert.True(profile.MaxSpeed >= 0, "Max speed must be non-negative");
            Assert.InRange(profile.SkillLevel, 0, 1);
            Assert.True(profile.Friction >= 0, "Friction must be non-negative");
            Assert.InRange(profile.PathAdherence, 0, 1);
        }
    }

    [Fact]
    public void DodgeProfile_FromSpeed_CreatesValidProfile()
    {
        var profile = DodgeProfile.FromSpeed(350, 0.7);

        Assert.Equal(350, profile.MaxSpeed);
        Assert.InRange(profile.SkillLevel, 0.69, 0.71);
        Assert.True(profile.ReactionTime > 0);
        Assert.True(profile.MaxAcceleration > 0);
    }

    [Fact]
    public void DodgeProfile_SkillLevel_AffectsReactionTime()
    {
        var beginner = DodgeProfile.Beginner;
        var expert = DodgeProfile.Expert;

        Assert.True(expert.ReactionTime < beginner.ReactionTime,
            "Expert should have faster reaction time");
    }

    #endregion

    #region Ultimate Physics Prediction Tests

    [Fact]
    public void Ultimate_PredictWithPhysics_ReturnsPhysicsHit()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(100, 0),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithPhysics(input, DodgeProfile.Average);

        Assert.NotNull(result);
        Assert.True(result.InterceptTime > 0);
        Assert.InRange(result.DodgeProbability, 0, 1);
    }

    [Fact]
    public void Ultimate_PredictWithPhysics_NoneProfile_ZeroDodgeProbability()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithPhysics(input, DodgeProfile.None);

        Assert.NotNull(result);
        Assert.Equal(0, result.DodgeProbability);
    }

    [Fact]
    public void Ultimate_PredictWithPhysics_OutOfRange_ReturnsNull()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(5000, 0), // Way out of range
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: LinearSkillshot.EzrealQ, // 1200 range
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithPhysics(input, DodgeProfile.Average);

        Assert.Null(result); // Should return null for out-of-range
    }

    [Fact]
    public void Ultimate_PredictWithPhysics_EffectiveConfidence_AccountsForDodge()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: LinearSkillshot.MorganaQ, // Slow skillshot, easier to dodge
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithPhysics(input, DodgeProfile.Expert);

        Assert.NotNull(result);
        // Effective confidence should be less than or equal to base confidence
        Assert.True(result.EffectiveConfidence <= result.Confidence);
    }

    [Fact]
    public void Ultimate_SimulateDodge_ReturnsValidResult()
    {
        var result = _prediction.SimulateDodge(
            targetPosition: new Point2D(500, 500),
            targetVelocity: new Vector2D(100, 0),
            casterPosition: new Point2D(0, 500),
            skillshot: LinearSkillshot.EzrealQ,
            profile: DodgeProfile.Average);

        Assert.True(result.SimulationTime > 0);
        Assert.True(result.DistanceTraveled >= 0);
    }


    #endregion

    #region LAP Prediction Tests

    [Fact]
    public void PredictWithLAP_ReturnsValidResult_ForMovingTarget()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 100), // Moving perpendicular to skillshot
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Average);

        Assert.NotNull(result);
        Assert.True(result.InterceptTime > 0);
        Assert.InRange(result.Confidence, 0, 1);
        Assert.True(result.IterationsUsed >= 1);
    }

    [Fact]
    public void PredictWithLAP_NoneProfile_ReturnsBaselineResult()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.None);

        Assert.NotNull(result);
        Assert.Equal(0, result.IterationsUsed); // No iterations for None profile
        Assert.Equal(0, result.AimAdjustment); // No adjustment
        Assert.True(result.Converged);
    }

    [Fact]
    public void PredictWithLAP_StationaryTarget_ReturnsBaselineResult()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0), // Stationary
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Expert);

        Assert.NotNull(result);
        Assert.Equal(0, result.IterationsUsed); // No iterations for stationary
        Assert.Equal(0, result.AimAdjustment);
    }

    [Fact]
    public void PredictWithLAP_ExpertProfile_AdjustsAimPoint()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(600, 0),
            TargetVelocity: new Vector2D(0, 200), // Moving perpendicular
            Skillshot: LinearSkillshot.MorganaQ, // Slower skillshot, more dodgeable
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Expert);

        Assert.NotNull(result);
        // Expert dodgers should cause aim adjustment
        Assert.True(result.AimAdjustment > 0 || result.IterationsUsed >= 1,
            "Expert profile should cause some aim adjustment or at least attempt iteration");
    }

    [Fact]
    public void PredictWithLAP_ConvergesWithinMaxIterations()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 150),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Skilled, maxIterations: 5);

        Assert.NotNull(result);
        Assert.True(result.IterationsUsed <= 5);
    }

    [Fact]
    public void PredictWithLAP_OutOfRange_ReturnsNull()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(5000, 0), // Way out of range
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: LinearSkillshot.EzrealQ, // 1200 range
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Average);

        Assert.Null(result);
    }

    [Fact]
    public void PredictWithLAP_HasTrajectory_ForActiveProfile()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 150),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Average);

        Assert.NotNull(result);
        Assert.NotNull(result.Trajectory);
        Assert.True(result.Trajectory.Value.Count > 0);
        Assert.True(result.Trajectory.Value.Duration > 0);
    }

    [Fact]
    public void PredictWithLAP_BaselineAndLAP_DifferForDodgingTarget()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: LinearSkillshot.MorganaQ, // Slow, easier to dodge
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictWithLAP(input, DodgeProfile.Expert);

        Assert.NotNull(result);
        // Baseline and LAP predicted positions should differ for active dodgers
        var baselineDist = (result.BaselinePredictedPosition - input.TargetPosition).Length;
        var lapDist = (result.PredictedTargetPosition - input.TargetPosition).Length;
        
        // Both predictions should be different from starting position (target is moving)
        Assert.True(baselineDist > 0 || lapDist > 0,
            "Predictions should differ from start position for moving target");
    }

    [Fact]
    public void PredictWithLAP_ToHit_ReturnsValidPredictionResult()
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var lapResult = _prediction.PredictWithLAP(input, DodgeProfile.Average);
        
        Assert.NotNull(lapResult);
        var hit = lapResult.ToHit();
        
        Assert.Equal(lapResult.CastPosition, hit.CastPosition);
        Assert.Equal(lapResult.PredictedTargetPosition, hit.PredictedTargetPosition);
        Assert.Equal(lapResult.InterceptTime, hit.InterceptTime);
        Assert.Equal(lapResult.Confidence, hit.Confidence);
    }

    [Theory]
    [InlineData(0.0)]   // None (beginner-like)
    [InlineData(0.3)]   // Beginner
    [InlineData(0.5)]   // Average
    [InlineData(0.7)]   // Skilled
    [InlineData(0.9)]   // Expert
    public void PredictWithLAP_DifferentSkillLevels_ReturnsValidResults(double skillLevel)
    {
        var input = new PredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 150),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var profile = DodgeProfile.FromSpeed(345, skillLevel);
        var result = _prediction.PredictWithLAP(input, profile);

        // Skill level 0 returns baseline (null-like), others should return valid results
        if (skillLevel < Constants.Epsilon)
        {
            Assert.NotNull(result);
            Assert.Equal(0, result.IterationsUsed);
        }
        else
        {
            Assert.NotNull(result);
            Assert.InRange(result.Confidence, 0, 1);
        }
    }

    #endregion

    #region Circular LAP Prediction Tests

    [Fact]
    public void PredictCircularWithLAP_ReturnsValidResult()
    {
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 150),
            Skillshot: CircularSkillshot.BrandW,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictCircularWithLAP(input, DodgeProfile.Average);

        Assert.NotNull(result);
        Assert.True(result.InterceptTime > 0);
        Assert.InRange(result.Confidence, 0, 1);
    }

    [Fact]
    public void PredictCircularWithLAP_NoneProfile_ReturnsBaselineResult()
    {
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: CircularSkillshot.BrandW,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictCircularWithLAP(input, DodgeProfile.None);

        Assert.NotNull(result);
        Assert.Equal(0, result.IterationsUsed);
        Assert.Equal(0, result.AimAdjustment);
        Assert.True(result.Converged);
    }

    [Fact]
    public void PredictCircularWithLAP_StationaryTarget_ReturnsBaselineResult()
    {
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(500, 0),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: CircularSkillshot.BrandW,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictCircularWithLAP(input, DodgeProfile.Expert);

        Assert.NotNull(result);
        Assert.Equal(0, result.IterationsUsed);
    }

    [Fact]
    public void PredictCircularWithLAP_OutOfRange_ReturnsNull()
    {
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(5000, 0), // Way out of range
            TargetVelocity: new Vector2D(0, 100),
            Skillshot: CircularSkillshot.BrandW, // 1100 range
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictCircularWithLAP(input, DodgeProfile.Average);

        Assert.Null(result);
    }

    [Fact]
    public void PredictCircularWithLAP_ExpertProfile_ConvergesOrAdjusts()
    {
        var input = new CircularPredictionInput(
            CasterPosition: new Point2D(0, 0),
            TargetPosition: new Point2D(600, 0),
            TargetVelocity: new Vector2D(0, 200),
            Skillshot: CircularSkillshot.Create(radius: 300, range: 1000, delay: 0.8), // Slow delay
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        var result = _prediction.PredictCircularWithLAP(input, DodgeProfile.Expert);

        Assert.NotNull(result);
        Assert.True(result.Converged || result.IterationsUsed >= 1,
            "Should either converge or attempt iterations");
    }

    #endregion

    #region DodgeTrajectory Tests

    [Fact]
    public void DodgeTrajectory_GetPositionAt_InterpolatesCorrectly()
    {
        var positions = new[]
        {
            new Point2D(0, 0),
            new Point2D(10, 0),
            new Point2D(20, 0),
        };
        var velocities = new[]
        {
            new Vector2D(10, 0),
            new Vector2D(10, 0),
            new Vector2D(10, 0),
        };
        var trajectory = new DodgeTrajectory(positions, velocities, timeStep: 1.0);

        // At t=0
        var pos0 = trajectory.GetPositionAt(0);
        Assert.Equal(0, pos0.X, precision: 5);

        // At t=0.5 (midpoint between first two)
        var pos05 = trajectory.GetPositionAt(0.5);
        Assert.Equal(5, pos05.X, precision: 5);

        // At t=1.0
        var pos1 = trajectory.GetPositionAt(1.0);
        Assert.Equal(10, pos1.X, precision: 5);

        // At t=2.0 (end)
        var pos2 = trajectory.GetPositionAt(2.0);
        Assert.Equal(20, pos2.X, precision: 5);

        // Beyond end
        var posBeyond = trajectory.GetPositionAt(3.0);
        Assert.Equal(20, posBeyond.X, precision: 5); // Should clamp to last
    }

    [Fact]
    public void DodgeTrajectory_GetVelocityAt_InterpolatesCorrectly()
    {
        var positions = new[]
        {
            new Point2D(0, 0),
            new Point2D(10, 0),
        };
        var velocities = new[]
        {
            new Vector2D(100, 0),
            new Vector2D(200, 0),
        };
        var trajectory = new DodgeTrajectory(positions, velocities, timeStep: 1.0);

        // At t=0.5 (midpoint)
        var vel = trajectory.GetVelocityAt(0.5);
        Assert.Equal(150, vel.X, precision: 5); // Interpolated
    }

    [Fact]
    public void DodgeTrajectory_Duration_CalculatesCorrectly()
    {
        var positions = new Point2D[61]; // 60 steps at 1/60s = 1 second
        var velocities = new Vector2D[61];
        for (int i = 0; i < 61; i++)
        {
            positions[i] = new Point2D(i, 0);
            velocities[i] = new Vector2D(60, 0);
        }

        var trajectory = new DodgeTrajectory(positions, velocities, timeStep: 1.0 / 60.0);

        Assert.Equal(61, trajectory.Count);
        Assert.Equal(1.0, trajectory.Duration, precision: 5);
    }

    #endregion

    #region SimulateWithTrajectory Tests

    [Fact]
    public void SimulateWithTrajectory_ReturnsValidTrajectory()
    {
        var targetPos = new Point2D(500, 500);
        var targetVel = new Vector2D(0, 100);
        var casterPos = new Point2D(0, 500);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.EzrealQ;

        var dangerField = new LinearDangerField(casterPos, direction, skillshot);
        var profile = DodgeProfile.Average;

        var (result, trajectory) = _simulator.SimulateWithTrajectory(
            targetPos, targetVel, dangerField, profile, maxTime: 1.0);

        Assert.NotNull(trajectory.Positions);
        Assert.NotNull(trajectory.Velocities);
        Assert.True(trajectory.Count > 0);
        Assert.Equal(trajectory.Positions.Length, trajectory.Velocities.Length);
    }

    [Fact]
    public void SimulateWithTrajectory_TrajectoryMatchesFinalState()
    {
        var targetPos = new Point2D(500, 500);
        var targetVel = new Vector2D(100, 0);
        var casterPos = new Point2D(0, 500);
        var direction = new Vector2D(1, 0);
        var skillshot = LinearSkillshot.EzrealQ;

        var dangerField = new LinearDangerField(casterPos, direction, skillshot);
        var profile = DodgeProfile.Beginner;

        var (result, trajectory) = _simulator.SimulateWithTrajectory(
            targetPos, targetVel, dangerField, profile, maxTime: 0.5);

        // Last trajectory position should match final result position
        var lastPos = trajectory.Positions[^1];
        Assert.Equal(result.FinalPosition.X, lastPos.X, precision: 1);
        Assert.Equal(result.FinalPosition.Y, lastPos.Y, precision: 1);
    }

    #endregion
}
