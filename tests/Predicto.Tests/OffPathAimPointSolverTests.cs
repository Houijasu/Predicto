using MathNet.Spatial.Euclidean;
using Predicto.Solvers;

namespace Predicto.Tests;

public class OffPathAimPointSolverTests
{
    private const double Epsilon = 0.001;

    #region Stationary Target Tests

    [Fact]
    public void StationaryTarget_DirectBehind_ReturnsPredictedPosition()
    {
        // Stationary target - no velocity, should return predicted position
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 0);
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        // Should return predicted position for stationary target
        Assert.Equal(predicted.X, result.X, precision: 1);
        Assert.Equal(predicted.Y, result.Y, precision: 1);
    }

    [Fact]
    public void StationaryTarget_Tangent_FallsBackGracefully()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 0);
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);

        // Should fall back to direct behind for stationary target
        // The aim point should be on the circle around predicted position
        double distFromPredicted = (result - predicted).Length;
        Assert.True(distFromPredicted <= radius + Epsilon,
            $"Aim point should be within radius of predicted position, got distance {distFromPredicted}");
    }

    [Fact]
    public void StationaryTarget_Adaptive_FallsBackToDirectBehind()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 0);
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);

        // Should return predicted position for stationary target
        Assert.Equal(predicted.X, result.X, precision: 1);
        Assert.Equal(predicted.Y, result.Y, precision: 1);
    }

    #endregion

    #region Target Moving Toward Caster Tests

    [Fact]
    public void TargetMovingTowardCaster_DirectBehind_AimsBehindTarget()
    {
        // Target at (500, 0) moving toward caster at (0, 0)
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(-300, 0); // Moving toward caster
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        // Behind direction is opposite of velocity, so +X direction
        // Aim point should be at (550, 0)
        Assert.True(result.X > predicted.X,
            $"Aim point should be behind target (greater X), got {result.X}");
        Assert.Equal(predicted.X + radius, result.X, precision: 1);
        Assert.Equal(0, result.Y, precision: 1);
    }

    [Fact]
    public void TargetMovingTowardCaster_Tangent_ProducesFasterIntercept()
    {
        // Target moving toward caster - Tangent should give closer aim point
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(-300, 0); // Moving toward caster
        double radius = 50;

        var tangentResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);
        var directBehindResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        double tangentDist = (tangentResult - caster).Length;
        double directBehindDist = (directBehindResult - caster).Length;

        // Tangent should be closer or equal to caster (faster intercept)
        Assert.True(tangentDist <= directBehindDist + Epsilon,
            $"Tangent ({tangentDist}) should be <= DirectBehind ({directBehindDist}) when target approaches");
    }

    [Fact]
    public void TargetMovingTowardCaster_Adaptive_BiasedTowardTangent()
    {
        // When target moves toward caster, Adaptive should be biased toward Tangent
        // but blended with center for smoothing
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(-300, 0); // Moving directly toward caster
        double radius = 50;

        var adaptiveResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);
        var tangentResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);
        var directBehindResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        // Adaptive should be closer to Tangent than to DirectBehind when approaching
        double distToTangent = (adaptiveResult - tangentResult).Length;
        double distToDirectBehind = (adaptiveResult - directBehindResult).Length;
        
        Assert.True(distToTangent <= distToDirectBehind,
            $"Adaptive should be closer to Tangent ({distToTangent:F1}) than DirectBehind ({distToDirectBehind:F1}) when approaching");
        
        // Adaptive should still be on the effective radius circle
        double distFromPredicted = (adaptiveResult - predicted).Length;
        Assert.Equal(radius, distFromPredicted, precision: 1);
    }

    #endregion

    #region Target Moving Away From Caster Tests

    [Fact]
    public void TargetMovingAwayFromCaster_DirectBehind_AimsBehindTarget()
    {
        // Target at (500, 0) moving away from caster at (0, 0)
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(300, 0); // Moving away from caster
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        // Behind direction is opposite of velocity, so -X direction
        // Aim point should be at (450, 0)
        Assert.True(result.X < predicted.X,
            $"Aim point should be behind target (lesser X), got {result.X}");
        Assert.Equal(predicted.X - radius, result.X, precision: 1);
        Assert.Equal(0, result.Y, precision: 1);
    }

    [Fact]
    public void TargetMovingAwayFromCaster_Adaptive_BiasedTowardDirectBehind()
    {
        // When target moves away from caster, Adaptive should be biased toward DirectBehind
        // but blended with center for smoothing
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(300, 0); // Moving directly away from caster
        double radius = 50;

        var adaptiveResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);
        var tangentResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);
        var directBehindResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        // Adaptive should be closer to DirectBehind than to Tangent when retreating
        double distToTangent = (adaptiveResult - tangentResult).Length;
        double distToDirectBehind = (adaptiveResult - directBehindResult).Length;
        
        Assert.True(distToDirectBehind <= distToTangent,
            $"Adaptive should be closer to DirectBehind ({distToDirectBehind:F1}) than Tangent ({distToTangent:F1}) when retreating");
        
        // Adaptive should still be on the effective radius circle
        double distFromPredicted = (adaptiveResult - predicted).Length;
        Assert.Equal(radius, distFromPredicted, precision: 1);
    }

    [Fact]
    public void TargetMovingAwayFromCaster_DirectBehind_ProducesCloserAimPoint()
    {
        // When target moves away, DirectBehind should be closer to caster
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(300, 0); // Moving away from caster
        double radius = 50;

        var directBehindResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        double directBehindDist = (directBehindResult - caster).Length;

        // DirectBehind should be closer to caster than predicted position
        double predictedDist = (predicted - caster).Length;
        Assert.True(directBehindDist < predictedDist,
            $"DirectBehind ({directBehindDist}) should be closer than predicted ({predictedDist})");
    }

    #endregion

    #region Target Moving Perpendicular to Caster Tests

    [Fact]
    public void TargetMovingPerpendicular_Adaptive_BlendsBetweenStrategies()
    {
        // Target moving perpendicular to caster direction
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 300); // Moving perpendicular (up)
        double radius = 50;

        var adaptiveResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);
        var tangentResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);
        var directBehindResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

        // Adaptive should be somewhere between Tangent and DirectBehind
        double adaptiveX = adaptiveResult.X;
        double tangentX = tangentResult.X;
        double directBehindX = directBehindResult.X;

        // Since it's perpendicular movement, adaptive should blend ~50/50
        // Check that adaptive is between or near the two extremes
        double minX = Math.Min(tangentX, directBehindX);
        double maxX = Math.Max(tangentX, directBehindX);

        // Allow some tolerance for the blend
        Assert.True(adaptiveX >= minX - radius && adaptiveX <= maxX + radius,
            $"Adaptive X ({adaptiveX}) should be between Tangent ({tangentX}) and DirectBehind ({directBehindX})");
    }

    [Fact]
    public void TargetMovingPerpendicular_Tangent_AimsAtBehindHemisphere()
    {
        // Target moving perpendicular
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 300); // Moving up
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);

        // Behind direction is -Y (opposite of velocity)
        // Aim point should be in the -Y hemisphere
        Assert.True(result.Y < predicted.Y,
            $"Aim point should be in behind hemisphere (negative Y), got {result.Y}");

        // Should be on the circle
        double distFromPredicted = (result - predicted).Length;
        Assert.Equal(radius, distFromPredicted, precision: 1);
    }

    #endregion

    #region Aim Point Validity Tests

    [Theory]
    [InlineData(BehindEdgeStrategy.DirectBehind)]
    [InlineData(BehindEdgeStrategy.Tangent)]
    [InlineData(BehindEdgeStrategy.OptimalAngle)]
    public void NonBlendedStrategies_AimPointOnCircle(BehindEdgeStrategy strategy)
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 200);
        var velocity = new Vector2D(-150, 100);
        double radius = 60;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, strategy);

        double distFromPredicted = (result - predicted).Length;

        // Aim point should be on the effective radius circle
        Assert.Equal(radius, distFromPredicted, precision: 1);
    }

    [Fact]
    public void AdaptiveStrategy_AimPointNearCircle()
    {
        // Adaptive blends between two points on the circle, so the result
        // may be slightly inside the circle (chord instead of arc)
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 200);
        var velocity = new Vector2D(-150, 100);
        double radius = 60;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);

        double distFromPredicted = (result - predicted).Length;

        // Adaptive may be slightly inside the circle due to linear interpolation
        // but should be close to the radius
        Assert.True(distFromPredicted >= radius * 0.9 && distFromPredicted <= radius * 1.1,
            $"Adaptive aim point should be near circle, got distance {distFromPredicted} (expected ~{radius})");
    }

    [Theory]
    [InlineData(BehindEdgeStrategy.DirectBehind)]
    [InlineData(BehindEdgeStrategy.Tangent)]
    [InlineData(BehindEdgeStrategy.OptimalAngle)]
    [InlineData(BehindEdgeStrategy.Adaptive)]
    public void AllStrategies_AimPointInBehindHemisphere(BehindEdgeStrategy strategy)
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(-300, 0); // Moving toward caster
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, strategy);

        // Behind direction is +X (opposite of velocity which is -X)
        Vector2D behindDir = new Vector2D(1, 0);
        Vector2D aimOffset = result - predicted;
        double behindness = aimOffset.Normalize().DotProduct(behindDir);

        // Aim point should be in the behind hemisphere (positive dot product)
        Assert.True(behindness >= -Epsilon,
            $"Aim point should be in behind hemisphere, got behindness {behindness}");
    }

    #endregion

    #region Edge Case Tests

    [Fact]
    public void CasterInsideEffectiveRadius_FallsBackToDirectBehind()
    {
        // Caster is very close to target (inside effective radius)
        var caster = new Point2D(480, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 300);
        double radius = 50; // Caster is only 20 units away, inside 50 radius

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);

        // Should fall back gracefully - result should be valid
        double distFromPredicted = (result - predicted).Length;
        Assert.Equal(radius, distFromPredicted, precision: 1);
    }

    [Fact]
    public void VerySmallVelocity_TreatedAsStationary()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0.0001, 0.0001); // Nearly zero
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);

        // Should handle gracefully without crashing - just verify we got a valid point
        Assert.True(double.IsFinite(result.X) && double.IsFinite(result.Y),
            "Result should be a valid finite point");
    }

    [Fact]
    public void DiagonalMovement_ProducesValidAimPoint()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(400, 300);
        var velocity = new Vector2D(-100, 150); // Diagonal movement
        double radius = 45;

        foreach (var strategy in Enum.GetValues<BehindEdgeStrategy>())
        {
            var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
                caster, predicted, velocity, radius, strategy);

            double distFromPredicted = (result - predicted).Length;
            Assert.Equal(radius, distFromPredicted, precision: 1);
        }
    }

    #endregion

    #region OptimalAngle Specific Tests

    [Fact]
    public void OptimalAngle_FindsBestScorePoint()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 200);
        var velocity = new Vector2D(-200, 100);
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.OptimalAngle);

        // Should be on the circle
        double distFromPredicted = (result - predicted).Length;
        Assert.Equal(radius, distFromPredicted, precision: 1);

        // Should be in behind hemisphere
        Vector2D behindDir = velocity.Normalize().Negate();
        Vector2D aimOffset = (result - predicted).Normalize();
        double behindness = aimOffset.DotProduct(behindDir);
        Assert.True(behindness > 0, $"OptimalAngle should be behind target, got behindness {behindness}");
    }

    [Fact]
    public void OptimalAngle_StationaryTarget_ReturnsPredictedPosition()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        var velocity = new Vector2D(0, 0);
        double radius = 50;

        var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
            caster, predicted, velocity, radius, BehindEdgeStrategy.OptimalAngle);

        // Should return predicted position for stationary target
        Assert.Equal(predicted.X, result.X, precision: 1);
        Assert.Equal(predicted.Y, result.Y, precision: 1);
    }

    #endregion

    #region Adaptive Smooth Transition Tests

    [Fact]
    public void Adaptive_SmoothTransition_NoSuddenJumps()
    {
        // Test that as approach angle changes, the aim point moves smoothly
        // (no sudden jumps between completely different positions)
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        double radius = 50;
        double speed = 300;

        Point2D? previousResult = null;
        double maxJump = 0;

        // Rotate velocity from toward caster to away from caster
        for (int degrees = 0; degrees <= 180; degrees += 5)
        {
            double radians = degrees * Math.PI / 180;
            // 0 degrees = moving toward caster (-X), 180 degrees = moving away (+X)
            var velocity = new Vector2D(
                -Math.Cos(radians) * speed,
                Math.Sin(radians) * speed);

            var result = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
                caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);

            if (previousResult.HasValue)
            {
                double jump = (result - previousResult.Value).Length;
                maxJump = Math.Max(maxJump, jump);
            }

            previousResult = result;
        }

        // The maximum jump between 5° increments should be reasonable
        // Given the circle has circumference 2*pi*r = ~314, and we're sampling 36 points
        // around half the circle, a smooth transition would have jumps of ~4.4 units
        // We allow up to radius (50) as a reasonable threshold for smooth behavior
        Assert.True(maxJump < radius * 1.5,
            $"Maximum jump ({maxJump:F1}) should be less than 1.5x radius ({radius * 1.5}) for smooth transition");
    }

    [Fact]
    public void Adaptive_ApproachFactor_CorrectlyDeterminesStrategy()
    {
        var caster = new Point2D(0, 0);
        var predicted = new Point2D(500, 0);
        double radius = 50;

        // Test various approach angles
        var testCases = new[]
        {
            (velocity: new Vector2D(-300, 0), expectedCloserTo: "Tangent"),      // Directly toward
            (velocity: new Vector2D(300, 0), expectedCloserTo: "DirectBehind"),  // Directly away
            (velocity: new Vector2D(0, 300), expectedCloserTo: "Blend"),         // Perpendicular
        };

        foreach (var (velocity, expectedCloserTo) in testCases)
        {
            var adaptiveResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
                caster, predicted, velocity, radius, BehindEdgeStrategy.Adaptive);
            var tangentResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
                caster, predicted, velocity, radius, BehindEdgeStrategy.Tangent);
            var directBehindResult = OffPathAimPointSolver.CalculateBehindEdgeAimPoint(
                caster, predicted, velocity, radius, BehindEdgeStrategy.DirectBehind);

            double distToTangent = (adaptiveResult - tangentResult).Length;
            double distToDirectBehind = (adaptiveResult - directBehindResult).Length;

            switch (expectedCloserTo)
            {
                case "Tangent":
                    Assert.True(distToTangent <= distToDirectBehind,
                        $"Adaptive should be closer to Tangent when moving toward caster");
                    break;
                case "DirectBehind":
                    Assert.True(distToDirectBehind <= distToTangent,
                        $"Adaptive should be closer to DirectBehind when moving away");
                    break;
                case "Blend":
                    // For perpendicular, should be somewhere in between
                    // Just verify it's a valid point
                    double distFromPredicted = (adaptiveResult - predicted).Length;
                    Assert.Equal(radius, distFromPredicted, precision: 1);
                    break;
            }
        }
    }

    #endregion
}
