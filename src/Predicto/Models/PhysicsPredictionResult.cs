using MathNet.Spatial.Euclidean;

namespace Predicto.Models;

/// <summary>
/// Extended prediction result that includes physics simulation data.
/// Contains the same information as PredictionResult.Hit plus dodge analysis.
/// </summary>
public sealed record PhysicsHit
{
    /// <summary>
    /// Where to aim the skillshot.
    /// </summary>
    public Point2D CastPosition { get; init; }

    /// <summary>
    /// Where the target will be at interception (accounting for dodge).
    /// </summary>
    public Point2D PredictedTargetPosition { get; init; }

    /// <summary>
    /// Time in seconds until interception.
    /// </summary>
    public double InterceptTime { get; init; }

    /// <summary>
    /// Base confidence score (0-1) from geometric prediction.
    /// </summary>
    public double Confidence { get; init; }

    /// <summary>
    /// Probability that the target will successfully dodge (0-1).
    /// Based on physics simulation of target's optimal escape path.
    /// </summary>
    public double DodgeProbability { get; init; }

    /// <summary>
    /// Adjusted confidence accounting for dodge probability.
    /// Effective = Confidence * (1 - DodgeProbability)
    /// </summary>
    public double EffectiveConfidence => Confidence * (1.0 - DodgeProbability);

    /// <summary>
    /// The predicted position if the target doesn't dodge.
    /// Useful for comparison with dodged position.
    /// </summary>
    public Point2D NoDodgePosition { get; init; }

    /// <summary>
    /// Distance the target is predicted to move while dodging.
    /// </summary>
    public double DodgeDistance { get; init; }

    /// <summary>
    /// Creates a PhysicsHit result.
    /// </summary>
    public PhysicsHit(
        Point2D castPosition,
        Point2D predictedTargetPosition,
        double interceptTime,
        double confidence,
        double dodgeProbability,
        Point2D noDodgePosition,
        double dodgeDistance)
    {
        CastPosition = castPosition;
        PredictedTargetPosition = predictedTargetPosition;
        InterceptTime = interceptTime;
        Confidence = confidence;
        DodgeProbability = dodgeProbability;
        NoDodgePosition = noDodgePosition;
        DodgeDistance = dodgeDistance;
    }

    /// <summary>
    /// Converts this to a standard Hit result (discarding physics data).
    /// </summary>
    public PredictionResult.Hit ToHit() => new(
        CastPosition,
        PredictedTargetPosition,
        InterceptTime,
        EffectiveConfidence);
}

/// <summary>
/// Result of a dodge trajectory simulation.
/// Contains the predicted path of the target when they attempt to dodge.
/// </summary>
public readonly struct DodgeSimulationResult
{
    /// <summary>
    /// Final position of the target after the simulation.
    /// </summary>
    public Point2D FinalPosition { get; init; }

    /// <summary>
    /// Final velocity of the target.
    /// </summary>
    public Vector2D FinalVelocity { get; init; }

    /// <summary>
    /// Total time simulated.
    /// </summary>
    public double SimulationTime { get; init; }

    /// <summary>
    /// Whether the target successfully escaped the danger zone.
    /// </summary>
    public bool EscapedDanger { get; init; }

    /// <summary>
    /// Distance traveled during the simulation.
    /// </summary>
    public double DistanceTraveled { get; init; }

    /// <summary>
    /// Minimum distance to the danger source achieved during simulation.
    /// Lower values indicate closer calls.
    /// </summary>
    public double MinDangerDistance { get; init; }

    /// <summary>
    /// Time when collision would occur (if any).
    /// NaN if no collision predicted.
    /// </summary>
    public double CollisionTime { get; init; }

    /// <summary>
    /// Whether a collision is predicted.
    /// </summary>
    public bool WillCollide => !double.IsNaN(CollisionTime);

    /// <summary>
    /// Creates a successful dodge result.
    /// </summary>
    public static DodgeSimulationResult Escaped(
        Point2D finalPosition,
        Vector2D finalVelocity,
        double simulationTime,
        double distanceTraveled,
        double minDangerDistance) => new()
    {
        FinalPosition = finalPosition,
        FinalVelocity = finalVelocity,
        SimulationTime = simulationTime,
        EscapedDanger = true,
        DistanceTraveled = distanceTraveled,
        MinDangerDistance = minDangerDistance,
        CollisionTime = double.NaN
    };

    /// <summary>
    /// Creates a collision result.
    /// </summary>
    public static DodgeSimulationResult Collided(
        Point2D finalPosition,
        Vector2D finalVelocity,
        double collisionTime,
        double distanceTraveled,
        double minDangerDistance) => new()
    {
        FinalPosition = finalPosition,
        FinalVelocity = finalVelocity,
        SimulationTime = collisionTime,
        EscapedDanger = false,
        DistanceTraveled = distanceTraveled,
        MinDangerDistance = minDangerDistance,
        CollisionTime = collisionTime
    };
}
