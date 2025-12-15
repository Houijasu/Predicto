using MathNet.Spatial.Euclidean;

namespace Predicto.Models;

/// <summary>
/// Result of a Least Action Principle (LAP) enhanced prediction.
/// LAP predicts where the target will dodge to and adjusts the aim point accordingly.
/// </summary>
public sealed record LAPPredictionResult
{
    /// <summary>
    /// LAP-adjusted aim point - where to actually fire the skillshot.
    /// This accounts for the target's predicted dodge trajectory.
    /// </summary>
    public Point2D CastPosition { get; init; }

    /// <summary>
    /// Where the target is predicted to be at interception (on their dodge path).
    /// </summary>
    public Point2D PredictedTargetPosition { get; init; }

    /// <summary>
    /// Time in seconds until interception.
    /// </summary>
    public double InterceptTime { get; init; }

    /// <summary>
    /// Confidence score (0-1) accounting for dodge uncertainty.
    /// </summary>
    public double Confidence { get; init; }

    /// <summary>
    /// Original baseline aim point (assuming no dodge).
    /// </summary>
    public Point2D BaselineCastPosition { get; init; }

    /// <summary>
    /// Where target would be without dodging (baseline prediction).
    /// </summary>
    public Point2D BaselinePredictedPosition { get; init; }

    /// <summary>
    /// Number of iterations used to converge on the aim point.
    /// </summary>
    public int IterationsUsed { get; init; }

    /// <summary>
    /// Distance between baseline and LAP-adjusted aim points.
    /// Larger values indicate more significant dodge prediction.
    /// </summary>
    public double AimAdjustment { get; init; }

    /// <summary>
    /// Whether the LAP prediction converged within tolerance.
    /// </summary>
    public bool Converged { get; init; }

    /// <summary>
    /// The sampled dodge trajectory (positions at each time step).
    /// Useful for visualization.
    /// </summary>
    public DodgeTrajectory? Trajectory { get; init; }

    /// <summary>
    /// Creates a new LAP prediction result.
    /// </summary>
    public LAPPredictionResult(
        Point2D castPosition,
        Point2D predictedTargetPosition,
        double interceptTime,
        double confidence,
        Point2D baselineCastPosition,
        Point2D baselinePredictedPosition,
        int iterationsUsed,
        double aimAdjustment,
        bool converged,
        DodgeTrajectory? trajectory = null)
    {
        CastPosition = castPosition;
        PredictedTargetPosition = predictedTargetPosition;
        InterceptTime = interceptTime;
        Confidence = confidence;
        BaselineCastPosition = baselineCastPosition;
        BaselinePredictedPosition = baselinePredictedPosition;
        IterationsUsed = iterationsUsed;
        AimAdjustment = aimAdjustment;
        Converged = converged;
        Trajectory = trajectory;
    }

    /// <summary>
    /// Creates a LAP result from baseline prediction (no dodge adjustment).
    /// Used when dodge profile is None or prediction fails.
    /// </summary>
    public static LAPPredictionResult FromBaseline(PredictionResult.Hit baseline)
    {
        return new LAPPredictionResult(
            castPosition: baseline.CastPosition,
            predictedTargetPosition: baseline.PredictedTargetPosition,
            interceptTime: baseline.InterceptTime,
            confidence: baseline.Confidence,
            baselineCastPosition: baseline.CastPosition,
            baselinePredictedPosition: baseline.PredictedTargetPosition,
            iterationsUsed: 0,
            aimAdjustment: 0,
            converged: true,
            trajectory: null);
    }

    /// <summary>
    /// Converts this to a standard Hit result.
    /// </summary>
    public PredictionResult.Hit ToHit() => new(
        CastPosition,
        PredictedTargetPosition,
        InterceptTime,
        Confidence);
}

/// <summary>
/// Represents a sampled dodge trajectory for visualization and analysis.
/// </summary>
public readonly struct DodgeTrajectory
{
    /// <summary>
    /// Sampled positions along the trajectory.
    /// </summary>
    public readonly Point2D[] Positions { get; }

    /// <summary>
    /// Sampled velocities along the trajectory.
    /// </summary>
    public readonly Vector2D[] Velocities { get; }

    /// <summary>
    /// Time step between samples.
    /// </summary>
    public readonly double TimeStep { get; }

    /// <summary>
    /// Total number of samples.
    /// </summary>
    public int Count => Positions.Length;

    /// <summary>
    /// Total duration of the trajectory.
    /// </summary>
    public double Duration => (Count - 1) * TimeStep;

    /// <summary>
    /// Creates a new dodge trajectory.
    /// </summary>
    public DodgeTrajectory(Point2D[] positions, Vector2D[] velocities, double timeStep)
    {
        Positions = positions;
        Velocities = velocities;
        TimeStep = timeStep;
    }

    /// <summary>
    /// Gets the interpolated position at a given time.
    /// </summary>
    public Point2D GetPositionAt(double time)
    {
        if (time <= 0 || Count == 0)
            return Positions[0];

        double index = time / TimeStep;
        int i0 = (int)index;

        if (i0 >= Count - 1)
            return Positions[Count - 1];

        double t = index - i0;
        var p0 = Positions[i0];
        var p1 = Positions[i0 + 1];

        return new Point2D(
            p0.X + (p1.X - p0.X) * t,
            p0.Y + (p1.Y - p0.Y) * t);
    }

    /// <summary>
    /// Gets the interpolated velocity at a given time.
    /// </summary>
    public Vector2D GetVelocityAt(double time)
    {
        if (time <= 0 || Count == 0)
            return Velocities[0];

        double index = time / TimeStep;
        int i0 = (int)index;

        if (i0 >= Count - 1)
            return Velocities[Count - 1];

        double t = index - i0;
        var v0 = Velocities[i0];
        var v1 = Velocities[i0 + 1];

        return new Vector2D(
            v0.X + (v1.X - v0.X) * t,
            v0.Y + (v1.Y - v0.Y) * t);
    }
}
