using MathNet.Spatial.Euclidean;

namespace Predicto.Models;

/// <summary>
/// Result of a prediction calculation.
/// Uses a discriminated union pattern for different outcomes.
/// </summary>
public abstract record PredictionResult
{
    private PredictionResult() { }

    /// <summary>
    /// A successful prediction where the skillshot can intercept the target.
    /// </summary>
    /// <param name="CastPosition">Where to aim the skillshot</param>
    /// <param name="PredictedTargetPosition">Where the target will be at interception</param>
    /// <param name="InterceptTime">Time in seconds until interception</param>
    /// <param name="Confidence">Confidence score (0-1) based on prediction reliability</param>
    public sealed record Hit(
        Point2D CastPosition,
        Point2D PredictedTargetPosition,
        double InterceptTime,
        double Confidence) : PredictionResult;

    /// <summary>
    /// The target is beyond the skillshot's maximum range.
    /// </summary>
    /// <param name="Distance">Current distance to target</param>
    /// <param name="MaxRange">Skillshot's maximum range</param>
    public sealed record OutOfRange(
        double Distance,
        double MaxRange) : PredictionResult;

    /// <summary>
    /// No valid interception solution exists (target moving too fast, etc.)
    /// </summary>
    /// <param name="Reason">Human-readable explanation of why interception failed</param>
    public sealed record Unreachable(
        string Reason) : PredictionResult;
}
