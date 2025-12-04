using MathNet.Spatial.Euclidean;

namespace Predicto.Models;

/// <summary>
/// Input parameters for a prediction request.
/// Uses readonly record struct for stack allocation and zero GC pressure in hot paths.
/// </summary>
/// <param name="CasterPosition">Position where the skillshot originates</param>
/// <param name="TargetPosition">Current position of the target</param>
/// <param name="TargetVelocity">Target's velocity vector (units per second)</param>
/// <param name="Skillshot">Skillshot parameters</param>
/// <param name="TargetHitboxRadius">Target's hitbox radius for collision detection</param>
/// <param name="TargetPath">Optional multi-waypoint path for the target (takes precedence over TargetVelocity)</param>
public readonly record struct PredictionInput(
    Point2D CasterPosition,
    Point2D TargetPosition,
    Vector2D TargetVelocity,
    LinearSkillshot Skillshot,
    double TargetHitboxRadius = 65.0,
    TargetPath? TargetPath = null)
{
    /// <summary>
    /// Returns true if this input uses a multi-waypoint path instead of simple velocity.
    /// </summary>
    public bool HasPath => TargetPath != null;

    /// <summary>
    /// Gets the effective velocity - either from TargetPath or TargetVelocity.
    /// </summary>
    public Vector2D EffectiveVelocity => TargetPath?.GetCurrentVelocity() ?? TargetVelocity;

    /// <summary>
    /// Gets the effective current position - either from TargetPath or TargetPosition.
    /// </summary>
    public Point2D EffectivePosition => TargetPath?.CurrentPosition ?? TargetPosition;

    /// <summary>
    /// Creates a prediction input with a multi-waypoint path.
    /// </summary>
    public static PredictionInput WithPath(
        Point2D casterPosition,
        TargetPath path,
        LinearSkillshot skillshot,
        double targetHitboxRadius = 65.0)
    {
        return new PredictionInput(
            casterPosition,
            path.CurrentPosition,
            path.GetCurrentVelocity(),
            skillshot,
            targetHitboxRadius,
            path);
    }
}

/// <summary>
/// Input parameters for circular (ground-targeted) skillshot prediction.
/// Uses readonly record struct for stack allocation and zero GC pressure.
/// </summary>
/// <param name="CasterPosition">Position where the skillshot originates (for range check)</param>
/// <param name="TargetPosition">Current position of the target</param>
/// <param name="TargetVelocity">Target's velocity vector (units per second)</param>
/// <param name="Skillshot">Circular skillshot parameters</param>
/// <param name="TargetHitboxRadius">Target's hitbox radius for collision detection</param>
/// <param name="TargetPath">Optional multi-waypoint path for the target</param>
public readonly record struct CircularPredictionInput(
    Point2D CasterPosition,
    Point2D TargetPosition,
    Vector2D TargetVelocity,
    CircularSkillshot Skillshot,
    double TargetHitboxRadius = 65.0,
    TargetPath? TargetPath = null)
{
    /// <summary>
    /// Returns true if this input uses a multi-waypoint path instead of simple velocity.
    /// </summary>
    public bool HasPath => TargetPath != null;

    /// <summary>
    /// Gets the effective velocity - either from TargetPath or TargetVelocity.
    /// </summary>
    public Vector2D EffectiveVelocity => TargetPath?.GetCurrentVelocity() ?? TargetVelocity;

    /// <summary>
    /// Gets the effective current position - either from TargetPath or TargetPosition.
    /// </summary>
    public Point2D EffectivePosition => TargetPath?.CurrentPosition ?? TargetPosition;

    /// <summary>
    /// Creates a circular prediction input with a multi-waypoint path.
    /// </summary>
    public static CircularPredictionInput WithPath(
        Point2D casterPosition,
        TargetPath path,
        CircularSkillshot skillshot,
        double targetHitboxRadius = 65.0)
    {
        return new CircularPredictionInput(
            casterPosition,
            path.CurrentPosition,
            path.GetCurrentVelocity(),
            skillshot,
            targetHitboxRadius,
            path);
    }
}

/// <summary>
/// Represents a potential target candidate for multi-target priority selection.
/// </summary>
/// <param name="Position">Current position of the target</param>
/// <param name="Velocity">Target's velocity vector (units per second)</param>
/// <param name="HitboxRadius">Target's hitbox radius for collision detection (default 65)</param>
/// <param name="PriorityWeight">User-defined target importance weight (default 1.0).
/// Higher values = more important target. Examples: ADC = 2.0, Tank = 0.5, Assassin = 1.5</param>
/// <param name="Path">Optional multi-waypoint path for the target</param>
/// <param name="Tag">Optional user data to identify the target (e.g., champion name, entity ID)</param>
public readonly record struct TargetCandidate(
    Point2D Position,
    Vector2D Velocity,
    double HitboxRadius = 65.0,
    double PriorityWeight = 1.0,
    TargetPath? Path = null,
    object? Tag = null)
{
    /// <summary>
    /// Creates a target candidate with a movement path.
    /// </summary>
    public static TargetCandidate WithPath(
        TargetPath path,
        double hitboxRadius = 65.0,
        double priorityWeight = 1.0,
        object? tag = null)
    {
        return new TargetCandidate(
            path.CurrentPosition,
            path.GetCurrentVelocity(),
            hitboxRadius,
            priorityWeight,
            path,
            tag);
    }

    /// <summary>
    /// Creates a stationary target candidate.
    /// </summary>
    public static TargetCandidate Stationary(
        Point2D position,
        double hitboxRadius = 65.0,
        double priorityWeight = 1.0,
        object? tag = null)
    {
        return new TargetCandidate(
            position,
            new Vector2D(0, 0),
            hitboxRadius,
            priorityWeight,
            null,
            tag);
    }
}

/// <summary>
/// Result of ranking a target candidate, including the prediction result and priority score.
/// </summary>
/// <param name="Candidate">The original target candidate</param>
/// <param name="Result">The prediction result for this target</param>
/// <param name="PriorityScore">Calculated priority score (higher = better target)</param>
public readonly record struct RankedTarget(
    TargetCandidate Candidate,
    PredictionResult Result,
    double PriorityScore)
{
    /// <summary>
    /// Returns true if this target can be hit.
    /// </summary>
    public bool IsHittable => Result is PredictionResult.Hit;

    /// <summary>
    /// Gets the hit result if this target is hittable, otherwise null.
    /// </summary>
    public PredictionResult.Hit? HitResult => Result as PredictionResult.Hit;

    /// <summary>
    /// Gets the tag from the candidate for easy identification.
    /// </summary>
    public object? Tag => Candidate.Tag;
}
