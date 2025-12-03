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
