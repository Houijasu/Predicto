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
public readonly record struct PredictionInput(
    Point2D CasterPosition,
    Point2D TargetPosition,
    Vector2D TargetVelocity,
    LinearSkillshot Skillshot,
    double TargetHitboxRadius = 65.0);
