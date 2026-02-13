using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

public sealed partial class InterceptSolver
{



    /// <summary>
    /// Result of a circular skillshot prediction.
    /// </summary>
    /// <param name="CastPosition">Where to cast the spell (center of the circle)</param>
    /// <param name="PredictedTargetPosition">Where the target will be when spell detonates</param>
    /// <param name="DetonationTime">Time until detonation (equals cast delay)</param>
    /// <param name="DistanceFromCenter">Distance from target center to spell center at detonation</param>
    public readonly record struct CircularInterceptResult(
        Point2D CastPosition,
        Point2D PredictedTargetPosition,
        double DetonationTime,
        double DistanceFromCenter);

    /// <summary>
    /// Solves for optimal cast position of a circular (instant) skillshot.
    ///
    /// Mathematical basis for circular spells:
    /// Unlike linear skillshots, circular spells have NO travel time.
    /// The spell detonates at a fixed position after a delay.
    ///
    /// Given:
    ///   P = target position, V = target velocity
    ///   d = cast delay (time until detonation)
    ///   r = spell radius + target hitbox radius (effective radius)
    ///
    /// The target position at detonation time:
    ///   P_final = P + V * d
    ///
    /// For a CENTER hit (most damage, often center has bonus effects):
    ///   Cast at P_final exactly
    ///
    /// For EDGE hit (behind-target strategy):
    ///   Cast at P_final + offset in direction opposite to V
    ///   This makes target walk INTO the spell
    /// </summary>
    /// <param name="casterPosition">Position of the caster (for range validation)</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="spellRadius">Radius of the circular spell effect</param>
    /// <param name="castDelay">Time from cast to detonation</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius</param>
    /// <param name="spellRange">Maximum cast range from caster</param>
    /// <returns>Circular intercept result, or null if out of range</returns>
    public static CircularInterceptResult? SolveCircularIntercept(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double spellRadius,
        double castDelay,
        double targetHitboxRadius,
        double spellRange)
    {
        // Input validation
        if (spellRadius < 0)
            throw new ArgumentException("Spell radius cannot be negative", nameof(spellRadius));
        if (castDelay < 0)
            throw new ArgumentException("Cast delay cannot be negative", nameof(castDelay));
        if (spellRange <= 0)
            throw new ArgumentException("Spell range must be positive", nameof(spellRange));
        if (targetHitboxRadius < 0)
            throw new ArgumentException("Target hitbox radius cannot be negative", nameof(targetHitboxRadius));

        // Effective collision radius
        double effectiveRadius = spellRadius + targetHitboxRadius;

        // Predict target position at detonation time
        Point2D predictedPosition = targetPosition + (targetVelocity * castDelay);

        // Check if predicted position is within cast range
        double distanceToCast = (predictedPosition - casterPosition).Length;
        if (distanceToCast > spellRange + Constants.RangeTolerance)
        {
            return null; // Out of range
        }

        // For circular spells, we aim directly at where target will be
        // The "distance from center" will be 0 for perfect center hit
        return new CircularInterceptResult(
            CastPosition: predictedPosition,
            PredictedTargetPosition: predictedPosition,
            DetonationTime: castDelay,
            DistanceFromCenter: 0);
    }

    /// <summary>
    /// Solves for BEHIND-TARGET cast position of a circular skillshot.
    ///
    /// Strategy: Instead of aiming at target center, we aim BEHIND them.
    /// This makes the target walk INTO the spell, requiring them to REVERSE
    /// direction to dodge - much harder than simply stopping or turning.
    ///
    /// Example for Xerath W on target moving north:
    ///   - Target at (500, 500) moving north at 350 u/s
    ///   - Delay = 0.5s → Target will be at (500, 325) at detonation
    ///   - With behindMargin = 1px and effectiveRadius = 265 (200 + 65)
    ///   - We aim at (500, 325 + 264) = (500, 589) - SOUTH of predicted position
    ///   - Target walks north INTO the spell from behind
    /// </summary>
    /// <param name="casterPosition">Position of the caster (for range validation)</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="spellRadius">Radius of the circular spell effect</param>
    /// <param name="castDelay">Time from cast to detonation</param>
    /// <param name="targetHitboxRadius">Target's hitbox radius</param>
    /// <param name="spellRange">Maximum cast range from caster</param>
    /// <param name="behindMargin">How far inside the edge to hit (default 1 pixel)</param>
    /// <returns>Circular intercept result with behind-target aim point</returns>
    public static CircularInterceptResult? SolveCircularBehindTarget(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double spellRadius,
        double castDelay,
        double targetHitboxRadius,
        double spellRange,
        double behindMargin = 1.0,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Adaptive)
    {
        // Input validation
        if (spellRadius < 0)
            throw new ArgumentException("Spell radius cannot be negative", nameof(spellRadius));
        if (castDelay < 0)
            throw new ArgumentException("Cast delay cannot be negative", nameof(castDelay));
        if (spellRange <= 0)
            throw new ArgumentException("Spell range must be positive", nameof(spellRange));
        if (targetHitboxRadius < 0)
            throw new ArgumentException("Target hitbox radius cannot be negative", nameof(targetHitboxRadius));
        if (behindMargin < 0)
            throw new ArgumentException("Behind margin cannot be negative", nameof(behindMargin));

        double effectiveRadius = spellRadius + targetHitboxRadius;

        // Predict target position at detonation time
        Point2D predictedPosition = targetPosition + (targetVelocity * castDelay);

        // For stationary targets, aim at center (no "behind" direction)
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            double stationaryDistanceToCast = (predictedPosition - casterPosition).Length;
            if (stationaryDistanceToCast > spellRange + Constants.RangeTolerance)
                return null;

            return new CircularInterceptResult(
                CastPosition: predictedPosition,
                PredictedTargetPosition: predictedPosition,
                DetonationTime: castDelay,
                DistanceFromCenter: 0);
        }

        // Calculate aim point based on strategy
        Point2D castPosition = CalculateBehindInitialPoint(
            casterPosition, predictedPosition, targetVelocity, effectiveRadius, behindMargin, strategy);

        // Check if cast position is within range
        double distanceToCast = (castPosition - casterPosition).Length;
        if (distanceToCast > spellRange + Constants.RangeTolerance)
        {
            // Behind position out of range - try center aim as fallback
            distanceToCast = (predictedPosition - casterPosition).Length;
            if (distanceToCast > spellRange + Constants.RangeTolerance)
                return null;

            return new CircularInterceptResult(
                CastPosition: predictedPosition,
                PredictedTargetPosition: predictedPosition,
                DetonationTime: castDelay,
                DistanceFromCenter: 0);
        }

        // Distance from spell center to target at detonation
        double finalOffset = (castPosition - predictedPosition).Length;
        return new CircularInterceptResult(
            CastPosition: castPosition,
            PredictedTargetPosition: predictedPosition,
            DetonationTime: castDelay,
            DistanceFromCenter: finalOffset);
    }

    /// <summary>
    /// Solves for circular intercept with path-based target movement.
    /// Handles multi-waypoint paths by predicting position at detonation time.
    /// </summary>
    public static CircularInterceptResult? SolveCircularPathIntercept(
        Point2D casterPosition,
        TargetPath path,
        double spellRadius,
        double castDelay,
        double targetHitboxRadius,
        double spellRange)
    {
        if (castDelay < 0)
            throw new ArgumentException("Cast delay cannot be negative", nameof(castDelay));

        double effectiveRadius = spellRadius + targetHitboxRadius;

        // Get position at detonation time using path prediction
        Point2D predictedPosition = path.GetPositionAtTime(castDelay);

        // Check if within cast range
        double distanceToCast = (predictedPosition - casterPosition).Length;
        if (distanceToCast > spellRange + Constants.RangeTolerance)
            return null;

        return new CircularInterceptResult(
            CastPosition: predictedPosition,
            PredictedTargetPosition: predictedPosition,
            DetonationTime: castDelay,
            DistanceFromCenter: 0);
    }

    /// <summary>
    /// Solves for behind-target circular intercept with path-based movement.
    /// Uses the velocity at detonation time to determine "behind" direction.
    /// </summary>
    public static CircularInterceptResult? SolveCircularPathBehindTarget(
        Point2D casterPosition,
        TargetPath path,
        double spellRadius,
        double castDelay,
        double targetHitboxRadius,
        double spellRange,
        double behindMargin = 1.0,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Adaptive)
    {
        if (castDelay < 0)
            throw new ArgumentException("Cast delay cannot be negative", nameof(castDelay));
        if (behindMargin < 0)
            throw new ArgumentException("Behind margin cannot be negative", nameof(behindMargin));

        double effectiveRadius = spellRadius + targetHitboxRadius;

        // Get position at detonation time
        Point2D predictedPosition = path.GetPositionAtTime(castDelay);

        // Get velocity at detonation time (might be on different segment)
        Vector2D velocityAtDetonation = path.GetVelocityAtTime(castDelay);

        // For stationary target (at detonation time), aim at center
        if (velocityAtDetonation.Length < Constants.MinVelocity)
        {
            double stationaryDistanceToCast = (predictedPosition - casterPosition).Length;
            if (stationaryDistanceToCast > spellRange + Constants.RangeTolerance)
                return null;

            return new CircularInterceptResult(
                CastPosition: predictedPosition,
                PredictedTargetPosition: predictedPosition,
                DetonationTime: castDelay,
                DistanceFromCenter: 0);
        }

        // Calculate aim point based on strategy
        Point2D castPosition = CalculateBehindInitialPoint(
            casterPosition, predictedPosition, velocityAtDetonation, effectiveRadius, behindMargin, strategy);

        // Check range
        double distanceToCast = (castPosition - casterPosition).Length;
        if (distanceToCast > spellRange + Constants.RangeTolerance)
        {
            // Fallback to center aim
            distanceToCast = (predictedPosition - casterPosition).Length;
            if (distanceToCast > spellRange + Constants.RangeTolerance)
                return null;

            return new CircularInterceptResult(
                CastPosition: predictedPosition,
                PredictedTargetPosition: predictedPosition,
                DetonationTime: castDelay,
                DistanceFromCenter: 0);
        }

        double finalOffset = (castPosition - predictedPosition).Length;
        return new CircularInterceptResult(
            CastPosition: castPosition,
            PredictedTargetPosition: predictedPosition,
            DetonationTime: castDelay,
            DistanceFromCenter: finalOffset);
    }

    /// <summary>
    /// Calculates confidence for circular skillshot prediction.
    /// Circular spells have different confidence factors than linear ones:
    /// - No flight time uncertainty (instant)
    /// - Delay uncertainty is primary factor
    /// - Target speed relative to spell radius matters
    /// </summary>
    public static double CalculateCircularConfidence(
        double castDelay,
        double targetSpeed,
        double spellRadius,
        double targetHitboxRadius)
    {
        double effectiveRadius = spellRadius + targetHitboxRadius;

        // Delay factor: longer delays = more uncertainty
        // Exponential decay with time constant of 2 seconds
        double delayFactor = Math.Exp(-castDelay / 2.0);

        // Speed/radius factor: fast targets relative to spell size are harder
        // How far can target travel during delay relative to spell radius?
        double travelDuringDelay = targetSpeed * castDelay;
        double coverageRatio = effectiveRadius / (travelDuringDelay + effectiveRadius);

        // Equal weight combination (0.5 + 0.5)
        double confidence = (0.5 * delayFactor) + (0.5 * coverageRatio);

        return Math.Clamp(confidence, Constants.Epsilon, 1.0);
    }
}
