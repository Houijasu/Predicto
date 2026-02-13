using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

public sealed partial class InterceptSolver
{
    /// <summary>
    /// Solves for hitscan/instant beam interception.
    ///
    /// Mathematical model:
    /// - After castDelay, the beam exists INSTANTLY from caster to Range along an aim direction
    /// - Collision occurs when target's hitbox circle is within (width/2) distance from beam line
    /// - The earliest possible hit is at t = castDelay if target is within range
    ///
    /// For hitscan, the intercept time is simply castDelay (the beam fires instantly).
    /// The aim point is the target's predicted position at castDelay.
    ///
    /// Edge collision model: target center must be within (targetRadius + width/2) of beam centerline.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="castDelay">Delay before beam fires (seconds)</param>
    /// <param name="targetHitboxRadius">Radius of target's hitbox</param>
    /// <param name="beamWidth">Width of the beam</param>
    /// <param name="beamRange">Maximum range of the beam</param>
    /// <returns>Tuple of (AimPoint, PredictedTargetPosition, InterceptTime) or null if unreachable</returns>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveHitscanIntercept(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double castDelay,
        double targetHitboxRadius,
        double beamWidth,
        double beamRange)
    {
        // Input validation
        if (castDelay < 0)
            throw new ArgumentException($"Cast delay cannot be negative, got {castDelay}", nameof(castDelay));
        ValidateCollisionInputs(targetHitboxRadius, beamWidth);
        if (beamRange <= 0)
            throw new ArgumentException($"Beam range must be positive, got {beamRange}", nameof(beamRange));

        // Effective collision radius: beam edge touches target edge
        double effectiveRadius = targetHitboxRadius + (beamWidth / 2);

        // Predict target position at cast delay (when beam fires)
        Point2D predictedPos = CalculatePredictedPosition(targetPosition, targetVelocity, castDelay);

        // Check if target is within range at fire time
        var displacement = predictedPos - casterPosition;
        double distance = displacement.Length;

        // For hitscan, we can hit if target center is within (range + effectiveRadius)
        // because beam edge can still touch target edge
        if (distance > beamRange + effectiveRadius)
        {
            return null; // Target out of range
        }

        // The aim point is the predicted target position at cast delay
        // The beam fires instantly, so intercept time = castDelay
        return (predictedPos, predictedPos, castDelay);
    }

    /// <summary>
    /// Solves for hitscan/instant beam interception with "behind target" aiming.
    ///
    /// For hitscan beams, we aim at the trailing edge of the target's hitbox
    /// to maximize hit probability (target moving into the beam).
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="castDelay">Delay before beam fires (seconds)</param>
    /// <param name="targetHitboxRadius">Radius of target's hitbox</param>
    /// <param name="beamWidth">Width of the beam</param>
    /// <param name="beamRange">Maximum range of the beam</param>
    /// <param name="behindMargin">Safety margin behind target (default 1.0)</param>
    /// <returns>Tuple of (AimPoint, PredictedTargetPosition, InterceptTime) or null if unreachable</returns>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveHitscanBehindTarget(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double castDelay,
        double targetHitboxRadius,
        double beamWidth,
        double beamRange,
        double behindMargin = 1.0,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Adaptive)
    {
        // Input validation
        if (castDelay < 0)
            throw new ArgumentException($"Cast delay cannot be negative, got {castDelay}", nameof(castDelay));
        ValidateCollisionInputs(targetHitboxRadius, beamWidth);
        if (beamRange <= 0)
            throw new ArgumentException($"Beam range must be positive, got {beamRange}", nameof(beamRange));
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (beamWidth / 2);

        // Predict target position at cast delay
        Point2D predictedPos = CalculatePredictedPosition(targetPosition, targetVelocity, castDelay);

        // Check range
        var displacement = predictedPos - casterPosition;
        double distance = displacement.Length;

        if (distance > beamRange + effectiveRadius)
        {
            return null; // Target out of range
        }

        // For stationary targets, aim at center
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            return (predictedPos, predictedPos, castDelay);
        }

        // Calculate aim point based on strategy at the firing time
        Point2D aimPoint = CalculateBehindInitialPoint(
            casterPosition, predictedPos, targetVelocity, effectiveRadius, behindMargin, strategy);

        // Verify aim point is within range
        double aimDistance = (aimPoint - casterPosition).Length;
        if (aimDistance > beamRange)
        {
            // Fall back to center aim if behind-aim is out of range
            return (predictedPos, predictedPos, castDelay);
        }

        return (aimPoint, predictedPos, castDelay);
    }

    /// <summary>
    /// Solves for hitscan intercept when target is following a multi-waypoint path.
    /// Returns the aim point and intercept time for an instant beam.
    /// </summary>
    public static PathInterceptResult? SolveHitscanPathIntercept(
        Point2D casterPosition,
        TargetPath path,
        double castDelay,
        double targetHitboxRadius,
        double beamWidth,
        double beamRange)
    {
        if (castDelay < 0)
            throw new ArgumentException($"Cast delay cannot be negative, got {castDelay}", nameof(castDelay));
        ValidateCollisionInputs(targetHitboxRadius, beamWidth);
        if (beamRange <= 0)
            throw new ArgumentException($"Beam range must be positive, got {beamRange}", nameof(beamRange));

        double effectiveRadius = targetHitboxRadius + (beamWidth / 2);

        // Get target position at cast delay
        Point2D predictedPos = path.GetPositionAtTime(castDelay);

        // Check if within range
        var displacement = predictedPos - casterPosition;
        double distance = displacement.Length;

        if (distance > beamRange + effectiveRadius)
        {
            return null; // Target out of range at fire time
        }

        // Find which waypoint segment the target will be on at castDelay
        int waypointIndex = 0;
        double remainingDistance = path.Speed * castDelay;
        var position = path.CurrentPosition;

        while (remainingDistance > 0 && waypointIndex < path.Waypoints.Count)
        {
            var target = path.Waypoints[waypointIndex];
            var distToWaypoint = (target - position).Length;

            if (distToWaypoint <= remainingDistance)
            {
                position = target;
                remainingDistance -= distToWaypoint;
                waypointIndex++;
            }
            else
            {
                break;
            }
        }

        // Clamp to valid range
        if (waypointIndex >= path.Waypoints.Count)
        {
            waypointIndex = path.Waypoints.Count - 1;
        }

        return new PathInterceptResult(predictedPos, predictedPos, castDelay, waypointIndex);
    }
}
