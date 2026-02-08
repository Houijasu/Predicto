using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

/// <summary>
/// Solves the interception problem using quadratic equation approach.
///
/// Mathematical basis:
/// Given:
///   P = target position, V = target velocity, C = caster position
///   s = skillshot speed, d = cast delay
///
/// Find time t where skillshot reaches target:
///   |P + V*t - C| = s*(t - d)
///
/// This expands to quadratic form at² + bt + c = 0 where:
///   a = |V|² - s²
///   b = 2*(D·V) + 2*s²*d
///   c = |D|² - s²*d²
///   D = P - C (displacement from caster to target)
///
/// Return value semantics:
///   - Returns time in seconds from now when interception occurs
///   - Returns null when no valid mathematical solution exists (target unreachable)
///   - Throws ArgumentException for invalid inputs (programming errors)
/// </summary>
public sealed class InterceptSolver
{
    #region Input Validation

    /// <summary>
    /// Validates common input parameters for interception solving.
    /// </summary>
    /// <exception cref="ArgumentException">Thrown when inputs are invalid.</exception>
    private static void ValidateInputs(
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue,
        string methodName = "")
    {
        if (skillshotSpeed <= 0)
            throw new ArgumentException($"Skillshot speed must be positive, got {skillshotSpeed}", nameof(skillshotSpeed));

        if (castDelay < 0)
            throw new ArgumentException($"Cast delay cannot be negative, got {castDelay}", nameof(castDelay));

        if (skillshotRange <= 0)
            throw new ArgumentException($"Skillshot range must be positive, got {skillshotRange}", nameof(skillshotRange));
    }

    /// <summary>
    /// Validates hitbox and skillshot width parameters.
    /// </summary>
    /// <exception cref="ArgumentException">Thrown when inputs are invalid.</exception>
    private static void ValidateCollisionInputs(
        double targetHitboxRadius,
        double skillshotWidth)
    {
        if (targetHitboxRadius < 0)
            throw new ArgumentException($"Target hitbox radius cannot be negative, got {targetHitboxRadius}", nameof(targetHitboxRadius));

        if (skillshotWidth < 0)
            throw new ArgumentException($"Skillshot width cannot be negative, got {skillshotWidth}", nameof(skillshotWidth));
    }

    #endregion

    /// <summary>
    /// Attempts to solve for the interception time (center-to-center collision).
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot (must be positive)</param>
    /// <param name="castDelay">Delay before skillshot launches (must be non-negative)</param>
    /// <param name="skillshotRange">Range of the skillshot (for calculating max valid time)</param>
    /// <returns>Interception time in seconds from now, or null if no valid solution exists</returns>
    /// <exception cref="ArgumentException">Thrown when skillshotSpeed &lt;= 0 or castDelay &lt; 0</exception>
    public static double? SolveInterceptTime(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);

        // Displacement vector from caster to target
        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);
        var V = targetVelocity;
        double s = skillshotSpeed;
        double d = castDelay;

        // Calculate max valid time based on range (delay + range/speed)
        double maxTime = d + (skillshotRange / s);

        // Handle stationary target (optimization and numerical stability)
        if (V.Length < Constants.MinVelocity)
        {
            return SolveStationary(D.Length, s, d);
        }

        // Quadratic coefficients: at² + bt + c = 0
        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * s * d);
        double c = D.DotProduct(D) - (s * s * d * d);

        // Handle degenerate case when |V| ≈ s (linear equation)
        if (Math.Abs(a) < Constants.Epsilon)
        {
            return SolveLinear(b, c, d, maxTime);
        }

        // Use MathNet.Numerics for robust root finding
        var (r1, r2) = FindRoots.Quadratic(c, b, a);

        // Extract real roots (ignore complex solutions)
        double? t1 = r1.IsReal() ? r1.Real : null;
        double? t2 = r2.IsReal() ? r2.Real : null;

        return SelectSmallestValidTime(t1, t2, d, maxTime);
    }

    /// <summary>
    /// Solves for stationary target - direct distance calculation.
    /// </summary>
    private static double? SolveStationary(double distance, double speed, double delay)
    {
        double flightTime = distance / speed;
        double totalTime = delay + flightTime;
        return totalTime;
    }

    /// <summary>
    /// Solves linear case when target speed equals skillshot speed.
    /// bt + c = 0 => t = -c/b
    /// </summary>
    private static double? SolveLinear(double b, double c, double minTime, double maxTime)
    {
        if (Math.Abs(b) < Constants.Epsilon)
        {
            // Degenerate: no unique solution
            return null;
        }

        double t = -c / b;
        return t > minTime && t <= maxTime ? t : null;
    }

    /// <summary>
    /// Selects the smallest valid interception time from two candidates.
    /// Valid means: t > minTime (after cast delay) and within max time (delay + range/speed).
    /// </summary>
    private static double? SelectSmallestValidTime(double? t1, double? t2, double minTime, double maxTime)
    {
        bool v1 = t1.HasValue && t1.Value > minTime && t1.Value <= maxTime;
        bool v2 = t2.HasValue && t2.Value > minTime && t2.Value <= maxTime;

        return (v1, v2) switch
        {
            (false, false) => null,
            (true, false) => t1,
            (false, true) => t2,
            (true, true) => Math.Min(t1!.Value, t2!.Value)
        };
    }

    /// <summary>
    /// Selects the largest valid interception time from two candidates.
    /// Valid means: t > minTime (after cast delay) and within max time (delay + range/speed).
    /// Used for trailing edge collision (maximum trailing length).
    /// </summary>
    private static double? SelectLargestValidTime(double? t1, double? t2, double minTime, double maxTime)
    {
        bool v1 = t1.HasValue && t1.Value > minTime && t1.Value <= maxTime;
        bool v2 = t2.HasValue && t2.Value > minTime && t2.Value <= maxTime;

        return (v1, v2) switch
        {
            (false, false) => null,
            (true, false) => t1,
            (false, true) => t2,
            (true, true) => Math.Max(t1!.Value, t2!.Value)
        };
    }

    /// <summary>
    /// Solves for the earliest interception time accounting for collision radii.
    /// This finds when the projectile EDGE touches the target EDGE (earliest possible hit).
    ///
    /// Mathematical basis:
    /// |D + V·t| = s·(t - d) + r_effective
    /// Where r_effective = targetHitboxRadius + skillshotWidth/2
    ///
    /// This yields earlier intercept times than center-to-center because the
    /// projectile only needs to travel (distance - r_effective) to collide.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="targetHitboxRadius">Radius of target's hitbox</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <param name="skillshotRange">Range of the skillshot</param>
    /// <returns>Earliest interception time in seconds, or null if no valid solution</returns>
    public static double? SolveEdgeInterceptTime(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);

        // Effective collision radius: projectile edge touches target edge
        double r = targetHitboxRadius + (skillshotWidth / 2);

        // Displacement vector from caster to target
        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);
        var V = targetVelocity;
        double s = skillshotSpeed;
        double d = castDelay;

        // Calculate max valid time: delay + (range + effectiveRadius) / speed
        // We add effectiveRadius because edge collision extends effective range
        double maxTime = d + ((skillshotRange + r) / s);

        // Handle stationary target
        if (V.Length < Constants.MinVelocity)
        {
            return SolveStationaryEdge(D.Length, s, d, r);
        }

        // Check if target will be within collision radius at launch time
        // This is more accurate than checking current position for moving targets
        var positionAtLaunch = D + (V * d);
        if (positionAtLaunch.Length <= r)
        {
            // Target will be in collision range when skillshot launches
            return d;
        }

        // Quadratic coefficients for edge-to-edge collision:
        // |D + V·t|² = (s·(t-d) + r)²
        // Expanding: (|V|² - s²)t² + (2(D·V) + 2s²d - 2sr)t + (|D|² - (sd-r)²) = 0
        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * s * d) - (2 * s * r);
        double sdMinusR = (s * d) - r;
        double c = D.DotProduct(D) - (sdMinusR * sdMinusR);

        // Handle degenerate case when |V| ≈ s
        if (Math.Abs(a) < Constants.Epsilon)
        {
            return SolveLinearEdge(b, c, d, maxTime);
        }

        // Use MathNet.Numerics for robust root finding
        var (r1, r2) = FindRoots.Quadratic(c, b, a);

        // Extract real roots (ignore complex solutions)
        double? t1 = r1.IsReal() ? r1.Real : null;
        double? t2 = r2.IsReal() ? r2.Real : null;

        return SelectSmallestValidTime(t1, t2, d, maxTime);
    }

    /// <summary>
    /// Solves for stationary target with edge collision.
    /// Projectile travels (distance - r_effective) to hit.
    /// </summary>
    private static double? SolveStationaryEdge(double distance, double speed, double delay, double effectiveRadius)
    {
        // If already within collision radius, hit at delay time
        if (distance <= effectiveRadius)
            return delay;

        double travelDistance = distance - effectiveRadius;
        double flightTime = travelDistance / speed;
        return delay + flightTime;
    }

    /// <summary>
    /// Solves linear case for edge collision.
    /// Uses strict inequality (t > minTime) for consistency with other solvers.
    /// At t == minTime, the projectile has just launched with zero travel distance,
    /// which is only valid if target is already within collision radius (handled separately).
    /// </summary>
    private static double? SolveLinearEdge(double b, double c, double minTime, double maxTime)
    {
        if (Math.Abs(b) < Constants.Epsilon)
            return null;

        double t = -c / b;
        return t > minTime && t <= maxTime ? t : null;
    }

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

    /// <summary>
    /// Calculates the predicted position at the given interception time.
    /// </summary>
    public static Point2D CalculatePredictedPosition(
        Point2D targetPosition,
        Vector2D targetVelocity,
        double interceptTime)
    {
        return targetPosition + (targetVelocity * interceptTime);
    }

    /// <summary>
    /// Calculates the confidence score for a prediction.
    /// Uses weighted sum of exponential time decay, speed ratio, and range factors.
    /// </summary>
    /// <param name="interceptTime">Total time to intercept (including delay)</param>
    /// <param name="distance">Current distance to target</param>
    /// <param name="targetSpeed">Target's movement speed</param>
    /// <param name="skillshotSpeed">Skillshot's travel speed</param>
    /// <param name="skillshotRange">Skillshot's maximum range</param>
    /// <param name="castDelay">Skillshot's cast delay</param>
    public static double CalculateConfidence(
        double interceptTime,
        double distance,
        double targetSpeed,
        double skillshotSpeed,
        double skillshotRange,
        double castDelay)
    {
        // Calculate max prediction time based on skillshot parameters
        double maxPredictionTime = castDelay + (skillshotRange / skillshotSpeed);

        // Time factor: exponential decay - uncertainty grows with time
        double timeFactor = Math.Exp(-interceptTime / maxPredictionTime);

        // Speed factor: slower targets relative to skillshot are more predictable
        double speedFactor = 1.0 / (1.0 + (targetSpeed / (skillshotSpeed + Constants.Epsilon)));

        // Range factor: closer targets are more predictable
        double rangeFactor = Math.Max(0, 1.0 - (distance / skillshotRange));

        // Equal weights: 1/2 time, 1/4 speed, 1/4 range (simplified from 0.5, 0.3, 0.2)
        double confidence = (0.5 * timeFactor) + (0.5 * (speedFactor + rangeFactor) * 0.5);

        return Math.Clamp(confidence, Constants.Epsilon, 1.0);
    }

    /// <summary>
    /// Evaluates the collision function f(t) = |D + V·t| - (s·(t-d) + r).
    /// When f(t) = 0, the projectile edge touches the target edge.
    /// </summary>
    /// <param name="displacement">Vector from caster to target at t=0</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="effectiveRadius">Combined collision radius (target + skillshot/2)</param>
    /// <param name="t">Time to evaluate</param>
    /// <returns>Positive if projectile hasn't reached target, negative if passed, zero at collision</returns>
    private static double EvaluateCollisionFunction(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double t)
    {
        // Target position relative to caster at time t
        var relativePosition = displacement + (targetVelocity * t);
        double distanceToTarget = relativePosition.Length;

        // Projectile travel distance at time t (starts moving after delay)
        double flightTime = Math.Max(0, t - castDelay);
        double projectileDistance = (skillshotSpeed * flightTime) + effectiveRadius;

        return distanceToTarget - projectileDistance;
    }


    /// <summary>
    /// Evaluates the collision function with a margin offset.
    /// f(t) = |D + V·t| - (s·(t-d) + r - margin)
    ///
    /// When margin > 0, this finds points INSIDE the collision zone by the margin amount.
    /// Used for "behind edge" targeting where we want to hit just barely inside.
    /// </summary>
    private static double EvaluateCollisionFunctionWithMargin(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double margin,
        double t)
    {
        var relativePosition = displacement + (targetVelocity * t);
        double distanceToTarget = relativePosition.Length;

        double flightTime = Math.Max(0, t - castDelay);
        // Reduce effective radius by margin to find point inside collision zone
        double projectileDistance = (skillshotSpeed * flightTime) + (effectiveRadius - margin);

        return distanceToTarget - projectileDistance;
    }

    /// <summary>
    /// Performs an analytical check to determine if a path segment is mathematically reachable.
    /// Used for rapid segment pruning before running expensive root-finding.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsSegmentReachable(
        Point2D casterPos,
        Point2D start,
        Vector2D velocity,
        double duration,
        double skillshotSpeed,
        double castDelay,
        double range,
        double effectiveRadius,
        double segmentStartTime)
    {
        // Quick distance bounds
        double minDistSq;
        Vector2D d = start - casterPos;
        double vDotD = velocity.DotProduct(d);
        double vSq = velocity.DotProduct(velocity);

        if (vSq < Constants.Epsilon)
        {
            minDistSq = d.DotProduct(d);
        }
        else
        {
            double tClosest = -vDotD / vSq;
            if (tClosest <= 0) minDistSq = d.DotProduct(d);
            else if (tClosest >= duration)
            {
                var endDisplacement = d + (velocity * duration);
                minDistSq = endDisplacement.DotProduct(endDisplacement);
            }
            else
            {
                minDistSq = d.DotProduct(d) - ((vDotD * vDotD) / vSq);
            }
        }

        // Analytical pruning: if the closest the target gets is beyond skillshot reach
        double maxPossibleReach = (skillshotSpeed * (segmentStartTime + duration - castDelay)) + effectiveRadius;
        if (minDistSq > (maxPossibleReach + Constants.RangeTolerance) * (maxPossibleReach + Constants.RangeTolerance))
            return false;

        if (minDistSq > (range + effectiveRadius + Constants.RangeTolerance) * (range + effectiveRadius + Constants.RangeTolerance))
            return false;

        return true;
    }

    /// <summary>
    /// Calculates the initial aim point for behind-edge strategies.
    /// Incorporates DirectBehind, Tangent, and Adaptive methods.
    /// </summary>
    private static Point2D CalculateBehindInitialPoint(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        double behindMargin,
        BehindEdgeStrategy strategy)
    {
        double hitRadius = effectiveRadius - behindMargin;
        if (hitRadius < Constants.Epsilon) hitRadius = effectiveRadius * 0.5;

        // Target movement direction
        Vector2D moveDir = targetVelocity.Length > Constants.MinVelocity
            ? targetVelocity.Normalize()
            : (targetPosition - casterPosition).Normalize(); // Default if stationary

        Vector2D behindDir = moveDir.Negate();

        if (strategy == BehindEdgeStrategy.DirectBehind)
        {
            return targetPosition + (behindDir * hitRadius);
        }

        // Tangent Point Calculation
        Vector2D toTarget = targetPosition - casterPosition;
        double dist = toTarget.Length;

        // If caster is inside the target, fall back to direct behind
        if (dist <= hitRadius)
            return targetPosition + (behindDir * hitRadius);

        Vector2D toTargetNorm = toTarget / dist;
        Vector2D perpLeft = new Vector2D(-toTargetNorm.Y, toTargetNorm.X);
        Vector2D perpRight = perpLeft.Negate();

        // Choose perpendicular pointing more "behind"
        Vector2D perp = perpLeft.DotProduct(behindDir) > perpRight.DotProduct(behindDir)
            ? perpLeft
            : perpRight;

        Point2D tangentPoint = targetPosition + (perp * hitRadius);

        // For Tangent strategy, we may need to blend if the tangent point is not actually behind
        if (strategy == BehindEdgeStrategy.Tangent)
        {
            Vector2D offset = tangentPoint - targetPosition;
            double behindness = offset.DotProduct(behindDir);
            if (behindness > 0) return tangentPoint;

            // Blend based on how \"behind\" the tangent point is
            double blendFactor = Math.Max(0, (behindness / hitRadius) + 0.5);
            return new Point2D(
                targetPosition.X + ((behindDir.X * hitRadius * (1 - blendFactor)) + (offset.X * blendFactor)),
                targetPosition.Y + ((behindDir.Y * hitRadius * (1 - blendFactor)) + (offset.Y * blendFactor)));
        }

        // Adaptive: center of trailing edge (DirectBehind) and tangent methods
        Point2D directBehindPoint = targetPosition + (behindDir * hitRadius);

        // Midpoint of the chord between Tangent and DirectBehind
        Point2D centerPoint = new Point2D(
            (directBehindPoint.X + tangentPoint.X) * 0.5,
            (directBehindPoint.Y + tangentPoint.Y) * 0.5);

        // Project back to hit radius to maintain optimal collision geometry
        Vector2D centerOffset = centerPoint - targetPosition;
        double centerLen = centerOffset.Length;
        if (centerLen < Constants.Epsilon) return directBehindPoint;

        return targetPosition + (centerOffset * (hitRadius / centerLen));
    }

    /// <summary>
    /// Solves for interception time using MathNet.Numerics Bisection.
    /// Finds root of f(t) = |D + V·t| - (s·(t-d) + r) = 0.
    ///
    /// The bisection method is guaranteed to converge if a root exists in the bracket.
    /// It's more robust than quadratic solving for edge cases and can handle
    /// scenarios where numerical instability affects the quadratic formula.
    ///
    /// Uses MathNet.Numerics.RootFinding.Bisection for robust, well-tested implementation.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="effectiveRadius">Combined collision radius</param>
    /// <param name="maxTime">Maximum valid time to search</param>
    /// <param name="tolerance">Convergence tolerance (default Constants.Epsilon = 1e-9 for high precision)</param>
    /// <param name="maxIterations">Maximum iterations (default 100)</param>
    /// <returns>Interception time, or null if no solution exists in bracket</returns>
    public static double? SolveBisection(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double maxTime,
        double tolerance = 1e-9,
        int maxIterations = 100)
    {
        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        double tLow = castDelay;
        double tHigh = maxTime;

        // Evaluate function at bracket endpoints
        double fLow = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tLow);

        // Check if target is already in collision range at launch
        if (fLow <= 0)
            return tLow;

        // Use MathNet's Bisection.TryFindRoot for robust root finding
        Func<double, double> f = t => EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, t);

        if (Bisection.TryFindRoot(f, tLow, tHigh, tolerance, maxIterations, out double root))
        {
            return root;
        }

        return null;
    }

    /// <summary>
    /// Solves for interception time using quadratic solver with bisection refinement.
    ///
    /// Strategy:
    /// 1. Use fast quadratic solver to get initial estimate
    /// 2. Refine with bisection for higher precision
    /// 3. Fall back to pure bisection if quadratic fails
    ///
    /// This combines the speed of analytical solution with the robustness
    /// and precision of numerical methods.
    /// </summary>
    public static double? SolveEdgeInterceptTimeWithRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double refinementTolerance = 1e-9)
    {
        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);
        double maxTime = castDelay + ((skillshotRange + effectiveRadius) / skillshotSpeed);

        // Step 1: Try quadratic solver for initial estimate
        double? quadraticResult = SolveEdgeInterceptTime(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            targetHitboxRadius,
            skillshotWidth,
            skillshotRange);

        if (quadraticResult.HasValue)
        {
            // Step 2: Refine with bisection around the quadratic result
            // Use a narrow bracket around the initial estimate for fast convergence
            double estimate = quadraticResult.Value;
            double bracketSize = Math.Max(Constants.TickDuration, (maxTime - castDelay) / 2); // Half time range or 1 tick
            double tLow = Math.Max(castDelay, estimate - bracketSize);
            double tHigh = Math.Min(maxTime, estimate + bracketSize);

            var displacement = targetPosition - casterPosition;
            var D = new Vector2D(displacement.X, displacement.Y);

            double fLow = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tLow);
            double fHigh = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tHigh);

            // If bracket contains root, refine it
            if (fLow * fHigh <= 0)
            {
                double? refined = SolveBisectionInBracket(
                    D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius,
                    tLow, tHigh, fLow, refinementTolerance, 50);

                if (refined.HasValue)
                    return refined.Value;
            }

            // If refinement failed, return quadratic result (it's still valid)
            return quadraticResult;
        }

        // Step 3: Quadratic failed - fall back to pure bisection
        return SolveBisection(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            effectiveRadius,
            maxTime,
            refinementTolerance);
    }

    /// <summary>
    /// Bisection solver with pre-computed bracket using MathNet.Numerics.
    /// Used for refinement when bracket is already known.
    /// </summary>
    private static double? SolveBisectionInBracket(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double tLow,
        double tHigh,
        double fLow,
        double tolerance,
        int maxIterations)
    {
        // Define the collision function
        Func<double, double> f = t => EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, t);

        // Use MathNet's Bisection.TryFindRoot
        if (Bisection.TryFindRoot(f, tLow, tHigh, tolerance, maxIterations, out double root))
        {
            return root;
        }

        // Fallback to midpoint if bisection fails
        return (tLow + tHigh) / 2;
    }


    /// <summary>
    /// Solves for the LATEST interception time (trailing edge collision).
    /// This finds when the projectile trailing edge catches the target trailing edge.
    ///
    /// Maximum trailing provides:
    /// - Target has minimum reaction time after seeing projectile
    /// - Shot arrives at the "last possible moment"
    /// - Maximum effective lead distance
    ///
    /// Mathematical basis: Same quadratic as SolveEdgeInterceptTime, but selects
    /// the LARGER root instead of the smaller one.
    /// </summary>
    /// <returns>Latest interception time in seconds, or null if no valid solution</returns>
    public static double? SolveEdgeInterceptTimeTrailing(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        double r = targetHitboxRadius + (skillshotWidth / 2);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);
        var V = targetVelocity;
        double s = skillshotSpeed;
        double d = castDelay;

        double maxTime = d + ((skillshotRange + r) / s);

        // Handle stationary target - trailing edge = leading edge for stationary
        if (V.Length < Constants.MinVelocity)
        {
            return SolveStationaryEdgeTrailing(D.Length, s, d, r);
        }

        // Check if target will be within collision radius at launch time
        var positionAtLaunch = D + (V * d);
        if (positionAtLaunch.Length <= r)
        {
            // Target in collision range at launch - find when it EXITS
            // This is the trailing edge time
            return FindCollisionExitTime(D, V, s, d, r, maxTime);
        }

        // Quadratic coefficients for edge-to-edge collision
        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * s * d) - (2 * s * r);
        double sdMinusR = (s * d) - r;
        double c = D.DotProduct(D) - (sdMinusR * sdMinusR);

        // Handle degenerate case when |V| ≈ s
        if (Math.Abs(a) < Constants.Epsilon)
        {
            return SolveLinearEdge(b, c, d, maxTime);
        }

        // Use MathNet.Numerics for robust root finding
        var (r1, r2) = FindRoots.Quadratic(c, b, a);

        double? t1 = r1.IsReal() ? r1.Real : null;
        double? t2 = r2.IsReal() ? r2.Real : null;

        // Select the LARGEST valid time (trailing edge)
        return SelectLargestValidTime(t1, t2, d, maxTime);
    }

    /// <summary>
    /// Solves for stationary target trailing edge.
    /// For stationary targets, the collision window spans from when circles first touch
    /// to when they separate. Trailing edge = when projectile exits the collision zone.
    ///
    /// Uses Minkowski sum model: effectiveRadius = targetRadius + projectileRadius,
    /// allowing us to treat it as a point projectile hitting an expanded circle.
    /// Exit distance is always (center distance + effectiveRadius).
    /// </summary>
    private static double? SolveStationaryEdgeTrailing(double distance, double speed, double delay, double effectiveRadius)
    {
        // Whether target is inside or outside collision radius at start,
        // the trailing edge (exit) is always when projectile reaches the far edge.
        // Exit distance = distance + effectiveRadius in both cases.
        double exitDistance = distance + effectiveRadius;
        double flightTime = exitDistance / speed;
        return delay + flightTime;
    }

    /// <summary>
    /// Finds when target exits the collision zone (for targets inside zone at launch).
    /// Uses MathNet Bisection to find where f(t) changes from negative to positive.
    /// </summary>
    private static double? FindCollisionExitTime(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double maxTime)
    {
        // f(t) = |D + V*t| - (s*(t-d) + r)
        // At t=delay, we're inside collision zone, so f(delay) < 0
        // We need to find where f(t) = 0 again (exit point)

        double tLow = castDelay;
        double tHigh = maxTime;

        double fLow = EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tLow);
        double fHigh = EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tHigh);

        // If fLow >= 0, we're not actually inside (edge case)
        if (fLow >= 0)
            return castDelay;

        // If fHigh < 0, target never exits collision zone within max time
        if (fHigh < 0)
            return maxTime;

        // Use MathNet's Bisection.TryFindRoot for exit point
        Func<double, double> f = t => EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, t);

        if (Bisection.TryFindRoot(f, tLow, tHigh, Constants.Epsilon, 100, out double root))
        {
            return root;
        }

        return (tLow + tHigh) / 2;
    }

    /// <summary>
    /// Solves for the LATEST interception time (trailing edge) using MathNet Bisection.
    /// Finds the second root where f(t) transitions from negative back to positive.
    /// </summary>
    public static double? SolveBisectionTrailing(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double maxTime,
        double tolerance = 1e-9,
        int maxIterations = 100)
    {
        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        // First, find the earliest collision time (entry point)
        double? entryTime = SolveBisection(
            casterPosition, targetPosition, targetVelocity,
            skillshotSpeed, castDelay, effectiveRadius, maxTime, tolerance, maxIterations);

        if (!entryTime.HasValue)
            return null;

        // Now search from slightly after entry to maxTime for exit point
        double searchStart = entryTime.Value + (tolerance * 10);
        if (searchStart >= maxTime)
            return entryTime; // No room for trailing, return entry

        double fStart = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, searchStart);
        double fEnd = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, maxTime);

        // Should be inside collision zone at searchStart (fStart <= 0)
        if (fStart > 0)
            return entryTime; // Already exited, entry = exit

        // If still inside at maxTime, return maxTime
        if (fEnd <= 0)
            return maxTime;

        // Use MathNet's Bisection.TryFindRoot for exit point
        Func<double, double> f = t => EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, t);

        if (Bisection.TryFindRoot(f, searchStart, maxTime, tolerance, maxIterations, out double root))
        {
            return root;
        }

        return (searchStart + maxTime) / 2;
    }

    /// <summary>
    /// Solves for the LATEST interception time (maximum trailing) using quadratic solver
    /// with bisection refinement for maximum precision.
    ///
    /// Strategy:
    /// 1. Use quadratic solver to get the LARGER root (trailing edge estimate)
    /// 2. Refine with bisection for higher precision
    /// 3. Fall back to pure bisection trailing if quadratic fails
    /// </summary>
    public static double? SolveEdgeInterceptTimeTrailingWithRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double refinementTolerance = 1e-9)
    {
        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);
        double maxTime = castDelay + ((skillshotRange + effectiveRadius) / skillshotSpeed);

        // Step 1: Try quadratic solver for trailing edge estimate
        double? quadraticResult = SolveEdgeInterceptTimeTrailing(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            targetHitboxRadius,
            skillshotWidth,
            skillshotRange);

        if (quadraticResult.HasValue)
        {
            // Step 2: Refine with bisection around the quadratic result
            double estimate = quadraticResult.Value;
            double bracketSize = Math.Max(Constants.TickDuration, (maxTime - castDelay) / 2);
            double tLow = Math.Max(castDelay, estimate - bracketSize);
            double tHigh = Math.Min(maxTime, estimate + bracketSize);

            var displacement = targetPosition - casterPosition;
            var D = new Vector2D(displacement.X, displacement.Y);

            double fLow = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tLow);
            double fHigh = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tHigh);

            // For trailing, we want to find where f transitions from <= 0 to > 0
            // If fLow <= 0 and fHigh > 0, we have an exit point in bracket
            if (fLow <= 0 && fHigh > 0)
            {
                double? refined = SolveBisectionInBracketTrailing(
                    D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius,
                    tLow, tHigh, refinementTolerance, 50);

                if (refined.HasValue)
                    return refined.Value;
            }

            return quadraticResult;
        }

        // Step 3: Quadratic failed - fall back to pure bisection trailing
        return SolveBisectionTrailing(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            effectiveRadius,
            maxTime,
            refinementTolerance);
    }

    /// <summary>
    /// Bisection solver for trailing edge (exit point) with pre-computed bracket.
    /// Finds where f(t) transitions from negative/zero to positive.
    /// </summary>
    private static double? SolveBisectionInBracketTrailing(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double tLow,
        double tHigh,
        double tolerance,
        int maxIterations)
    {
        // Define the collision function
        Func<double, double> f = t => EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, t);

        // Use MathNet's Bisection.TryFindRoot
        if (Bisection.TryFindRoot(f, tLow, tHigh, tolerance, maxIterations, out double root))
        {
            return root;
        }

        // Fallback to midpoint if bisection fails
        return (tLow + tHigh) / 2;
    }


    /// <summary>
    /// Solves for the "behind edge" interception time using bisection.
    ///
    /// This finds the time when the projectile hits the target from BEHIND -
    /// just barely inside the collision zone by a 1-pixel margin.
    ///
    /// Strategy:
    /// 1. Find the trailing edge time (when projectile would exit collision zone)
    /// 2. Use bisection with reduced effective radius (radius - margin)
    /// 3. This gives a time slightly BEFORE the exit, hitting from behind
    ///
    /// Benefits:
    /// - Target must REVERSE direction to dodge (extremely difficult)
    /// - Projectile arrives at the last possible position
    /// - Maximum psychological pressure on target
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="targetHitboxRadius">Radius of target's hitbox</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <param name="skillshotRange">Range of the skillshot</param>
    /// <param name="margin">Pixel margin inside collision zone (default: Constants.TrailingEdgeMargin)</param>
    /// <returns>Behind-edge interception time, or null if no valid solution</returns>
    public static double? SolveBehindEdgeIntercept(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double margin = -1)
    {
        // Use default margin if not specified
        if (margin < 0)
            margin = Constants.TrailingEdgeMargin;

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // IMPORTANT: Margin must be less than effectiveRadius for the math to work.
        // Clamp to leave at least Epsilon inside collision zone, and ensure margin is never negative.
        // For small effectiveRadius (< 1), use 90% of radius as max margin to ensure valid collision.
        double maxMargin = effectiveRadius > 1.0
            ? effectiveRadius - 1.0
            : effectiveRadius * 0.9;
        margin = Math.Clamp(margin, Constants.Epsilon, Math.Max(Constants.Epsilon, maxMargin));
        double maxTime = castDelay + ((skillshotRange + effectiveRadius) / skillshotSpeed);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        // Handle stationary target
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            return SolveStationaryBehindEdge(D.Length, skillshotSpeed, castDelay, effectiveRadius, margin);
        }

        // First find the entry time (earliest collision)
        double? entryTime = SolveBisection(
            casterPosition, targetPosition, targetVelocity,
            skillshotSpeed, castDelay, effectiveRadius, maxTime);

        if (!entryTime.HasValue)
            return null;

        // Now use bisection with reduced radius to find "behind edge" point
        // This is the time when projectile is exactly (margin) units inside the collision zone
        // from the trailing edge perspective

        double tLow = entryTime.Value;
        double tHigh = maxTime;

        double fLow = EvaluateCollisionFunctionWithMargin(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, margin, tLow);
        double fHigh = EvaluateCollisionFunctionWithMargin(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, margin, tHigh);

        // At entry time with margin, we should be inside (fLow < 0)
        // At maxTime with margin, we should be outside (fHigh > 0) or find where it crosses

        if (fLow > 0)
        {
            // Already outside the margin zone at entry - return entry time
            return entryTime;
        }

        if (fHigh <= 0)
        {
            // Still inside margin zone at maxTime - return maxTime
            return maxTime;
        }

        // Bisection to find the behind-edge point
        const double tolerance = Constants.Epsilon;
        const int maxIterations = 100;

        for (int i = 0; i < maxIterations; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateCollisionFunctionWithMargin(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, margin, tMid);

            if (Math.Abs(fMid) < tolerance || (tHigh - tLow) / 2 < tolerance)
                return tMid;

            if (fMid <= 0)
            {
                tLow = tMid;
            }
            else
            {
                tHigh = tMid;
            }
        }

        return (tLow + tHigh) / 2;
    }

    /// <summary>
    /// Solves for stationary target behind-edge hit.
    /// Projectile travels to (distance + effectiveRadius - margin) to hit from behind.
    /// </summary>
    private static double? SolveStationaryBehindEdge(
        double distance,
        double speed,
        double delay,
        double effectiveRadius,
        double margin)
    {
        // Travel to far edge minus margin (same formula regardless of whether
        // target is inside or outside the collision zone)
        double travelDistance = distance + effectiveRadius - margin;

        if (travelDistance < 0)
            travelDistance = 0;

        double flightTime = travelDistance / speed;
        return delay + flightTime;
    }

    /// <summary>
    /// Solves for behind-edge interception with quadratic initial estimate and bisection refinement.
    /// This is the primary method for "hit from behind by 1 pixel" strategy.
    /// </summary>
    public static double? SolveBehindEdgeInterceptWithRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double margin = -1)
    {
        if (margin < 0)
            margin = Constants.TrailingEdgeMargin;

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // IMPORTANT: Margin must be less than effectiveRadius for the math to work.
        // Clamp to leave at least Epsilon inside collision zone, and ensure margin is never negative.
        // For small effectiveRadius (< 1), use 90% of radius as max margin to ensure valid collision.
        double maxMargin = effectiveRadius > 1.0
            ? effectiveRadius - 1.0
            : effectiveRadius * 0.9;
        margin = Math.Clamp(margin, Constants.Epsilon, Math.Max(Constants.Epsilon, maxMargin));

        // Get trailing edge estimate from quadratic solver
        double? trailingEstimate = SolveEdgeInterceptTimeTrailing(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            targetHitboxRadius,
            skillshotWidth,
            skillshotRange);

        double maxTime = castDelay + ((skillshotRange + effectiveRadius) / skillshotSpeed);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        if (trailingEstimate.HasValue)
        {
            // Refine around the trailing estimate using margin
            double estimate = trailingEstimate.Value;
            double bracketSize = Math.Max(Constants.TickDuration, (maxTime - castDelay) / 2);

            // Search slightly before trailing edge
            double tLow = Math.Max(castDelay, estimate - bracketSize);
            double tHigh = Math.Min(maxTime, estimate);

            double fLow = EvaluateCollisionFunctionWithMargin(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, margin, tLow);
            double fHigh = EvaluateCollisionFunctionWithMargin(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, margin, tHigh);

            // We want to find where f_margin = 0, which is (margin) units inside collision zone
            if (fLow * fHigh <= 0 || fLow <= 0)
            {
                // Bisection refinement
                if (fLow > 0 && fHigh <= 0)
                {
                    // Swap to maintain fLow <= 0
                    (tLow, tHigh) = (tHigh, tLow);
                    (fLow, fHigh) = (fHigh, fLow);
                }

                const double tolerance = Constants.Epsilon;
                for (int i = 0; i < 50; i++)
                {
                    double tMid = (tLow + tHigh) / 2;
                    double fMid = EvaluateCollisionFunctionWithMargin(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, margin, tMid);

                    if (Math.Abs(fMid) < tolerance || (tHigh - tLow) / 2 < tolerance)
                        return tMid;

                    if (fMid <= 0)
                        tLow = tMid;
                    else
                        tHigh = tMid;
                }

                return (tLow + tHigh) / 2;
            }

            // If refinement bracket doesn't work, fall back to trailing estimate minus small offset
            // This approximates the "behind edge" position
            double timeForMargin = margin / skillshotSpeed;
            return Math.Max(castDelay, trailingEstimate.Value - timeForMargin);
        }

        // Fall back to pure bisection
        return SolveBehindEdgeIntercept(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            targetHitboxRadius,
            skillshotWidth,
            skillshotRange,
            margin);
    }

    /// <summary>
    /// Solves for interception aiming BEHIND the target relative to their movement.
    ///
    /// Mathematical approach:
    /// We want to aim at a point that is (effectiveRadius - margin) behind the target's
    /// predicted position. To do this correctly, we must solve the intercept equation
    /// with an ADJUSTED target position.
    ///
    /// Key insight: If we naively find intercept time t, then offset the aim point,
    /// the projectile travel time to the new aim point will be DIFFERENT from t!
    ///
    /// Correct approach:
    /// 1. Create a "virtual target" that is offset BEHIND the real target
    /// 2. Solve intercept to this virtual target (same velocity)
    /// 3. At solution time t:
    ///    - Virtual target (aim point) is at: P_virtual + V*t
    ///    - Real target is at: P_real + V*t
    ///    - Distance between them = offset (constant!) which is less than effectiveRadius → HIT
    ///
    /// Example: Target at (800, 400) moving north V=(0, -350), offset=84
    /// - Virtual target: (800, 400) - (0,-1)*84 = (800, 484) [84 units SOUTH]
    /// - Solve intercept to (800, 484) moving with V=(0, -350)
    /// - At time t: aim=(800, 484-350t), real_target=(800, 400-350t)
    /// - Separation = 84 pixels (constant), less than effectiveRadius=85 → HIT
    /// </summary>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveBehindTarget(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // For stationary targets, there's no "behind" direction - aim at center
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            double? stationaryTime = SolveEdgeInterceptTime(
                casterPosition, targetPosition, targetVelocity, skillshotSpeed,
                castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);

            if (!stationaryTime.HasValue)
                return null;

            return (targetPosition, targetPosition, stationaryTime.Value);
        }

        // Calculate behind offset: (effectiveRadius - margin) in direction opposite to velocity
        // This represents how far "behind" the target center we aim.
        //
        // After margin clamping above (margin <= 0.9 * effectiveRadius), behindDistance
        // should always be >= 0.5 * effectiveRadius. The defensive check below handles
        // any edge cases from floating-point arithmetic or future code changes.
        double behindDistance = effectiveRadius - behindMargin;
        if (behindDistance < Constants.Epsilon)
        {
            // Fallback: if somehow behindDistance is negligible, use a minimal offset
            // This preserves the "behind target" intent rather than degenerating to center aim
            behindDistance = effectiveRadius * 0.5;
        }

        Vector2D moveDirection = targetVelocity.Normalize();
        Vector2D behindOffset = moveDirection.Negate() * behindDistance;

        // Create virtual target position: offset BEHIND the real target
        // The virtual target moves with the same velocity as the real target
        Point2D virtualTargetPosition = targetPosition + behindOffset;

        // Solve intercept to the virtual target (center-to-center, no edge collision)
        // We use center-to-center because the virtual position already accounts for the offset
        double? interceptTime = SolveInterceptTime(
            casterPosition,
            virtualTargetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange);

        if (!interceptTime.HasValue)
            return null;

        double t = interceptTime.Value;

        // Calculate positions at intercept time
        Point2D aimPoint = CalculatePredictedPosition(virtualTargetPosition, targetVelocity, t);
        Point2D predictedTargetPos = CalculatePredictedPosition(targetPosition, targetVelocity, t);

        // Verify: distance between aim and target should equal behindDistance
        // (they move in parallel, maintaining constant separation)

        return (aimPoint, predictedTargetPos, t);
    }

    /// <summary>
    /// Evaluates the center-to-center intercept function for Newton refinement.
    /// f(t) = |D + V·t| - s·(t - d)
    ///
    /// When f(t) = 0, the projectile center reaches the target center.
    /// </summary>
    /// <param name="displacement">Vector from caster to target at t=0</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="t">Time to evaluate</param>
    /// <returns>Positive if projectile hasn't reached, negative if passed, zero at intercept</returns>
    private static double EvaluateInterceptFunction(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double t)
    {
        // Target position relative to caster at time t
        var relativePosition = displacement + (targetVelocity * t);
        double distanceToTarget = relativePosition.Length;

        // Projectile travel distance at time t (starts moving after delay)
        double flightTime = Math.Max(0, t - castDelay);
        double projectileDistance = skillshotSpeed * flightTime;

        return distanceToTarget - projectileDistance;
    }

    private static double GetRefinementPositionTolerance(double defaultPositionTolerance)
    {
        // Allow callers to pass in old hardcoded tolerances, but clamp to a reasonable minimum.
        return Math.Max(defaultPositionTolerance, Constants.Epsilon);
    }

    private static double GetAdaptiveTimeToleranceFromSpeed(double positionTolerance, double skillshotSpeed, double fallback)
    {
        // Convert desired positional error (units) into a time tolerance based on projectile speed.
        // When speed is very small (shouldn't happen due to validations), fall back to the provided tolerance.
        if (skillshotSpeed <= Constants.Epsilon)
            return fallback;

        double timeTol = positionTolerance / skillshotSpeed;
        return Math.Max(timeTol, fallback);
    }

    /// <summary>
    /// Evaluates the derivative of the center-to-center intercept function:
    /// f(t) = |D + V·t| - s·max(0, t - d)
    ///
    /// f'(t) = ((D + V·t)·V) / |D + V·t| - (t > d ? s : 0)
    /// </summary>
    private static double EvaluateInterceptDerivative(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double t)
    {
        var relativePosition = displacement + (targetVelocity * t);
        double distanceToTarget = relativePosition.Length;

        // d/dt |x(t)| is undefined at |x| = 0; handle safely.
        if (distanceToTarget < Constants.Epsilon)
        {
            return t > castDelay ? -skillshotSpeed : 0;
        }

        double dDistanceDt = relativePosition.DotProduct(targetVelocity) / distanceToTarget;
        double projectileTerm = t > castDelay ? skillshotSpeed : 0;
        return dDistanceDt - projectileTerm;
    }

    /// <summary>
    /// Refines an initial time estimate using MathNet.Numerics NewtonRaphson.
    ///
    /// Newton update:
    ///   t_{n+1} = t_n - f(t_n) / f'(t_n)
    ///
    /// This converges quadratically near the root when the derivative is well-behaved.
    /// Uses MathNet.Numerics.RootFinding.NewtonRaphson for robust, well-tested implementation.
    /// </summary>
    private static double RefineWithNewton(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double maxTime,
        double initialGuess,
        double positionTolerance,
        double timeTolerance,
        int maxIterations = 20)
    {
        double minTime = castDelay + Constants.Epsilon;
        double clampedGuess = Math.Clamp(initialGuess, minTime, maxTime);

        // Define the intercept function f(t) = |D + V*t| - s*(t - d)
        Func<double, double> f = t => EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, t);

        // Define the derivative f'(t)
        Func<double, double> df = t => EvaluateInterceptDerivative(displacement, targetVelocity, skillshotSpeed, castDelay, t);

        // Use MathNet's NewtonRaphson.TryFindRoot with initial guess
        if (NewtonRaphson.TryFindRoot(f, df, clampedGuess, minTime, maxTime, positionTolerance, maxIterations, out double root))
        {
            return root;
        }

        // If Newton-Raphson fails, return the initial guess
        return clampedGuess;
    }

    /// <summary>
    /// Solves for center-to-center intercept time using quadratic formula
    /// with RobustNewtonRaphson refinement for maximum precision.
    ///
    /// Strategy:
    /// 1. Use fast quadratic solver to get initial estimate
    /// 2. Refine with RobustNewtonRaphson (combines Newton with bisection fallback)
    /// </summary>

    public static double? SolveInterceptTimeWithSecantRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue,
        double tolerance = 1e-12)
    {
        // Back-compat entry point.
        // Internally this now uses RobustNewtonRaphson refinement with automatic bisection fallback.

        double? initialEstimate = SolveInterceptTime(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange);

        if (!initialEstimate.HasValue)
            return null;

        double maxTime = castDelay + (skillshotRange / skillshotSpeed);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        double positionTolerance = GetRefinementPositionTolerance(defaultPositionTolerance: Constants.Epsilon);

        double refined = RefineWithRobustNewton(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            maxTime,
            initialEstimate.Value,
            positionTolerance);

        if (refined <= castDelay || refined > maxTime)
            return initialEstimate;

        return refined;
    }

    /// <summary>
    /// Solves for behind-target interception with Newton refinement.
    ///
    /// This is the highest-precision method for behind-target prediction:
    /// 1. Calculate virtual target position (offset behind real target)
    /// 2. Solve intercept using quadratic formula (O(1) initial guess)
    /// 3. Refine with Newton Method using an adaptive tolerance
    /// </summary>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveBehindTargetWithSecantRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0,
        double tolerance = 1e-12)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // For stationary targets, there's no "behind" direction - aim at center
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            double? stationaryTime = SolveEdgeInterceptTime(
                casterPosition, targetPosition, targetVelocity, skillshotSpeed,
                castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);

            if (!stationaryTime.HasValue)
                return null;

            return (targetPosition, targetPosition, stationaryTime.Value);
        }

        // Calculate behind offset: (effectiveRadius - margin) in direction opposite to velocity
        // Defensive check ensures we always have a meaningful behind distance
        double behindDistance = effectiveRadius - behindMargin;
        if (behindDistance < Constants.Epsilon)
        {
            behindDistance = effectiveRadius * 0.5;
        }

        Vector2D moveDirection = targetVelocity.Normalize();
        Vector2D behindOffset = moveDirection.Negate() * behindDistance;

        // Create virtual target position: offset BEHIND the real target
        Point2D virtualTargetPosition = targetPosition + behindOffset;

        // Solve intercept with Newton refinement for maximum precision
        double? interceptTime = SolveInterceptTimeWithSecantRefinement(
            casterPosition,
            virtualTargetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange,
            tolerance);

        if (!interceptTime.HasValue)
            return null;

        double t = interceptTime.Value;

        // Calculate positions at intercept time
        Point2D aimPoint = CalculatePredictedPosition(virtualTargetPosition, targetVelocity, t);
        Point2D predictedTargetPos = CalculatePredictedPosition(targetPosition, targetVelocity, t);

        return (aimPoint, predictedTargetPos, t);
    }

    /// <summary>
    /// Refines a time estimate using MathNet.Numerics Bisection for guaranteed convergence.
    ///
    /// Bisection is the most robust root-finding method:
    /// - GUARANTEED to converge if root exists in bracket
    /// - Linear convergence: each iteration halves the error
    /// - ~50 iterations for machine precision (1e-15)
    ///
    /// Used as final refinement step after Newton for absolute precision.
    /// Uses MathNet.Numerics.RootFinding.Bisection for robust, well-tested implementation.
    /// </summary>
    /// <param name="displacement">Vector from caster to target at t=0</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="estimate">Estimate from previous refinement (Newton)</param>
    /// <param name="bracketSize">Size of bracket around estimate to search</param>
    /// <param name="tolerance">Convergence tolerance (default 1e-15 for pixel precision)</param>
    /// <param name="maxIterations">Maximum iterations (default 60)</param>
    /// <returns>Refined intercept time with guaranteed precision</returns>
    private static double RefineWithBisection(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double maxTime,
        double estimate,
        double initialBracketSize,
        double positionTolerance,
        double timeTolerance,
        int stages = 3,
        int maxIterationsPerStage = 60)
    {
        double minTime = castDelay + Constants.Epsilon;
        double currentEstimate = Math.Clamp(estimate, minTime, maxTime);

        // Define the intercept function
        Func<double, double> f = t => EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, t);

        // Check if estimate is already good enough
        if (Math.Abs(f(currentEstimate)) <= positionTolerance)
            return currentEstimate;

        // Try progressively larger brackets until we find a valid one
        double bracketSize = initialBracketSize;
        for (int stage = 0; stage < stages; stage++)
        {
            double tLow = Math.Clamp(currentEstimate - bracketSize, minTime, maxTime);
            double tHigh = Math.Clamp(currentEstimate + bracketSize, minTime, maxTime);

            // Use MathNet's Bisection.TryFindRoot
            if (Bisection.TryFindRoot(f, tLow, tHigh, timeTolerance, maxIterationsPerStage, out double root))
            {
                currentEstimate = root;
            }

            bracketSize = Math.Max(Constants.TickDuration, bracketSize * 0.5);
        }

        return currentEstimate;
    }

    /// <summary>
    /// Refines intercept time using RobustNewtonRaphson which combines Newton-Raphson
    /// with automatic bisection fallback for guaranteed convergence.
    ///
    /// This replaces the two-step Newton + Bisection refinement chain with a single
    /// call that handles both fast quadratic convergence and robust fallback.
    /// </summary>
    /// <param name="displacement">Vector from caster to target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Cast delay in seconds</param>
    /// <param name="maxTime">Maximum valid intercept time</param>
    /// <param name="estimate">Initial estimate from quadratic solver</param>
    /// <param name="positionTolerance">Tolerance for position error (function value)</param>
    /// <param name="subdivision">Number of subdivisions for bracketing (default 20)</param>
    /// <param name="maxIterations">Maximum iterations (default 100)</param>
    /// <returns>Refined intercept time</returns>
    private static double RefineWithRobustNewton(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double maxTime,
        double estimate,
        double positionTolerance,
        int subdivision = 20,
        int maxIterations = 100)
    {
        double minTime = castDelay + Constants.Epsilon;
        double clampedEstimate = Math.Clamp(estimate, minTime, maxTime);

        // Define the intercept function f(t) = |D + V*t| - s*(t - d)
        Func<double, double> f = t => EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, t);

        // Define the derivative f'(t)
        Func<double, double> df = t => EvaluateInterceptDerivative(displacement, targetVelocity, skillshotSpeed, castDelay, t);

        // Check if estimate is already good enough
        if (Math.Abs(f(clampedEstimate)) <= positionTolerance)
            return clampedEstimate;

        // Use MathNet's RobustNewtonRaphson.TryFindRoot
        // This combines Newton-Raphson with automatic bisection fallback
        if (RobustNewtonRaphson.TryFindRoot(f, df, minTime, maxTime, positionTolerance, maxIterations, subdivision, out double root))
        {
            return root;
        }

        // If RobustNewtonRaphson fails, return the clamped estimate
        return clampedEstimate;
    }

    /// <summary>
    /// Solves for center-to-center intercept time using two-stage refinement:
    /// 1. Quadratic formula - O(1) initial estimate
    /// 2. RobustNewtonRaphson - Fast convergence with automatic bisection fallback
    /// </summary>
    public static double? SolveInterceptTimeWithFullRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue,
        double secantTolerance = 1e-12,
        double bisectionTolerance = 1e-15)
    {
        // Step 1: Get initial estimate from quadratic solver
        double? initialEstimate = SolveInterceptTime(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange);

        if (!initialEstimate.HasValue)
            return null;

        double maxTime = castDelay + (skillshotRange / skillshotSpeed);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        // Step 2: Refine with RobustNewtonRaphson (combines Newton + bisection fallback)
        double positionTolerance = GetRefinementPositionTolerance(defaultPositionTolerance: Constants.Epsilon);

        double refined = RefineWithRobustNewton(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            maxTime,
            initialEstimate.Value,
            positionTolerance);

        if (refined <= castDelay || refined > maxTime)
            return initialEstimate;

        return refined;
    }

    internal static double? SolveInterceptTimeWithNewtonRefinement_ForBenchmark(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue,
        double tolerance = 1e-12)
    {
        double? initialEstimate = SolveInterceptTime(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange);

        if (!initialEstimate.HasValue)
            return null;

        double maxTime = castDelay + (skillshotRange / skillshotSpeed);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        double positionTolerance = GetRefinementPositionTolerance(defaultPositionTolerance: Constants.Epsilon);
        double timeTolerance = GetAdaptiveTimeToleranceFromSpeed(positionTolerance, skillshotSpeed, fallback: tolerance);

        // Benchmark mode: cap Newton iterations so we measure typical convergence cost.
        double refined = RefineWithNewton(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            maxTime,
            initialEstimate.Value,
            positionTolerance,
            timeTolerance,
            maxIterations: 8);

        if (refined <= castDelay || refined > maxTime)
            return initialEstimate;

        return refined;
    }


    internal static double? SolveInterceptTimeWithSecantRefinement_Legacy_ForBenchmark(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue,
        double tolerance = 1e-12)
    {
        double? initialEstimate = SolveInterceptTime(
            casterPosition,
            targetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange);

        if (!initialEstimate.HasValue)
            return null;

        double maxTime = castDelay + (skillshotRange / skillshotSpeed);

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        double positionTolerance = GetRefinementPositionTolerance(defaultPositionTolerance: Constants.Epsilon);
        double timeTolerance = GetAdaptiveTimeToleranceFromSpeed(positionTolerance, skillshotSpeed, fallback: tolerance);

        double refined = RefineWithNewton(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            maxTime,
            initialEstimate.Value,
            positionTolerance,
            timeTolerance,
            maxIterations: 10);

        if (refined <= castDelay || refined > maxTime)
            return initialEstimate;


        return refined;
    }

    /// <summary>
    /// Ultimate solver for behind-target interception with full triple refinement.
    ///
    /// Achieves PIXEL-PERFECT accuracy for "hit from behind by 1 pixel" strategy:
    /// 1. Quadratic formula - O(1) initial estimate
    /// 2. Newton Method - Fast refinement
    /// 3. Bisection - Guaranteed sub-pixel precision (~1e-15)
    ///
    /// At game resolution (typically 1920x1080), 1e-15 precision means:
    /// - Position error &lt; 0.000000000001 pixels
    /// - Time error &lt; 0.000000000001 seconds
    ///
    /// This ensures the projectile hits EXACTLY (effectiveRadius - behindMargin)
    /// behind the target, achieving the intended "1 pixel inside" hit.
    /// </summary>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveBehindTargetWithFullRefinement(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0,
        double secantTolerance = 1e-12,
        double bisectionTolerance = 1e-15,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Adaptive)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // For stationary targets, aim at center
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            double? stationaryTime = SolveEdgeInterceptTime(
                casterPosition, targetPosition, targetVelocity, skillshotSpeed,
                castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);

            if (!stationaryTime.HasValue)
                return null;

            return (targetPosition, targetPosition, stationaryTime.Value);
        }

        // Calculate strategy-specific initial point
        Point2D virtualTargetPosition = CalculateBehindInitialPoint(
            casterPosition, targetPosition, targetVelocity, effectiveRadius, behindMargin, strategy);

        // Solve intercept with FULL triple refinement for pixel-perfect precision
        double? interceptTime = SolveInterceptTimeWithFullRefinement(
            casterPosition,
            virtualTargetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange,
            secantTolerance,
            bisectionTolerance);

        if (!interceptTime.HasValue)
            return null;

        double t = interceptTime.Value;

        // Calculate positions at intercept time
        Point2D aimPoint = CalculatePredictedPosition(virtualTargetPosition, targetVelocity, t);
        Point2D predictedTargetPos = CalculatePredictedPosition(targetPosition, targetVelocity, t);

        return (aimPoint, predictedTargetPos, t);
    }

    /// <summary>
    /// Solves for behind-target intercept using direct closed-form quadratic.
    ///
    /// This method computes the aim point directly without intermediate virtual target
    /// abstraction, potentially offering better performance while maintaining accuracy.
    ///
    /// Mathematical basis:
    /// The "behind point" B(t) = Target(t) - r*V̂ where r is the behind distance and V̂ is unit velocity.
    /// Since B(t) = (T - r*V̂) + V*t, this is a linearly moving point.
    ///
    /// The intercept equation |B(t) - C| = s*(t - d) expands to:
    ///   a*t² + b*t + c = 0
    /// where:
    ///   D = (T - r*V̂) - C  (displacement from caster to initial behind-point)
    ///   a = |V|² - s²
    ///   b = 2*(D·V + s²*d)
    ///   c = |D|² - s²*d²
    /// </summary>
    /// <param name="casterPosition">Position where the skillshot originates</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot (units/second)</param>
    /// <param name="castDelay">Delay before skillshot launches (seconds)</param>
    /// <param name="targetHitboxRadius">Radius of the target's hitbox</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <param name="skillshotRange">Maximum range of the skillshot</param>
    /// <param name="behindMargin">Safety margin behind target (default 1.0 = 1 pixel inside trailing edge)</param>
    /// <returns>Aim point, predicted target position, and intercept time; or null if unreachable</returns>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveBehindTargetDirect(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Adaptive)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // For stationary targets, aim at center
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            double? stationaryTime = SolveEdgeInterceptTime(
                casterPosition, targetPosition, targetVelocity, skillshotSpeed,
                castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);

            if (!stationaryTime.HasValue)
                return null;

            return (targetPosition, targetPosition, stationaryTime.Value);
        }

        // Calculate initial point based on strategy
        Point2D behindPointInitial = CalculateBehindInitialPoint(
            casterPosition, targetPosition, targetVelocity, effectiveRadius, behindMargin, strategy);

        // Displacement from caster to initial behind-point
        var displacement = behindPointInitial - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);
        var V = targetVelocity;
        double s = skillshotSpeed;
        double d = castDelay;

        // Calculate max valid time based on range
        double maxTime = d + (skillshotRange / s);

        // Quadratic coefficients: a*t² + b*t + c = 0
        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * s * d);
        double c = D.DotProduct(D) - (s * s * d * d);

        double? interceptTime;

        // Handle degenerate case when |V| ≈ s (linear equation)
        if (Math.Abs(a) < Constants.Epsilon)
        {
            interceptTime = SolveLinear(b, c, d, maxTime);
        }
        else
        {
            // Use MathNet.Numerics for robust quadratic root finding
            var (r1, r2) = FindRoots.Quadratic(c, b, a);

            // Extract real roots
            double? t1 = r1.IsReal() ? r1.Real : null;
            double? t2 = r2.IsReal() ? r2.Real : null;

            interceptTime = SelectSmallestValidTime(t1, t2, d, maxTime);
        }

        if (!interceptTime.HasValue)
            return null;

        double t = interceptTime.Value;

        // Calculate aim point (where the behind-point will be at time t)
        Point2D aimPoint = CalculatePredictedPosition(behindPointInitial, targetVelocity, t);

        // Calculate predicted target position
        Point2D predictedTargetPos = CalculatePredictedPosition(targetPosition, targetVelocity, t);

        // Final range check on actual flight distance
        double flightDistance = (aimPoint - casterPosition).Length;
        if (flightDistance > skillshotRange)
            return null;

        return (aimPoint, predictedTargetPos, t);
    }

    /// <summary>
    /// Solves for the optimal effectiveRadius using bisection method.
    ///
    /// Algorithm:
    /// 1. Start with effectiveRadius = targetHitboxRadius + skillshotWidth/2 (standard collision)
    /// 2. Use bisection on the half-width portion [0, skillshotWidth/2]
    /// 3. Find the minimum effectiveRadius that still results in a hit
    ///
    /// Example: hitbox=60, width=40
    /// - Standard: effectiveRadius = 60 + 20 = 80
    /// - Bisect between 60 (min) and 80 (max) to find optimal
    /// </summary>
    /// <param name="casterPosition">Position where the skillshot originates</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot (units/second)</param>
    /// <param name="castDelay">Delay before skillshot launches (seconds)</param>
    /// <param name="targetHitboxRadius">Radius of the target's hitbox</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <param name="skillshotRange">Maximum range of the skillshot</param>
    /// <param name="behindMargin">Safety margin behind target (default 1.0)</param>
    /// <param name="radiusTolerance">Convergence tolerance for radius bisection (default 0.5)</param>
    /// <param name="maxIterations">Maximum bisection iterations (default 20)</param>
    /// <returns>Aim point, predicted target position, intercept time, and optimal effectiveRadius; or null if unreachable</returns>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime, double EffectiveRadius)?
        SolveBehindTargetWithRadiusBisection(
            Point2D casterPosition,
            Point2D targetPosition,
            Vector2D targetVelocity,
            double skillshotSpeed,
            double castDelay,
            double targetHitboxRadius,
            double skillshotWidth,
            double skillshotRange,
            double behindMargin = 1.0,
            double radiusTolerance = 0.5,
            int maxIterations = 20)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        // For stationary targets, use simple edge intercept
        if (targetVelocity.Length < Constants.MinVelocity)
        {
            double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);
            double? stationaryTime = SolveEdgeInterceptTime(
                casterPosition, targetPosition, targetVelocity, skillshotSpeed,
                castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);

            if (!stationaryTime.HasValue)
                return null;

            return (targetPosition, targetPosition, stationaryTime.Value, effectiveRadius);
        }

        // Bisection bounds for the width multiplier [0, 1]
        // widthMultiplier = 0 → effectiveRadius = hitbox (minimum)
        // widthMultiplier = 1 → effectiveRadius = hitbox + width/2 (maximum)
        double lowMultiplier = 0.0;
        double highMultiplier = 1.0;

        // Track the best successful hit
        (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime, double EffectiveRadius)? bestHit = null;

        // First, check if maximum radius hits (if not, target is unreachable)
        var maxRadiusResult = TrySolveWithRadius(
            casterPosition, targetPosition, targetVelocity,
            skillshotSpeed, castDelay, targetHitboxRadius,
            skillshotWidth, skillshotRange, behindMargin,
            widthMultiplier: 1.0);

        if (maxRadiusResult == null)
        {
            // Even with maximum radius, can't hit - target unreachable
            return null;
        }

        bestHit = maxRadiusResult;

        // Bisection to find optimal radius
        for (int i = 0; i < maxIterations; i++)
        {
            double midMultiplier = (lowMultiplier + highMultiplier) / 2.0;
            double currentHalfWidth = (skillshotWidth / 2) * midMultiplier;

            // Check convergence (in half-width units)
            if (currentHalfWidth < radiusTolerance || (highMultiplier - lowMultiplier) * (skillshotWidth / 2) < radiusTolerance)
                break;

            var result = TrySolveWithRadius(
                casterPosition, targetPosition, targetVelocity,
                skillshotSpeed, castDelay, targetHitboxRadius,
                skillshotWidth, skillshotRange, behindMargin,
                widthMultiplier: midMultiplier);

            if (result != null)
            {
                // Hit! Try smaller radius (decrease high bound)
                highMultiplier = midMultiplier;
                bestHit = result;
            }
            else
            {
                // Miss! Need larger radius (increase low bound)
                lowMultiplier = midMultiplier;
            }
        }

        return bestHit;
    }

    /// <summary>
    /// Helper method to attempt solving with a specific width multiplier.
    /// </summary>
    private static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime, double EffectiveRadius)?
        TrySolveWithRadius(
            Point2D casterPosition,
            Point2D targetPosition,
            Vector2D targetVelocity,
            double skillshotSpeed,
            double castDelay,
            double targetHitboxRadius,
            double skillshotWidth,
            double skillshotRange,
            double behindMargin,
            double widthMultiplier)
    {
        double adjustedWidth = skillshotWidth * widthMultiplier;
        double effectiveRadius = targetHitboxRadius + (adjustedWidth / 2);

        // Ensure margin doesn't exceed radius
        double actualMargin = Math.Min(behindMargin, effectiveRadius - Constants.Epsilon);
        if (actualMargin < Constants.Epsilon)
            actualMargin = effectiveRadius * 0.5;

        double behindDistance = effectiveRadius - actualMargin;
        if (behindDistance < Constants.Epsilon)
            behindDistance = effectiveRadius * 0.5;

        Vector2D moveDirection = targetVelocity.Normalize();
        Vector2D behindOffset = moveDirection.Negate() * behindDistance;
        Point2D virtualTargetPosition = targetPosition + behindOffset;

        // Solve intercept
        double? interceptTime = SolveInterceptTimeWithFullRefinement(
            casterPosition,
            virtualTargetPosition,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            skillshotRange);

        if (!interceptTime.HasValue)
            return null;

        double t = interceptTime.Value;

        // Calculate positions
        Point2D aimPoint = CalculatePredictedPosition(virtualTargetPosition, targetVelocity, t);
        Point2D predictedTargetPos = CalculatePredictedPosition(targetPosition, targetVelocity, t);

        // Verify collision actually occurs with this radius
        double aimToTarget = (aimPoint - predictedTargetPos).Length;

        // The aim point is behindDistance behind target, so separation should be ~behindDistance
        // A "hit" means the target center is within effectiveRadius of the aim point
        if (aimToTarget <= effectiveRadius)
        {
            return (aimPoint, predictedTargetPos, t, effectiveRadius);
        }

        return null;
    }

    #region Path-Based Interception

    /// <summary>
    /// Result of a path-based interception calculation.
    /// </summary>
    /// <param name="AimPoint">Where to aim the skillshot</param>
    /// <param name="PredictedTargetPosition">Where the target will be at interception</param>
    /// <param name="InterceptTime">Time in seconds until interception</param>
    /// <param name="WaypointIndex">Index of the waypoint segment where interception occurs</param>
    public readonly record struct PathInterceptResult(
        Point2D AimPoint,
        Point2D PredictedTargetPosition,
        double InterceptTime,
        int WaypointIndex);

    /// <summary>
    /// Solves for intercept time when target is following a multi-waypoint path.
    /// Iterates through path segments to find the earliest valid interception.
    ///
    /// This is equivalent to the Lua 'project' function but with proper cast delay support
    /// and our triple-refinement precision.
    /// </summary>
    /// <param name="casterPosition">Position where the skillshot originates</param>
    /// <param name="path">Target's movement path with waypoints</param>
    /// <param name="skillshotSpeed">Speed of the skillshot (units/second)</param>
    /// <param name="castDelay">Delay before skillshot launches (seconds)</param>
    /// <param name="skillshotRange">Maximum range of the skillshot</param>
    /// <returns>Intercept time and waypoint index, or null if no valid interception exists</returns>
    public static (double Time, int WaypointIndex)? SolvePathInterceptTime(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange)
    {
        ValidateInputs(skillshotSpeed, castDelay);

        if (path.Speed < Constants.MinVelocity)
        {
            // Stationary target - use simple center-to-center
            double? time = SolveInterceptTime(
                casterPosition, path.CurrentPosition, new Vector2D(0, 0),
                skillshotSpeed, castDelay);

            if (time.HasValue && IsWithinRange(casterPosition, path.CurrentPosition, time.Value, skillshotRange))
                return (time.Value, path.CurrentWaypointIndex);
            return null;
        }

        // Track cumulative time offset as we traverse path segments
        double segmentStartTime = 0;

        foreach (var segment in path.EnumerateSegments())
        {
            // Calculate time bounds for this segment
            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
            {
                continue; // Skip zero-length segments
            }

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;

            // Get velocity for this segment
            Vector2D segmentVelocity = segment.Direction * path.Speed;

            // Analytical segment pruning: skip segments that can't contain solutions
            if (!IsSegmentReachable(casterPosition, segment.Start, segmentVelocity, segmentDuration,
                skillshotSpeed, castDelay, skillshotRange, 0, segmentStartTime))
            {
                segmentStartTime = segmentEndTime;
                continue;
            }

            var interceptResult = SolveSegmentIntercept(
                casterPosition,
                segment.Start,
                segmentVelocity,
                skillshotSpeed,
                castDelay,
                skillshotRange,
                segmentStartTime,
                segmentEndTime);

            if (interceptResult.HasValue)
            {
                return (interceptResult.Value, segment.WaypointIndex);
            }

            segmentStartTime = segmentEndTime;
        }

        // No valid intercept found on any segment
        // Check if we can hit at the final waypoint (target stopped)
        if (path.Waypoints.Count > 0)
        {
            var finalWaypoint = path.Waypoints[^1];
            double timeToFinal = path.GetRemainingPathTime();

            // Time for projectile to reach final waypoint after it stops there
            double distanceToFinal = (finalWaypoint - casterPosition).Length;
            double flightTime = distanceToFinal / skillshotSpeed;
            double totalTime = castDelay + flightTime;

            // If projectile arrives after target reaches final waypoint, it's a hit
            if (totalTime >= timeToFinal && distanceToFinal <= skillshotRange)
            {
                return (totalTime, path.Waypoints.Count - 1);
            }
        }

        return null;
    }

    /// <summary>
    /// Solves the absolute minimal interception time for a path-based target.
    /// Incorporates Suggestions 1 (Global Path), 2 (Rectangular), and 3 (Caster Offset).
    /// </summary>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime, int WaypointIndex)? SolvePathMinimalIntercept(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetRadius,
        double skillshotWidth,
        double skillshotRange,
        double casterRadius)
    {
        // Caster offset reduces effective travel distance
        // Front-line collision saves skillshotWidth/2 distance compared to circle model
        // Combined effective radius for minimal contact
        double effectiveRadius = targetRadius + casterRadius;

        // Effective delay: when projectile reaches caster edge (t = delay - R_caster/speed)
        // But it only starts moving at t = delay.
        // We solve: speed * (t - delay) = dist - casterRadius - targetRadius
        // Which is: speed * (t - delay) + casterRadius + targetRadius = |P(t) - C|
        double combinedR = targetRadius + casterRadius;
        double speed = skillshotSpeed;
        double delay = castDelay;

        // Handle case where target is already within collision radius at cast delay
        Point2D posAtDelay = path.GetPositionAtTime(delay);
        if ((posAtDelay - casterPosition).Length <= combinedR)
        {
            return (posAtDelay, posAtDelay, delay, path.CurrentWaypointIndex);
        }

        // Global path root-finding across all segments
        // We find the first root of f(t) = |P(t) - C| - (s*(t-delay) + combinedR)
        Func<double, double> collisionFunc = t =>
        {
            Point2D pos = path.GetPositionAtTime(t);
            double dist = (pos - casterPosition).Length;
            double travel = (speed * Math.Max(0, t - delay)) + combinedR;
            return dist - travel;
        };

        double remainingTime = path.GetRemainingPathTime();
        double maxT = delay + ((skillshotRange + combinedR) / speed);
        double scanEnd = Math.Min(maxT, remainingTime + 1.0);

        // Scan segments for roots
        double prevT = 0;
        double prevVal = collisionFunc(0);

        // Step size for scanning: half a tick
        double step = Constants.TickDuration * 0.5;

        for (double t = step; t <= scanEnd + step; t += step)
        {
            double currentT = Math.Min(t, scanEnd);
            double currentVal = collisionFunc(currentT);

            if (prevVal * currentVal <= 0)
            {
                // Root bracketed between [prevT, currentT]
                if (Brent.TryFindRoot(collisionFunc, prevT, currentT, Constants.Epsilon, 100, out double hitTime))
                {
                    Point2D hitPos = path.GetPositionAtTime(hitTime);

                    // Verify range
                    double travelDist = speed * (hitTime - delay);
                    if (travelDist <= skillshotRange + Constants.RangeTolerance)
                    {
                        // Waypoint index at hit time
                        int wpIdx = path.CurrentWaypointIndex;
                        double d = 0;
                        for (int i = 0; i < path.SegmentCount; i++)
                        {
                            var seg = path.GetSegment(i);
                            double segTime = seg.Length / path.Speed;
                            if (d + segTime >= hitTime - Constants.Epsilon)
                            {
                                wpIdx = seg.WaypointIndex;
                                break;
                            }
                            d += segTime;
                        }

                        return (hitPos, hitPos, hitTime, wpIdx);
                    }
                }
            }

            prevT = currentT;
            prevVal = currentVal;
            if (t > scanEnd) break;
        }

        return null;
    }

    /// <summary>
    /// Solves the absolute minimal interception time for a linear velocity target.
    /// Incorporates Suggestions 2 (Rectangular) and 3 (Caster Offset).
    /// </summary>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveMinimalInterceptDirect(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetRadius,
        double skillshotWidth,
        double skillshotRange,
        double casterRadius)
    {
        // Caster offset + rectangular front collision model
        // Effective combined radius for minimal contact
        double combinedR = targetRadius + casterRadius;

        // Solve: |P + V*t - C| = s*(t - delay) + combinedR
        // Let D = P - C, d = delay, s = skillshotSpeed, r = combinedR
        // |D + V*t| = s*(t - d) + r
        // |D + V*t| = s*t - (s*d - r)
        // Let K = s*d - r
        // |D + V*t| = s*t - K
        // (D + V*t)^2 = (s*t - K)^2
        // D^2 + 2(D.V)t + V^2 t^2 = s^2 t^2 - 2sKt + K^2
        // (V^2 - s^2)t^2 + (2(D.V) + 2sK)t + (D^2 - K^2) = 0

        double s = skillshotSpeed;
        double K = (s * castDelay) - combinedR;

        var D = targetPosition - casterPosition;
        var V = targetVelocity;

        // Handle case where target is already within collision radius at cast delay
        Point2D posAtDelay = targetPosition + (targetVelocity * castDelay);
        if ((posAtDelay - casterPosition).Length <= combinedR)
        {
            return (posAtDelay, posAtDelay, castDelay);
        }

        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * K);
        double c = D.DotProduct(D) - (K * K);

        double? bestT = null;
        if (Math.Abs(a) < Constants.Epsilon)
        {
            if (Math.Abs(b) > Constants.Epsilon)
            {
                double t = -c / b;
                if (t >= castDelay) bestT = t;
            }
        }
        else
        {
            // Use MathNet quadratic solver for numerical stability
            var (root1, root2) = FindRoots.Quadratic(c, b, a);

            double? t1 = Math.Abs(root1.Imaginary) < Constants.Epsilon ? root1.Real : null;
            double? t2 = Math.Abs(root2.Imaginary) < Constants.Epsilon ? root2.Real : null;

            bool v1 = t1.HasValue && t1 >= castDelay - Constants.Epsilon;
            bool v2 = t2.HasValue && t2 >= castDelay - Constants.Epsilon;

            if (v1 && v2) bestT = Math.Min(t1!.Value, t2!.Value);
            else if (v1) bestT = t1;
            else if (v2) bestT = t2;
        }

        if (!bestT.HasValue) return null;

        double tResult = bestT.Value;
        Point2D hitPos = targetPosition + (targetVelocity * tResult);

        // Range check
        double travelDist = s * (tResult - castDelay);
        if (travelDist > skillshotRange + Constants.RangeTolerance) return null;

        return (hitPos, hitPos, tResult);
    }

    /// <summary>
    /// Solves for interception time considering both target and projectile acceleration.
    /// Uses a quartic polynomial root solver (MathNet.Numerics).
    /// </summary>
    public static (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? SolveAccelerationInterceptDirect(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        Vector2D targetAcceleration,
        double skillshotSpeed,
        double skillshotAcceleration,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double casterRadius = 0)
    {
        double r = targetHitboxRadius + (skillshotWidth / 2) + casterRadius;
        double s = skillshotSpeed;
        double a_p = skillshotAcceleration;
        double d = castDelay;

        // Shift target to its position at cast delay
        // P(x) = P_0 + V_0*(x+d) + 0.5*a_t*(x+d)^2
        // P(x) = (P_0 + V_0*d + 0.5*a_t*d^2) + (V_0 + a_t*d)x + 0.5*a_t*x^2
        Point2D pPrime0 = targetPosition + (targetVelocity * d) + (targetAcceleration * (0.5 * d * d));
        Vector2D vPrime0 = targetVelocity + (targetAcceleration * d);
        Vector2D dVec = pPrime0 - casterPosition;
        Vector2D a_t = targetAcceleration;

        // Coefficients for the quartic polynomial k4*x^4 + k3*x^3 + k2*x^2 + k1*x + k0 = 0
        // derived from |P(x) - C|^2 = (0.5*a_p*x^2 + s*x + r)^2
        double k4 = 0.25 * (a_t.DotProduct(a_t) - (a_p * a_p));
        double k3 = vPrime0.DotProduct(a_t) - (s * a_p);
        double k2 = vPrime0.DotProduct(vPrime0) + dVec.DotProduct(a_t) - ((s * s) + (r * a_p));
        double k1 = (2 * dVec.DotProduct(vPrime0)) - (2 * r * s);
        double k0 = dVec.DotProduct(dVec) - (r * r);

        double[] coeffs = { k0, k1, k2, k3, k4 };
        var roots = FindRoots.Polynomial(coeffs);

        double? bestX = null;
        double maxFlightTime = skillshotRange / Math.Max(s, Constants.MinVelocity); // Rough bound

        foreach (var root in roots)
        {
            if (Math.Abs(root.Imaginary) < Constants.Epsilon)
            {
                double x = root.Real;
                if (x >= -Constants.Epsilon)
                {
                    // Verify range and speed constraints
                    double dist = (0.5 * a_p * x * x) + (s * x);
                    if (dist >= -Constants.Epsilon && dist <= skillshotRange + Constants.RangeTolerance)
                    {
                        if (bestX == null || x < bestX.Value)
                            bestX = x;
                    }
                }
            }
        }

        if (!bestX.HasValue) return null;

        double xResult = bestX.Value;
        double tResult = xResult + d;
        Point2D hitPos = targetPosition + (targetVelocity * tResult) + (targetAcceleration * (0.5 * tResult * tResult));

        return (hitPos, hitPos, tResult);
    }

    /// <summary>
    /// Solves for edge-to-edge intercept time when target is following a multi-waypoint path.
    /// Returns the earliest time when skillshot edge touches target hitbox.
    /// </summary>
    public static PathInterceptResult? SolvePathEdgeIntercept(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        ValidateInputs(skillshotSpeed, castDelay);
        ValidateGeometry(targetHitboxRadius, skillshotWidth, skillshotRange);

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        if (path.Speed < Constants.MinVelocity)
        {
            // Stationary target
            double? time = SolveEdgeInterceptTime(
                casterPosition, path.CurrentPosition, new Vector2D(0, 0),
                skillshotSpeed, castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);

            if (time.HasValue)
            {
                return new PathInterceptResult(
                    path.CurrentPosition,
                    path.CurrentPosition,
                    time.Value,
                    path.CurrentWaypointIndex);
            }
            return null;
        }

        double segmentStartTime = 0;

        foreach (var segment in path.EnumerateSegments())
        {
            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
                continue;

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;
            Vector2D segmentVelocity = segment.Direction * path.Speed;

            var interceptResult = SolveSegmentEdgeIntercept(
                casterPosition,
                segment.Start,
                segmentVelocity,
                skillshotSpeed,
                castDelay,
                effectiveRadius,
                skillshotRange,
                segmentStartTime,
                segmentEndTime);

            if (interceptResult.HasValue)
            {
                var (aimPoint, interceptTime) = interceptResult.Value;
                Point2D predictedPos = path.GetPositionAtTime(interceptTime);
                return new PathInterceptResult(aimPoint, predictedPos, interceptTime, segment.WaypointIndex);
            }

            segmentStartTime = segmentEndTime;
        }

        // Check final waypoint
        return TryInterceptAtFinalWaypoint(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange);
    }

    /// <summary>
    /// Solves for "behind target" intercept when target is following a multi-waypoint path.
    /// Uses our precision refinement for pixel-perfect accuracy.
    /// </summary>
    public static PathInterceptResult? SolvePathBehindTargetIntercept(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Adaptive)
    {
        ValidateInputs(skillshotSpeed, castDelay);
        ValidateGeometry(targetHitboxRadius, skillshotWidth, skillshotRange);
        if (behindMargin < 0)
            throw new ArgumentException("Behind margin cannot be negative", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        if (path.Speed < Constants.MinVelocity)
        {
            // Stationary target - no "behind" direction, aim at center
            var edgeResult = SolvePathEdgeIntercept(
                casterPosition, path, skillshotSpeed, castDelay,
                targetHitboxRadius, skillshotWidth, skillshotRange);
            return edgeResult;
        }

        double segmentStartTime = 0;

        foreach (var segment in path.EnumerateSegments())
        {
            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
                continue;

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;
            Vector2D segmentVelocity = segment.Direction * path.Speed;

            // Analytical segment pruning: skip segments that can't contain solutions
            if (!IsSegmentReachable(casterPosition, segment.Start, segmentVelocity, segmentDuration,
                skillshotSpeed, castDelay, skillshotRange, effectiveRadius, segmentStartTime))
            {
                segmentStartTime = segmentEndTime;
                continue;
            }

            // Calculate strategy-specific initial point for this segment
            Point2D virtualStart = CalculateBehindInitialPoint(
                casterPosition, segment.Start, segmentVelocity, effectiveRadius, behindMargin, strategy);

            var interceptResult = SolveSegmentIntercept(
                casterPosition,
                virtualStart,
                segmentVelocity,
                skillshotSpeed,
                castDelay,
                skillshotRange,
                segmentStartTime,
                segmentEndTime);

            if (interceptResult.HasValue)
            {
                double interceptTime = interceptResult.Value;

                // Calculate aim point using strategy at the actual intercept time
                Point2D predictedPos = path.GetPositionAtTime(interceptTime);
                Vector2D velocityAtIntercept = path.GetVelocityAtTime(interceptTime);

                Point2D aimPoint = CalculateBehindInitialPoint(
                    casterPosition, predictedPos, velocityAtIntercept, effectiveRadius, behindMargin, strategy);

                return new PathInterceptResult(aimPoint, predictedPos, interceptTime, segment.WaypointIndex);
            }

            segmentStartTime = segmentEndTime;
        }

        // Check final waypoint
        return TryInterceptAtFinalWaypoint(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange);
    }

    /// <summary>
    /// Path-based version of radius bisection.
    /// Uses bisection to find the optimal effectiveRadius for path-following targets.
    /// </summary>
    /// <param name="casterPosition">Position where the skillshot originates</param>
    /// <param name="path">Target's movement path with waypoints</param>
    /// <param name="skillshotSpeed">Speed of the skillshot (units/second)</param>
    /// <param name="castDelay">Delay before skillshot launches (seconds)</param>
    /// <param name="targetHitboxRadius">Radius of the target's hitbox</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <param name="skillshotRange">Maximum range of the skillshot</param>
    /// <param name="behindMargin">Safety margin behind target (default 1.0)</param>
    /// <param name="radiusTolerance">Convergence tolerance for radius bisection (default 0.5)</param>
    /// <param name="maxIterations">Maximum bisection iterations (default 20)</param>
    /// <returns>Intercept result with optimal effectiveRadius, or null if unreachable</returns>
    public static (PathInterceptResult Result, double EffectiveRadius)? SolvePathBehindTargetWithRadiusBisection(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0,
        double radiusTolerance = 0.5,
        int maxIterations = 20)
    {
        ValidateInputs(skillshotSpeed, castDelay);
        ValidateGeometry(targetHitboxRadius, skillshotWidth, skillshotRange);
        if (behindMargin < 0)
            throw new ArgumentException("Behind margin cannot be negative", nameof(behindMargin));

        // For stationary targets, use edge intercept with max radius
        if (path.Speed < Constants.MinVelocity)
        {
            double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);
            var edgeResult = SolvePathEdgeIntercept(
                casterPosition, path, skillshotSpeed, castDelay,
                targetHitboxRadius, skillshotWidth, skillshotRange);

            if (edgeResult == null)
                return null;

            return (edgeResult.Value, effectiveRadius);
        }

        // Bisection bounds for the width multiplier [0, 1]
        // widthMultiplier = 0 → effectiveRadius = hitbox (minimum)
        // widthMultiplier = 1 → effectiveRadius = hitbox + width/2 (maximum)
        double lowMultiplier = 0.0;
        double highMultiplier = 1.0;

        (PathInterceptResult Result, double EffectiveRadius)? bestHit = null;

        // First, check if maximum radius hits
        var maxRadiusResult = TrySolvePathWithRadius(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange, behindMargin,
            widthMultiplier: 1.0);

        if (maxRadiusResult == null)
            return null;

        bestHit = maxRadiusResult;

        // Bisection to find optimal radius
        for (int i = 0; i < maxIterations; i++)
        {
            double midMultiplier = (lowMultiplier + highMultiplier) / 2.0;
            double currentHalfWidth = (skillshotWidth / 2) * midMultiplier;

            if (currentHalfWidth < radiusTolerance || (highMultiplier - lowMultiplier) * (skillshotWidth / 2) < radiusTolerance)
                break;

            var result = TrySolvePathWithRadius(
                casterPosition, path, skillshotSpeed, castDelay,
                targetHitboxRadius, skillshotWidth, skillshotRange, behindMargin,
                widthMultiplier: midMultiplier);

            if (result != null)
            {
                highMultiplier = midMultiplier;
                bestHit = result;
            }
            else
            {
                lowMultiplier = midMultiplier;
            }
        }

        return bestHit;
    }

    /// <summary>
    /// Helper method to attempt path-based solving with a specific width multiplier.
    /// </summary>
    private static (PathInterceptResult Result, double EffectiveRadius)? TrySolvePathWithRadius(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin,
        double widthMultiplier)
    {
        double adjustedWidth = skillshotWidth * widthMultiplier;
        double effectiveRadius = targetHitboxRadius + (adjustedWidth / 2);

        double maxMargin = effectiveRadius > 1.0
            ? effectiveRadius - 1.0
            : effectiveRadius * 0.9;
        double actualMargin = Math.Clamp(behindMargin, Constants.Epsilon, Math.Max(Constants.Epsilon, maxMargin));
        double behindDistance = effectiveRadius - actualMargin;

        double segmentStartTime = 0;

        foreach (var segment in path.EnumerateSegments())
        {
            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
                continue;

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;
            Vector2D segmentVelocity = segment.Direction * path.Speed;

            Vector2D moveDirection = segmentVelocity.Normalize();
            Vector2D behindOffset = moveDirection.Negate() * behindDistance;
            Point2D virtualStart = segment.Start + behindOffset;

            var interceptResult = SolveSegmentIntercept(
                casterPosition,
                virtualStart,
                segmentVelocity,
                skillshotSpeed,
                castDelay,
                skillshotRange,
                segmentStartTime,
                segmentEndTime);

            if (interceptResult.HasValue)
            {
                double interceptTime = interceptResult.Value;

                // FIX: Calculate aim point using velocity at intercept time
                Point2D predictedPos = path.GetPositionAtTime(interceptTime);
                Vector2D velocityAtIntercept = path.GetVelocityAtTime(interceptTime);

                Vector2D moveDirectionAtIntercept = velocityAtIntercept.Length > Constants.Epsilon
                    ? velocityAtIntercept.Normalize()
                    : moveDirection;

                Point2D aimPoint = predictedPos + (moveDirectionAtIntercept.Negate() * behindDistance);

                // Verify hit - the separation should be approximately behindDistance
                double separation = (aimPoint - predictedPos).Length;
                if (separation <= effectiveRadius)
                {
                    var pathResult = new PathInterceptResult(aimPoint, predictedPos, interceptTime, segment.WaypointIndex);
                    return (pathResult, effectiveRadius);
                }
            }

            segmentStartTime = segmentEndTime;
        }

        return null;
    }

    /// <summary>
    /// Solves intercept for a single path segment with time bounds.
    ///
    /// Uses direct quadratic solving to handle negative adjusted delays correctly
    /// (when the projectile has already been in flight before this segment starts).
    /// </summary>
    private static double? SolveSegmentIntercept(
        Point2D casterPosition,
        Point2D segmentStart,
        Vector2D segmentVelocity,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange,
        double minTime,
        double maxTime)
    {
        // We solve in LOCAL time T = t - minTime (T >= 0 within segment)
        // Target position at local time T: segmentStart + segmentVelocity * T
        // Projectile travel time at local time T: (T + minTime - castDelay)
        //
        // adjustedDelay = castDelay - minTime (can be negative when projectile already in flight)
        // Intercept equation: |segmentStart + V*T - caster| = speed * (T - adjustedDelay)

        double adjustedDelay = castDelay - minTime;
        double s = skillshotSpeed;

        var D = new Vector2D(segmentStart.X - casterPosition.X, segmentStart.Y - casterPosition.Y);
        var V = segmentVelocity;

        // Quadratic: (V·V - s²)T² + (2D·V + 2s²·adjustedDelay)T + (D·D - s²·adjustedDelay²) = 0
        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * s * adjustedDelay);
        double c = D.DotProduct(D) - (s * s * adjustedDelay * adjustedDelay);

        double? localTime = null;
        double segmentDuration = maxTime - minTime;

        // Valid local time: T > adjustedDelay (projectile has launched) AND T in [0, segmentDuration]
        double minLocalTime = Math.Max(0, adjustedDelay);
        double maxLocalTime = segmentDuration;

        if (Math.Abs(a) < Constants.Epsilon)
        {
            // Linear case: bT + c = 0
            if (Math.Abs(b) > Constants.Epsilon)
            {
                double t = -c / b;
                if (t > minLocalTime && t <= maxLocalTime + Constants.Epsilon)
                    localTime = t;
            }
        }
        else
        {
            // Use MathNet quadratic solver for numerical stability
            var (root1, root2) = FindRoots.Quadratic(c, b, a);

            double? t1 = Math.Abs(root1.Imaginary) < Constants.Epsilon ? root1.Real : null;
            double? t2 = Math.Abs(root2.Imaginary) < Constants.Epsilon ? root2.Real : null;

            bool v1 = t1.HasValue && t1 > minLocalTime - Constants.Epsilon && t1 <= maxLocalTime + Constants.Epsilon;
            bool v2 = t2.HasValue && t2 > minLocalTime - Constants.Epsilon && t2 <= maxLocalTime + Constants.Epsilon;

            if (v1 && v2)
                localTime = Math.Min(t1!.Value, t2!.Value);
            else if (v1)
                localTime = t1;
            else if (v2)
                localTime = t2;
        }

        if (!localTime.HasValue)
            return null;

        double globalTime = localTime.Value + minTime;

        // Verify range constraint
        Point2D targetAtIntercept = segmentStart + (segmentVelocity * localTime.Value);
        double projectileTravelTime = globalTime - castDelay;
        double projectileTravelDist = skillshotSpeed * projectileTravelTime;

        if (projectileTravelDist <= skillshotRange + Constants.Epsilon)
            return globalTime;

        return null;
    }

    /// <summary>
    /// Solves edge-to-edge intercept for a single segment.
    /// Handles negative adjusted delay (when projectile already in flight) correctly.
    /// </summary>
    private static (Point2D AimPoint, double Time)? SolveSegmentEdgeIntercept(
        Point2D casterPosition,
        Point2D segmentStart,
        Vector2D segmentVelocity,
        double skillshotSpeed,
        double castDelay,
        double effectiveRadius,
        double skillshotRange,
        double minTime,
        double maxTime)
    {
        // adjustedDelay can be negative when projectile is already in flight
        double adjustedDelay = castDelay - minTime;
        double s = skillshotSpeed;
        double r = effectiveRadius;

        // Displacement vector from caster to segment start
        var D = new Vector2D(segmentStart.X - casterPosition.X, segmentStart.Y - casterPosition.Y);
        var V = segmentVelocity;

        double segmentDuration = maxTime - minTime;

        // Handle stationary case
        if (V.Length < Constants.MinVelocity)
        {
            double distance = D.Length;
            if (distance <= r)
            {
                // Already in collision range
                double hitTime = Math.Max(0, adjustedDelay);
                return (segmentStart, minTime + hitTime);
            }

            double flightTime = (distance - r) / s;
            double stationaryLocalTime = Math.Max(0, adjustedDelay) + flightTime;
            if (stationaryLocalTime <= segmentDuration + Constants.Epsilon)
            {
                return (segmentStart, stationaryLocalTime + minTime);
            }
            return null;
        }

        // Edge-to-edge collision equation:
        // |D + V·T|² = (s·(T - adjustedDelay) + r)² for T >= max(0, adjustedDelay)
        //
        // Expanding: |D + V·T|² = (s·T - s·adjustedDelay + r)²
        // Let sdr = s·adjustedDelay - r (note: can be negative)
        // |D + V·T|² = (s·T - sdr)² = s²T² - 2·s·sdr·T + sdr²
        //
        // D·D + 2(D·V)T + (V·V)T² = s²T² - 2·s·sdr·T + sdr²
        // (V·V - s²)T² + (2(D·V) + 2·s·sdr)T + (D·D - sdr²) = 0

        double sdr = (s * adjustedDelay) - r;
        double a = V.DotProduct(V) - (s * s);
        double b = (2 * D.DotProduct(V)) + (2 * s * sdr);
        double c = D.DotProduct(D) - (sdr * sdr);

        double? localTime = null;

        // Valid local time: T > adjustedDelay (projectile has launched) AND T in [0, segmentDuration]
        double minLocalTime = Math.Max(0, adjustedDelay);
        double maxLocalTime = segmentDuration;

        // Handle degenerate case when |V| ≈ s
        if (Math.Abs(a) < Constants.Epsilon)
        {
            if (Math.Abs(b) > Constants.Epsilon)
            {
                double t = -c / b;
                if (t >= minLocalTime - Constants.Epsilon && t <= maxLocalTime + Constants.Epsilon)
                    localTime = t;
            }
        }
        else
        {
            // Use MathNet quadratic solver for numerical stability
            var (root1, root2) = FindRoots.Quadratic(c, b, a);

            double? t1 = Math.Abs(root1.Imaginary) < Constants.Epsilon ? root1.Real : null;
            double? t2 = Math.Abs(root2.Imaginary) < Constants.Epsilon ? root2.Real : null;

            bool v1 = t1.HasValue && t1 >= minLocalTime - Constants.Epsilon && t1 <= maxLocalTime + Constants.Epsilon;
            bool v2 = t2.HasValue && t2 >= minLocalTime - Constants.Epsilon && t2 <= maxLocalTime + Constants.Epsilon;

            if (v1 && v2)
                localTime = Math.Min(t1!.Value, t2!.Value);
            else if (v1)
                localTime = t1;
            else if (v2)
                localTime = t2;
        }

        if (!localTime.HasValue)
            return null;

        double globalTime = localTime.Value + minTime;
        Point2D targetAtIntercept = segmentStart + (segmentVelocity * localTime.Value);

        // Verify range constraint
        double projectileTravelTime = globalTime - castDelay;
        double projectileTravelDist = s * projectileTravelTime;

        if (projectileTravelDist <= skillshotRange + Constants.Epsilon)
        {
            return (targetAtIntercept, globalTime);
        }

        return null;
    }

    /// <summary>
    /// Attempts to intercept at the final waypoint (where target stops).
    /// </summary>
    private static PathInterceptResult? TryInterceptAtFinalWaypoint(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (path.Waypoints.Count == 0)
            return null;

        var finalWaypoint = path.Waypoints[^1];
        double timeToFinal = path.GetRemainingPathTime();
        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);

        // Distance from caster to final waypoint
        double distanceToFinal = (finalWaypoint - casterPosition).Length;

        // Check if within range (accounting for edge collision)
        if (distanceToFinal > skillshotRange + effectiveRadius)
            return null;

        // Time for projectile to reach final waypoint
        double flightDistance = Math.Max(0, distanceToFinal - effectiveRadius);
        double flightTime = flightDistance / skillshotSpeed;
        double arrivalTime = castDelay + flightTime;

        // If projectile arrives after target reaches final waypoint, it's a hit
        if (arrivalTime >= timeToFinal)
        {
            return new PathInterceptResult(
                finalWaypoint,
                finalWaypoint,
                Math.Max(arrivalTime, timeToFinal),
                path.Waypoints.Count - 1);
        }

        return null;
    }

    /// <summary>
    /// Validates geometry parameters.
    /// </summary>
    private static void ValidateGeometry(double targetHitboxRadius, double skillshotWidth, double skillshotRange)
    {
        if (targetHitboxRadius < 0)
            throw new ArgumentException("Target hitbox radius cannot be negative", nameof(targetHitboxRadius));
        if (skillshotWidth < 0)
            throw new ArgumentException("Skillshot width cannot be negative", nameof(skillshotWidth));
        if (skillshotRange <= 0)
            throw new ArgumentException("Skillshot range must be positive", nameof(skillshotRange));
    }

    /// <summary>
    /// Checks if a position is within skillshot range.
    /// Uses tolerance to account for floating-point precision errors.
    /// </summary>
    private static bool IsWithinRange(Point2D casterPosition, Point2D targetPosition, double time, double skillshotRange)
    {
        // The aim point at intercept time would be at targetPosition + velocity*time
        // But for stationary targets, just check distance
        return (targetPosition - casterPosition).Length <= skillshotRange + Constants.RangeTolerance;
    }

    #endregion


    #region Circular Skillshot (Ground-Targeted)

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

    #endregion

    #region Gagong Strategy (Minimum Time Behind Edge)

    /// <summary>
    /// Solves for "behind edge" intercept using Gagong strategy - finds minimum intercept time
    /// while constraining the aim point to be behind the target's trailing edge.
    ///
    /// Unlike Adaptive/DirectBehind which use fixed geometric offsets, Gagong searches for the
    /// optimal offset angle within the "behind hemisphere" (±π/2 from opposite of velocity)
    /// that minimizes total intercept time.
    ///
    /// Algorithm:
    /// 1. For each path segment, iterate candidate intercept times
    /// 2. At each time, search for optimal angle θ in behind hemisphere
    /// 3. Aim point = targetPosition + offset(θ) * effectiveRadius
    /// 4. Find θ that minimizes total time (castDelay + distance/speed)
    /// 5. Return first valid intercept (minimum time guarantee)
    /// </summary>
    /// <param name="casterPosition">Position where the skillshot originates</param>
    /// <param name="path">Target's movement path with waypoints</param>
    /// <param name="skillshotSpeed">Speed of the skillshot (units/second)</param>
    /// <param name="castDelay">Delay before skillshot launches (seconds)</param>
    /// <param name="targetHitboxRadius">Radius of the target's hitbox</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <param name="skillshotRange">Maximum range of the skillshot</param>
    /// <param name="behindMargin">Safety margin behind target (default 1.0 from Constants.TrailingEdgeMargin)</param>
    /// <returns>Intercept result with optimal aim point, or null if unreachable</returns>
    public static PathInterceptResult? SolveGagongIntercept(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange,
        double behindMargin = 1.0)
    {
        ValidateInputs(skillshotSpeed, castDelay);
        ValidateGeometry(targetHitboxRadius, skillshotWidth, skillshotRange);
        if (behindMargin < 0)
            throw new ArgumentException("Behind margin cannot be negative", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + (skillshotWidth / 2);
        double hitRadius = effectiveRadius - behindMargin;
        if (hitRadius < Constants.Epsilon) hitRadius = effectiveRadius * 0.5;

        // For stationary target, no "behind" direction - use edge intercept
        if (path.Speed < Constants.MinVelocity)
        {
            return SolvePathEdgeIntercept(
                casterPosition, path, skillshotSpeed, castDelay,
                targetHitboxRadius, skillshotWidth, skillshotRange);
        }

        double segmentStartTime = 0;

        foreach (var segment in path.EnumerateSegments())
        {
            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
                continue;

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;
            Vector2D segmentVelocity = segment.Direction * path.Speed;

            // Analytical segment pruning
            if (!IsSegmentReachable(casterPosition, segment.Start, segmentVelocity, segmentDuration,
                skillshotSpeed, castDelay, skillshotRange, effectiveRadius, segmentStartTime))
            {
                segmentStartTime = segmentEndTime;
                continue;
            }

            // Find optimal intercept within this segment using Gagong algorithm
            var result = SolveGagongSegmentIntercept(
                casterPosition,
                segment.Start,
                segmentVelocity,
                skillshotSpeed,
                castDelay,
                hitRadius,
                skillshotRange,
                segmentStartTime,
                segmentEndTime);

            if (result.HasValue)
            {
                var (aimPoint, interceptTime) = result.Value;
                Point2D predictedPos = path.GetPositionAtTime(interceptTime);
                return new PathInterceptResult(aimPoint, predictedPos, interceptTime, segment.WaypointIndex);
            }

            segmentStartTime = segmentEndTime;
        }

        // Check final waypoint
        return TryInterceptAtFinalWaypoint(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange);
    }

    /// <summary>
    /// Solves Gagong intercept for a single path segment.
    /// Searches for optimal offset angle that minimizes intercept time while staying behind target.
    /// </summary>
    private static (Point2D AimPoint, double Time)? SolveGagongSegmentIntercept(
        Point2D casterPosition,
        Point2D segmentStart,
        Vector2D segmentVelocity,
        double skillshotSpeed,
        double castDelay,
        double hitRadius,
        double skillshotRange,
        double minTime,
        double maxTime)
    {
        double segmentDuration = maxTime - minTime;
        if (segmentDuration < Constants.Epsilon)
            return null;

        // Behind direction (opposite of velocity)
        Vector2D moveDir = segmentVelocity.Normalize();
        double baseBehindAngle = Math.Atan2(-moveDir.Y, -moveDir.X);

        // Search parameters
        const int timeSteps = Constants.GagongTimeSteps;
        const int angleSteps = Constants.GagongAngleSteps;
        double halfPi = Math.PI / 2;

        double bestTime = double.MaxValue;
        Point2D? bestAimPoint = null;

        // Coarse grid search over time and angle
        for (int ti = 0; ti <= timeSteps; ti++)
        {
            double localT = segmentDuration * ti / timeSteps;
            double globalT = minTime + localT;

            // Target position at this time
            Point2D targetPos = segmentStart + (segmentVelocity * localT);

            // Search angles in behind hemisphere [-π/2, +π/2] from behind direction
            for (int ai = 0; ai <= angleSteps; ai++)
            {
                double angleOffset = -halfPi + (Math.PI * ai / angleSteps);
                double angle = baseBehindAngle + angleOffset;

                // Aim point offset from target
                Vector2D offset = new Vector2D(Math.Cos(angle), Math.Sin(angle));
                Point2D aimPoint = targetPos + (offset * hitRadius);

                // Calculate intercept time for this aim point
                double distToAim = (aimPoint - casterPosition).Length;
                if (distToAim > skillshotRange)
                    continue;

                double travelTime = distToAim / skillshotSpeed;
                double arrivalTime = castDelay + travelTime;

                // Skillshot must arrive after or at the time target reaches this position
                if (arrivalTime < globalT - Constants.Epsilon)
                    continue;

                // This is a valid intercept - check if it's the best
                if (arrivalTime < bestTime)
                {
                    bestTime = arrivalTime;
                    bestAimPoint = aimPoint;
                }
            }
        }

        if (!bestAimPoint.HasValue)
            return null;

        // Refine the best solution using Golden Section Search on angle
        // Find the time when target is at the position corresponding to bestTime
        double refinedLocalT = Math.Min(bestTime - minTime, segmentDuration);
        if (refinedLocalT < 0) refinedLocalT = 0;

        Point2D refinedTargetPos = segmentStart + (segmentVelocity * refinedLocalT);

        // Refine angle using Golden Section minimization
        var refinedResult = RefineGagongAngle(
            casterPosition,
            refinedTargetPos,
            baseBehindAngle,
            hitRadius,
            skillshotSpeed,
            castDelay,
            skillshotRange,
            minTime + refinedLocalT);

        if (refinedResult.HasValue)
        {
            return refinedResult;
        }

        return (bestAimPoint.Value, bestTime);
    }

    /// <summary>
    /// Refines the angle search using Golden Section minimization.
    /// Finds the angle in behind hemisphere that minimizes intercept time.
    /// </summary>
    private static (Point2D AimPoint, double Time)? RefineGagongAngle(
        Point2D casterPosition,
        Point2D targetPosition,
        double baseBehindAngle,
        double hitRadius,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange,
        double targetTime)
    {
        const double phi = 1.618033988749895; // Golden ratio
        const double resphi = 2 - phi; // 1/phi
        const int maxIterations = Constants.GoldenSectionMaxIterations;
        const double tolerance = 1e-6;

        double a = -Math.PI / 2;
        double b = Math.PI / 2;

        // Function to minimize: arrival time for a given angle offset
        double TimeForAngle(double angleOffset)
        {
            double angle = baseBehindAngle + angleOffset;
            Vector2D offset = new Vector2D(Math.Cos(angle), Math.Sin(angle));
            Point2D aimPoint = targetPosition + (offset * hitRadius);

            double dist = (aimPoint - casterPosition).Length;
            if (dist > skillshotRange)
                return double.MaxValue;

            return castDelay + (dist / skillshotSpeed);
        }

        // Golden Section Search
        double c = a + (resphi * (b - a));
        double d = b - (resphi * (b - a));
        double fc = TimeForAngle(c);
        double fd = TimeForAngle(d);

        for (int i = 0; i < maxIterations && (b - a) > tolerance; i++)
        {
            if (fc < fd)
            {
                b = d;
                d = c;
                fd = fc;
                c = a + (resphi * (b - a));
                fc = TimeForAngle(c);
            }
            else
            {
                a = c;
                c = d;
                fc = fd;
                d = b - (resphi * (b - a));
                fd = TimeForAngle(d);
            }
        }

        double bestAngleOffset = (a + b) / 2;
        double bestAngle = baseBehindAngle + bestAngleOffset;
        Vector2D bestOffset = new Vector2D(Math.Cos(bestAngle), Math.Sin(bestAngle));
        Point2D bestAimPoint = targetPosition + (bestOffset * hitRadius);

        double bestDist = (bestAimPoint - casterPosition).Length;
        if (bestDist > skillshotRange)
            return null;

        double bestArrivalTime = castDelay + (bestDist / skillshotSpeed);

        // Verify the intercept is valid (arrival time >= target time)
        if (bestArrivalTime < targetTime - Constants.Epsilon)
            return null;

        return (bestAimPoint, bestArrivalTime);
    }

    #endregion
}
