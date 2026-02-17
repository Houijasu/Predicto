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
public sealed partial class InterceptSolver
{

    /// <summary>
    /// Validates common input parameters for interception solving.
    /// </summary>
    /// <exception cref="ArgumentException">Thrown when inputs are invalid.</exception>
    private static void ValidateInputs(
        double skillshotSpeed,
        double castDelay,
        double skillshotRange = double.MaxValue)
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

}
