using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

/// <summary>
/// Calculates optimal off-path aim points for "behind edge" skillshot targeting.
/// The aim point is intentionally NOT on the target's path - it's positioned so
/// the skillshot's edge clips the back of the target's hitbox.
/// </summary>
public static class OffPathAimPointSolver
{
    /// <summary>
    /// Calculates the optimal aim point that is OFF the target's path,
    /// positioned so the skillshot clips the trailing edge of the target's hitbox.
    /// 
    /// This solves the geometric problem: find point A such that:
    /// 1. A is at distance effectiveRadius from predicted position P
    /// 2. A is in the "behind" hemisphere (opposite to target velocity)
    /// 3. The skillshot from caster C aimed at A will hit the back edge of target
    /// </summary>
    public static Point2D CalculateBehindEdgeAimPoint(
        Point2D casterPosition,
        Point2D predictedPosition,
        Vector2D targetVelocity,
        double effectiveRadius,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Tangent)
    {
        return strategy switch
        {
            BehindEdgeStrategy.DirectBehind => CalculateDirectBehind(predictedPosition, targetVelocity, effectiveRadius),
            BehindEdgeStrategy.Tangent => CalculateTangentPoint(casterPosition, predictedPosition, targetVelocity, effectiveRadius),
            BehindEdgeStrategy.Adaptive => CalculateAdaptive(casterPosition, predictedPosition, targetVelocity, effectiveRadius),
            _ => CalculateTangentPoint(casterPosition, predictedPosition, targetVelocity, effectiveRadius)
        };
    }

    /// <summary>
    /// Strategy 1: Direct behind - aim point is directly behind target on path line.
    /// Simple but may not be optimal for all caster positions.
    /// </summary>
    private static Point2D CalculateDirectBehind(
        Point2D predictedPosition,
        Vector2D targetVelocity,
        double effectiveRadius)
    {
        if (targetVelocity.Length < Constants.Epsilon)
            return predictedPosition;

        Vector2D behindDir = targetVelocity.Normalize().Negate();
        return predictedPosition + behindDir * effectiveRadius;
    }

    /// <summary>
    /// Strategy 2: Tangent point - find the tangent from caster to target's hitbox circle
    /// that is in the "behind" hemisphere. This is geometrically optimal.
    /// </summary>
    private static Point2D CalculateTangentPoint(
        Point2D casterPosition,
        Point2D predictedPosition,
        Vector2D targetVelocity,
        double effectiveRadius)
    {
        Vector2D toTarget = predictedPosition - casterPosition;
        double distanceToTarget = toTarget.Length;

        // If caster is inside the effective radius, fall back to direct behind
        if (distanceToTarget <= effectiveRadius)
            return CalculateDirectBehind(predictedPosition, targetVelocity, effectiveRadius);

        Vector2D toTargetNorm = toTarget / distanceToTarget;

        // Target movement direction
        Vector2D moveDir = targetVelocity.Length > Constants.Epsilon
            ? targetVelocity.Normalize()
            : toTargetNorm; // Default to toward caster if stationary

        Vector2D behindDir = moveDir.Negate();

        // Calculate tangent angle: sin(θ) = R / d
        double sinTheta = effectiveRadius / distanceToTarget;
        double cosTheta = Math.Sqrt(1 - sinTheta * sinTheta);

        // Perpendicular to line of fire (two options: left and right)
        Vector2D perpLeft = new Vector2D(-toTargetNorm.Y, toTargetNorm.X);
        Vector2D perpRight = perpLeft.Negate();

        // Choose the perpendicular that points more toward "behind"
        Vector2D perp = perpLeft.DotProduct(behindDir) > perpRight.DotProduct(behindDir)
            ? perpLeft
            : perpRight;

        // Calculate tangent point
        // The tangent point is at: P + perp * R (on the circle)
        // But we need to verify it's reachable from C
        Point2D tangentPoint = predictedPosition + perp * effectiveRadius;

        // Verify the tangent point is in the "behind" hemisphere
        Vector2D tangentOffset = tangentPoint - predictedPosition;
        double behindness = tangentOffset.DotProduct(behindDir);

        if (behindness > 0)
        {
            return tangentPoint;
        }

        // If tangent point is not behind enough, blend toward direct behind
        // This handles edge cases where caster is nearly perpendicular to target path
        Point2D directBehind = predictedPosition + behindDir * effectiveRadius;

        // Blend based on how "behind" the tangent point is
        double blendFactor = Math.Max(0, behindness / effectiveRadius + 0.5);
        return Lerp(directBehind, tangentPoint, blendFactor);
    }

    /// <summary>
    /// Strategy 3: Adaptive - returns the center point between Tangent and DirectBehind (trailing edge).
    /// This provides a balanced aim point that works well for all movement directions.
    /// The result is projected back onto the effective radius circle.
    /// </summary>
    private static Point2D CalculateAdaptive(
        Point2D casterPosition,
        Point2D predictedPosition,
        Vector2D targetVelocity,
        double effectiveRadius)
    {
        if (targetVelocity.Length < Constants.Epsilon)
            return CalculateDirectBehind(predictedPosition, targetVelocity, effectiveRadius);

        // Calculate both aim points
        Point2D tangentPoint = CalculateTangentPoint(casterPosition, predictedPosition, targetVelocity, effectiveRadius);
        Point2D directBehindPoint = CalculateDirectBehind(predictedPosition, targetVelocity, effectiveRadius);

        // Simply take the center between Tangent and DirectBehind
        Point2D centerPoint = Lerp(tangentPoint, directBehindPoint, 0.5);

        // Project back onto the effective radius circle to maintain valid aim geometry
        Vector2D offset = centerPoint - predictedPosition;
        double offsetLength = offset.Length;
        if (offsetLength < Constants.Epsilon)
        {
            // Center is at predicted position, fall back to direct behind
            return directBehindPoint;
        }

        // Scale to effective radius
        return predictedPosition + offset * (effectiveRadius / offsetLength);
    }

    /// <summary>
    /// Solves the complete intercept problem with off-path aim point.
    /// Returns both the aim point and the intercept time.
    /// </summary>
    public static (Point2D AimPoint, double InterceptTime)? SolveOffPathIntercept(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetRadius,
        double skillshotWidth,
        double skillshotRange,
        BehindEdgeStrategy strategy = BehindEdgeStrategy.Tangent)
    {
        double effectiveRadius = targetRadius + skillshotWidth / 2;

        // Iterative solution: find intercept time, then calculate off-path aim point
        // The challenge: aim point affects travel time, which affects intercept time

        // Initial estimate using center prediction
        var centerResult = InterceptSolver.SolvePathEdgeIntercept(
            casterPosition, path, skillshotSpeed, castDelay,
            targetRadius, skillshotWidth, skillshotRange);

        if (centerResult == null)
            return null;

        double interceptTime = centerResult.Value.InterceptTime;
        Point2D predictedPos = path.GetPositionAtTime(interceptTime);
        Vector2D velocity = path.GetVelocityAtTime(interceptTime);

        // Calculate off-path aim point
        Point2D aimPoint = CalculateBehindEdgeAimPoint(
            casterPosition, predictedPos, velocity, effectiveRadius, strategy);

        // Refine: the aim point changes the travel distance, which may change intercept time
        // Sug 4: Continuous Collision Detection (CCD)
        // Iterative refinement using Brent's method to find the true intercept time
        // for an off-path aim point. This is more robust than a fixed 5-iteration loop.

        Func<double, double> timeResidual = t =>
        {
            var pos = path.GetPositionAtTime(t);
            var v = path.GetVelocityAtTime(t);
            var ap = CalculateBehindEdgeAimPoint(casterPosition, pos, v, effectiveRadius, strategy);
            double dist = (ap - casterPosition).Length;
            return (castDelay + dist / skillshotSpeed) - t;
        };

        // Bracket the search: [0, maxTime]
        double maxTime = castDelay + skillshotRange / skillshotSpeed + path.GetRemainingPathTime();
        if (Brent.TryFindRoot(timeResidual, 0, maxTime, Constants.Epsilon, 100, out double refinedTime))
        {
            interceptTime = refinedTime;
            predictedPos = path.GetPositionAtTime(interceptTime);
            velocity = path.GetVelocityAtTime(interceptTime);
            aimPoint = CalculateBehindEdgeAimPoint(casterPosition, predictedPos, velocity, effectiveRadius, strategy);
        }
        else
        {
            // Fallback to fixed iteration if Brent fails to bracket (e.g. discontinuous path)
            for (int iteration = 0; iteration < 10; iteration++)
            {
                double travelDistance = (aimPoint - casterPosition).Length;
                double travelTime = travelDistance / skillshotSpeed;
                double newInterceptTime = castDelay + travelTime;

                if (Math.Abs(newInterceptTime - interceptTime) < Constants.Epsilon)
                    break;

                interceptTime = newInterceptTime;
                predictedPos = path.GetPositionAtTime(interceptTime);
                velocity = path.GetVelocityAtTime(interceptTime);
                aimPoint = CalculateBehindEdgeAimPoint(casterPosition, predictedPos, velocity, effectiveRadius, strategy);
            }
        }

        // Validate range
        double finalDistance = (aimPoint - casterPosition).Length;
        if (finalDistance > skillshotRange)
            return null;

        return (aimPoint, interceptTime);
    }

    private static Point2D Lerp(Point2D a, Point2D b, double t)
    {
        return new Point2D(
            a.X + (b.X - a.X) * t,
            a.Y + (b.Y - a.Y) * t);
    }

    /// <summary>
    /// Calculates the TRUE intercept time - when the skillshot's edge first touches the target's hitbox.
    /// This solves the geometric collision problem between a moving LINE (skillshot path) and moving CIRCLE (target).
    /// 
    /// For off-path aim points, the skillshot travels along a line toward the aim point.
    /// Collision occurs when the target's hitbox touches this line (perpendicular distance = collision radius).
    /// </summary>
    /// <param name="casterPosition">Where the skillshot originates</param>
    /// <param name="aimPoint">Where the skillshot is aimed (determines direction)</param>
    /// <param name="targetPosition">Target's current position</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the projectile</param>
    /// <param name="castDelay">Delay before projectile starts moving</param>
    /// <param name="targetRadius">Target's hitbox radius</param>
    /// <param name="skillshotWidth">Width of the skillshot</param>
    /// <returns>Time in seconds when collision occurs, or null if no collision</returns>
    public static double? CalculateTrueInterceptTime(
        Point2D casterPosition,
        Point2D aimPoint,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetRadius,
        double skillshotWidth)
    {
        // Combined collision radius
        double collisionRadius = targetRadius + skillshotWidth / 2;

        // Direction from caster to aim point
        Vector2D toAim = aimPoint - casterPosition;
        double aimDistance = toAim.Length;
        if (aimDistance < Constants.Epsilon)
            return null;

        Vector2D shotDirection = toAim / aimDistance;

        // Perpendicular to shot direction (for calculating distance to line)
        Vector2D perpendicular = new Vector2D(-shotDirection.Y, shotDirection.X);

        // The skillshot travels along the line: C + shotDirection * s * (t - delay)
        // The target moves: P + V * t
        //
        // Distance from target to skillshot LINE (not point) is:
        //   d(t) = |( Target(t) - C ) · perpendicular|
        //        = |( P + V*t - C ) · perpendicular|
        //        = |( (P - C) · perp ) + ( V · perp ) * t|
        //
        // Collision when d(t) = collisionRadius AND projectile has traveled far enough

        Vector2D casterToTarget = targetPosition - casterPosition;
        double initialPerpDist = casterToTarget.DotProduct(perpendicular);
        double perpVelocity = targetVelocity.DotProduct(perpendicular);

        // perpDist(t) = initialPerpDist + perpVelocity * t
        // |perpDist(t)| = collisionRadius
        //
        // Two cases:
        // Case 1: initialPerpDist + perpVelocity * t = collisionRadius
        // Case 2: initialPerpDist + perpVelocity * t = -collisionRadius

        double? t1 = null, t2 = null;

        if (Math.Abs(perpVelocity) > Constants.Epsilon)
        {
            t1 = (collisionRadius - initialPerpDist) / perpVelocity;
            t2 = (-collisionRadius - initialPerpDist) / perpVelocity;
        }
        else
        {
            // Target not moving perpendicular to shot line
            // Check if it's already within collision distance
            if (Math.Abs(initialPerpDist) <= collisionRadius)
            {
                // Target path is parallel and within collision range
                // Find when projectile reaches target's forward position
                double forwardDist = casterToTarget.DotProduct(shotDirection);
                double forwardVelocity = targetVelocity.DotProduct(shotDirection);

                // Projectile position along line at time t: speed * (t - delay)
                // Target position along line at time t: forwardDist + forwardVelocity * t
                // 
                // Collision when projectile catches up to target (within collision radius):
                // speed * (t - delay) >= forwardDist + forwardVelocity * t - collisionRadius
                // (speed - forwardVelocity) * t >= forwardDist - collisionRadius + speed * delay

                double relativeSpeed = skillshotSpeed - forwardVelocity;
                if (relativeSpeed > Constants.Epsilon)
                {
                    double tCollision = (forwardDist - collisionRadius + skillshotSpeed * castDelay) / relativeSpeed;
                    if (tCollision >= castDelay)
                        return tCollision;
                }
                return null;
            }
            return null; // Target path misses the shot line entirely
        }

        // For each candidate time, verify:
        // 1. t >= castDelay (projectile must have launched)
        // 2. Projectile has traveled far enough to reach the target's forward position
        // 3. Target is in front of caster (not behind)

        double? bestTime = null;

        foreach (double? tCandidate in new[] { t1, t2 })
        {
            if (!tCandidate.HasValue || tCandidate.Value < castDelay)
                continue;

            double t = tCandidate.Value;

            // Where is the projectile at time t?
            double projectileForwardPos = skillshotSpeed * (t - castDelay);

            // Where is the target (along the shot direction) at time t?
            Vector2D targetAtT = (targetPosition - casterPosition) + targetVelocity * t;
            double targetForwardPos = targetAtT.DotProduct(shotDirection);

            // Projectile must have reached the target's position (within collision radius)
            // Projectile leading edge is at: projectileForwardPos + collisionRadius (we want edge contact)
            // Actually, for edge contact: projectileForwardPos >= targetForwardPos - collisionRadius
            if (projectileForwardPos >= targetForwardPos - collisionRadius && targetForwardPos > 0)
            {
                if (!bestTime.HasValue || t < bestTime.Value)
                    bestTime = t;
            }
        }

        return bestTime;
    }
}


