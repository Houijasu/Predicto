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
            BehindEdgeStrategy.OptimalAngle => CalculateOptimalAngle(casterPosition, predictedPosition, targetVelocity, effectiveRadius),
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
    /// Strategy 3: Optimal angle - uses continuous math to find the angle on the hitbox circle
    /// that balances "behindness" (70%) and "accessibility" from caster (30%).
    /// 
    /// Instead of discrete sampling, this computes the optimal angle analytically by:
    /// 1. Finding the caster's angle relative to target movement direction
    /// 2. Clamping to the behind hemisphere [π/2, 3π/2]
    /// 3. Blending between this angle and π (directly behind) using the 0.7/0.3 weights
    /// 
    /// This produces smooth, continuous transitions as positions change.
    /// </summary>
    private static Point2D CalculateOptimalAngle(
        Point2D casterPosition,
        Point2D predictedPosition,
        Vector2D targetVelocity,
        double effectiveRadius)
    {
        if (targetVelocity.Length < Constants.Epsilon)
            return predictedPosition;

        Vector2D moveDir = targetVelocity.Normalize();
        
        // Perpendicular to movement direction
        Vector2D perpToMove = new Vector2D(-moveDir.Y, moveDir.X);

        // Express caster position in the coordinate system centered at predicted position
        // with moveDir as X-axis and perpToMove as Y-axis
        Vector2D towardCaster = casterPosition - predictedPosition;
        double casterAlongMove = towardCaster.DotProduct(moveDir);    // Cm
        double casterAlongPerp = towardCaster.DotProduct(perpToMove); // Cp

        // Calculate caster's angle in this coordinate system
        // Angle 0 = movement direction, π = directly behind
        double casterAngle = Math.Atan2(casterAlongPerp, casterAlongMove);

        // Clamp to behind hemisphere [π/2, 3π/2]
        // This ensures we never aim at the front of the target
        double clampedAngle = ClampAngleToBehindHemisphere(casterAngle);

        // Blend between clamped caster angle (accessibility) and π (directly behind)
        // Using weights: 0.7 for behindness, 0.3 for accessibility
        const double directBehindAngle = Math.PI;
        const double behindWeight = 0.7;
        double optimalAngle = clampedAngle + (directBehindAngle - clampedAngle) * behindWeight;

        // Convert angle back to point on circle
        double x = Math.Cos(optimalAngle);
        double y = Math.Sin(optimalAngle);
        return predictedPosition + moveDir * (x * effectiveRadius) + perpToMove * (y * effectiveRadius);
    }

    /// <summary>
    /// Clamps an angle to the behind hemisphere [π/2, 3π/2].
    /// Angles in the front hemisphere are clamped to the nearest edge.
    /// </summary>
    private static double ClampAngleToBehindHemisphere(double angle)
    {
        // Normalize to [-π, π]
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;

        // Behind hemisphere is where x < 0, i.e., angle in (π/2, 3π/2) or equivalently |angle| > π/2
        if (Math.Abs(angle) >= Math.PI / 2)
        {
            // Already in behind hemisphere
            return angle;
        }

        // In front hemisphere - clamp to nearest edge (π/2 or -π/2)
        return angle >= 0 ? Math.PI / 2 : -Math.PI / 2;
    }

    /// <summary>
    /// Strategy 4: Adaptive - returns the center point between Tangent and DirectBehind (trailing edge).
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
        // Iterate to converge
        for (int iteration = 0; iteration < 5; iteration++)
        {
            double travelDistance = (aimPoint - casterPosition).Length;
            double travelTime = travelDistance / skillshotSpeed;
            double newInterceptTime = castDelay + travelTime;

            // Check if intercept time changed significantly
            if (Math.Abs(newInterceptTime - interceptTime) < 0.001)
                break;

            interceptTime = newInterceptTime;
            predictedPos = path.GetPositionAtTime(interceptTime);
            velocity = path.GetVelocityAtTime(interceptTime);

            aimPoint = CalculateBehindEdgeAimPoint(
                casterPosition, predictedPos, velocity, effectiveRadius, strategy);
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
    /// This solves the geometric collision problem between a moving projectile and moving target.
    /// 
    /// Given:
    /// - Projectile travels from caster toward aimPoint at skillshotSpeed after castDelay
    /// - Target starts at targetPosition and moves with targetVelocity
    /// - Collision occurs when distance between centers = targetRadius + skillshotWidth/2
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

        // Projectile position at time t (for t >= castDelay):
        //   Proj(t) = C + shotDirection * speed * (t - delay)
        //
        // Target position at time t:
        //   Target(t) = P + V * t
        //
        // Collision when |Proj(t) - Target(t)| = collisionRadius
        //
        // Let D = C - P (vector from target to caster)
        // |D + shotDirection * speed * (t - delay) - V * t| = collisionRadius
        //
        // Rearrange:
        // |(D - shotDirection * speed * delay) + (shotDirection * speed - V) * t| = collisionRadius
        //
        // Let:
        //   w = D - shotDirection * speed * delay  (initial offset)
        //   u = shotDirection * speed - V          (relative velocity)
        //
        // |w + u*t|^2 = R^2
        // |u|^2 * t^2 + 2*(w·u)*t + |w|^2 - R^2 = 0

        Vector2D D = casterPosition - targetPosition;
        Vector2D w = D - shotDirection * skillshotSpeed * castDelay;
        Vector2D u = shotDirection * skillshotSpeed - targetVelocity;

        double a = u.DotProduct(u);            // |u|^2
        double b = 2 * w.DotProduct(u);        // 2*(w·u)
        double c = w.DotProduct(w) - collisionRadius * collisionRadius;  // |w|^2 - R^2

        // Solve quadratic at^2 + bt + c = 0
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
            return null; // No intersection

        double sqrtDisc = Math.Sqrt(discriminant);

        // Two solutions - we want the smallest t >= castDelay
        double t1 = (-b - sqrtDisc) / (2 * a);
        double t2 = (-b + sqrtDisc) / (2 * a);

        // Return the first valid collision time (>= castDelay)
        if (t1 >= castDelay)
            return t1;
        if (t2 >= castDelay)
            return t2;

        return null; // Collision happened in the past or during delay
    }
}

public enum BehindEdgeStrategy
{
    /// <summary>
    /// Aim directly behind target on path line extended.
    /// Simple, works well when caster is roughly aligned with target path.
    /// </summary>
    DirectBehind,

    /// <summary>
    /// Calculate tangent point from caster to target's hitbox circle.
    /// Geometrically optimal for edge clipping.
    /// </summary>
    Tangent,

    /// <summary>
    /// Search for optimal angle that balances behindness and hit reliability.
    /// Most robust but slightly more expensive.
    /// </summary>
    OptimalAngle,

    /// <summary>
    /// Adaptive strategy: uses Tangent when target moves toward caster (faster intercept),
    /// and DirectBehind when target moves away. Smoothly blends between them to avoid
    /// sudden jumps in aim position.
    /// </summary>
    Adaptive
}
