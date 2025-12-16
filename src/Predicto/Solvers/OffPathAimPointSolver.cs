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
    /// Strategy 3: Optimal angle - search for the angle on the hitbox circle
    /// that maximizes "behindness" while maintaining valid intercept geometry.
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
        Vector2D behindDir = moveDir.Negate();

        // Perpendicular to movement (for parameterizing the circle)
        Vector2D perpToMove = new Vector2D(-moveDir.Y, moveDir.X);

        // Search angles in the "behind" hemisphere (90° to 270° relative to movement)
        // Angle 0 = movement direction, 180° = behind direction
        Point2D bestPoint = predictedPosition + behindDir * effectiveRadius;
        double bestScore = double.MinValue;

        // Sample angles in the behind hemisphere
        for (int i = 0; i <= 20; i++)
        {
            // Angle from 90° to 270° (behind hemisphere)
            double t = i / 20.0;
            double angle = Math.PI * (0.5 + t); // 90° to 270°

            // Point on circle at this angle
            double x = Math.Cos(angle);
            double y = Math.Sin(angle);
            Point2D circlePoint = predictedPosition + moveDir * (x * effectiveRadius) + perpToMove * (y * effectiveRadius);

            // Score: combination of "behindness" and "ease of hit" from caster
            Vector2D offset = circlePoint - predictedPosition;
            double behindness = offset.Normalize().DotProduct(behindDir);

            // Prefer points that are more directly reachable from caster
            Vector2D toCaster = (casterPosition - circlePoint).Normalize();
            Vector2D toCenter = (predictedPosition - circlePoint).Normalize();
            double accessibility = toCaster.DotProduct(toCenter);

            double score = behindness * 0.7 + accessibility * 0.3;

            if (score > bestScore)
            {
                bestScore = score;
                bestPoint = circlePoint;
            }
        }

        return bestPoint;
    }

    /// <summary>
    /// Strategy 4: Adaptive - smoothly blends between Tangent and DirectBehind based on
    /// whether the target is moving toward or away from the caster.
    /// 
    /// - Target moving toward caster: Use Tangent (faster intercept)
    /// - Target moving away from caster: Use DirectBehind (target walks into skillshot)
    /// - Target moving perpendicular: Blend between both strategies
    /// 
    /// The blend uses a smooth transition to avoid sudden jumps in aim position.
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

        // Determine if target is moving toward or away from caster
        Vector2D toCaster = (casterPosition - predictedPosition);
        double distToCaster = toCaster.Length;
        if (distToCaster < Constants.Epsilon)
            return tangentPoint;

        Vector2D toCasterNorm = toCaster / distToCaster;
        Vector2D moveDir = targetVelocity.Normalize();

        // Dot product: +1 = moving directly toward caster, -1 = moving directly away
        // 0 = moving perpendicular
        double approachFactor = moveDir.DotProduct(toCasterNorm);

        // Map approach factor to blend weight using smooth transition
        // approachFactor in [-1, 1] -> blendWeight in [0, 1]
        // +1 (toward) -> 1.0 (use Tangent)
        // -1 (away)   -> 0.0 (use DirectBehind)
        // 0 (perp)    -> 0.5 (blend)
        // Use smoothstep for smooth transition
        double t = (approachFactor + 1.0) / 2.0; // Map [-1, 1] to [0, 1]
        double blendWeight = SmoothStep(t);

        // Blend between DirectBehind (weight=0) and Tangent (weight=1)
        return Lerp(directBehindPoint, tangentPoint, blendWeight);
    }

    /// <summary>
    /// Attempt to get a smoother blend value beyond regular smoothstep.
    /// </summary>
    private static double SmoothStep(double t)
    {
        // Regular smoothstep: 3t² - 2t³
        t = Math.Clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
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
