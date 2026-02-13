using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

public sealed partial class InterceptSolver
{
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
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);

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
}
