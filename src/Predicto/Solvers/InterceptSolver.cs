using MathNet.Numerics;
using MathNet.Spatial.Euclidean;

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
        double maxTime = d + skillshotRange / s;

        // Handle stationary target (optimization and numerical stability)
        if (V.Length < Constants.MinVelocity)
        {
            return SolveStationary(D.Length, s, d);
        }

        // Quadratic coefficients: at² + bt + c = 0
        double a = V.DotProduct(V) - s * s;
        double b = 2 * D.DotProduct(V) + 2 * s * s * d;
        double c = D.DotProduct(D) - s * s * d * d;

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
        double r = targetHitboxRadius + skillshotWidth / 2;

        // Displacement vector from caster to target
        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);
        var V = targetVelocity;
        double s = skillshotSpeed;
        double d = castDelay;

        // Calculate max valid time: delay + (range + effectiveRadius) / speed
        // We add effectiveRadius because edge collision extends effective range
        double maxTime = d + (skillshotRange + r) / s;

        // Handle stationary target
        if (V.Length < Constants.MinVelocity)
        {
            return SolveStationaryEdge(D.Length, s, d, r);
        }

        // Check if target will be within collision radius at launch time
        // This is more accurate than checking current position for moving targets
        var positionAtLaunch = D + V * d;
        if (positionAtLaunch.Length <= r)
        {
            // Target will be in collision range when skillshot launches
            return d;
        }

        // Quadratic coefficients for edge-to-edge collision:
        // |D + V·t|² = (s·(t-d) + r)²
        // Expanding: (|V|² - s²)t² + (2(D·V) + 2s²d - 2sr)t + (|D|² - (sd-r)²) = 0
        double a = V.DotProduct(V) - s * s;
        double b = 2 * D.DotProduct(V) + 2 * s * s * d - 2 * s * r;
        double sdMinusR = s * d - r;
        double c = D.DotProduct(D) - sdMinusR * sdMinusR;

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
    /// </summary>
    private static double? SolveLinearEdge(double b, double c, double minTime, double maxTime)
    {
        if (Math.Abs(b) < Constants.Epsilon)
            return null;

        double t = -c / b;
        return t >= minTime && t <= maxTime ? t : null;
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
        double maxPredictionTime = castDelay + skillshotRange / skillshotSpeed;

        // Time factor: exponential decay - uncertainty grows exponentially with time
        // Weight: 0.5 (highest - time is most important for prediction accuracy)
        double timeFactor = Math.Exp(-interceptTime / maxPredictionTime);

        // Speed factor: slower targets relative to skillshot are more predictable
        // Weight: 0.3
        double speedFactor = 1.0 / (1.0 + targetSpeed / (skillshotSpeed + Constants.Epsilon));

        // Range factor: closer targets are more predictable
        // Weight: 0.2
        double rangeFactor = Math.Max(0, 1.0 - distance / skillshotRange);

        // Weighted sum combination
        double confidence = 0.5 * timeFactor + 0.3 * speedFactor + 0.2 * rangeFactor;

        return Math.Clamp(confidence, 0.1, 1.0);
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
        var relativePosition = displacement + targetVelocity * t;
        double distanceToTarget = relativePosition.Length;

        // Projectile travel distance at time t (starts moving after delay)
        double flightTime = Math.Max(0, t - castDelay);
        double projectileDistance = skillshotSpeed * flightTime + effectiveRadius;

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
        var relativePosition = displacement + targetVelocity * t;
        double distanceToTarget = relativePosition.Length;

        double flightTime = Math.Max(0, t - castDelay);
        // Reduce effective radius by margin to find point inside collision zone
        double projectileDistance = skillshotSpeed * flightTime + (effectiveRadius - margin);

        return distanceToTarget - projectileDistance;
    }

    /// <summary>
    /// Solves for interception time using the bisection method.
    /// Finds root of f(t) = |D + V·t| - (s·(t-d) + r) = 0.
    /// 
    /// The bisection method is guaranteed to converge if a root exists in the bracket.
    /// It's more robust than quadratic solving for edge cases and can handle
    /// scenarios where numerical instability affects the quadratic formula.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="targetPosition">Current position of the target</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="effectiveRadius">Combined collision radius</param>
    /// <param name="maxTime">Maximum valid time to search</param>
    /// <param name="tolerance">Convergence tolerance (default 1e-9 for high precision)</param>
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
        double fHigh = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tHigh);

        // Check if target is already in collision range at launch
        if (fLow <= 0)
            return tLow;

        // No sign change means no root in interval (target unreachable)
        if (fLow * fHigh > 0)
            return null;

        // Bisection iterations
        for (int i = 0; i < maxIterations; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tMid);

            // Check convergence
            if (Math.Abs(fMid) < tolerance || (tHigh - tLow) / 2 < tolerance)
                return tMid;

            // Narrow the bracket
            if (fLow * fMid < 0)
            {
                tHigh = tMid;
                // fHigh = fMid; // Not needed, we use fLow for sign check
            }
            else
            {
                tLow = tMid;
                fLow = fMid;
            }
        }

        // Return best estimate after max iterations
        return (tLow + tHigh) / 2;
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
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        double maxTime = castDelay + (skillshotRange + effectiveRadius) / skillshotSpeed;

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
            double bracketSize = Math.Max(0.1, (maxTime - castDelay) * 0.1); // 10% of time range or 0.1s
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
    /// Bisection solver with pre-computed bracket and function values.
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
        for (int i = 0; i < maxIterations; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tMid);

            if (Math.Abs(fMid) < tolerance || (tHigh - tLow) / 2 < tolerance)
                return tMid;

            if (fLow * fMid < 0)
            {
                tHigh = tMid;
            }
            else
            {
                tLow = tMid;
                fLow = fMid;
            }
        }

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
        double r = targetHitboxRadius + skillshotWidth / 2;

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);
        var V = targetVelocity;
        double s = skillshotSpeed;
        double d = castDelay;

        double maxTime = d + (skillshotRange + r) / s;

        // Handle stationary target - trailing edge = leading edge for stationary
        if (V.Length < Constants.MinVelocity)
        {
            return SolveStationaryEdgeTrailing(D.Length, s, d, r);
        }

        // Check if target will be within collision radius at launch time
        var positionAtLaunch = D + V * d;
        if (positionAtLaunch.Length <= r)
        {
            // Target in collision range at launch - find when it EXITS
            // This is the trailing edge time
            return FindCollisionExitTime(D, V, s, d, r, maxTime);
        }

        // Quadratic coefficients for edge-to-edge collision
        double a = V.DotProduct(V) - s * s;
        double b = 2 * D.DotProduct(V) + 2 * s * s * d - 2 * s * r;
        double sdMinusR = s * d - r;
        double c = D.DotProduct(D) - sdMinusR * sdMinusR;

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
    /// For stationary targets, collision window is when projectile passes through.
    /// Trailing = center distance + effective radius.
    /// </summary>
    private static double? SolveStationaryEdgeTrailing(double distance, double speed, double delay, double effectiveRadius)
    {
        // If already within collision radius at start, trailing is when projectile exits
        if (distance <= effectiveRadius)
        {
            // Projectile needs to travel (effectiveRadius + distance) to exit collision zone
            // Actually for a point projectile passing through a circular target:
            // Entry at: distance - effectiveRadius (or 0 if inside)
            // Exit at: distance + effectiveRadius
            double exitDistance = distance + effectiveRadius;
            double flightTime = exitDistance / speed;
            return delay + flightTime;
        }

        // Travel to far edge: distance + effectiveRadius
        double travelDistance = distance + effectiveRadius;
        double flight = travelDistance / speed;
        return delay + flight;
    }

    /// <summary>
    /// Finds when target exits the collision zone (for targets inside zone at launch).
    /// Uses bisection to find where f(t) changes from negative to positive.
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

        // Bisection to find exit point
        for (int i = 0; i < 100; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tMid);

            if (Math.Abs(fMid) < 1e-9 || (tHigh - tLow) / 2 < 1e-9)
                return tMid;

            if (fMid < 0)
                tLow = tMid;
            else
                tHigh = tMid;
        }

        return (tLow + tHigh) / 2;
    }

    /// <summary>
    /// Solves for the LATEST interception time (trailing edge) using bisection.
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
        double searchStart = entryTime.Value + tolerance * 10;
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

        // Bisection for exit point
        double tLow = searchStart;
        double tHigh = maxTime;
        double fLow = fStart;

        for (int i = 0; i < maxIterations; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateCollisionFunction(D, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tMid);

            if (Math.Abs(fMid) < tolerance || (tHigh - tLow) / 2 < tolerance)
                return tMid;

            if (fMid <= 0)
            {
                tLow = tMid;
                fLow = fMid;
            }
            else
            {
                tHigh = tMid;
            }
        }

        return (tLow + tHigh) / 2;
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
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        double maxTime = castDelay + (skillshotRange + effectiveRadius) / skillshotSpeed;

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
            double bracketSize = Math.Max(0.1, (maxTime - castDelay) * 0.1);
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
        for (int i = 0; i < maxIterations; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateCollisionFunction(displacement, targetVelocity, skillshotSpeed, castDelay, effectiveRadius, tMid);

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

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;

        // IMPORTANT: Margin must be less than effectiveRadius for the math to work.
        // If margin >= effectiveRadius, the "behind edge" concept breaks down
        // (no room inside the collision zone). Clamp to 90% of effectiveRadius.
        margin = Math.Min(margin, effectiveRadius * 0.9);
        double maxTime = castDelay + (skillshotRange + effectiveRadius) / skillshotSpeed;

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
        const double tolerance = 1e-9;
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
        // Behind edge = far edge minus margin
        double travelDistance;

        if (distance <= effectiveRadius)
        {
            // Target inside collision zone - travel to far edge minus margin
            travelDistance = distance + effectiveRadius - margin;
        }
        else
        {
            // Target outside - travel to far edge minus margin
            travelDistance = distance + effectiveRadius - margin;
        }

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

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;

        // IMPORTANT: Margin must be less than effectiveRadius for the math to work.
        // If margin >= effectiveRadius, the "behind edge" concept breaks down
        // (no room inside the collision zone). Clamp to 90% of effectiveRadius.
        margin = Math.Min(margin, effectiveRadius * 0.9);

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

        double maxTime = castDelay + (skillshotRange + effectiveRadius) / skillshotSpeed;

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        if (trailingEstimate.HasValue)
        {
            // Refine around the trailing estimate using margin
            double estimate = trailingEstimate.Value;
            double bracketSize = Math.Max(0.05, (maxTime - castDelay) * 0.05);

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

                const double tolerance = 1e-9;
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

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;

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
        double behindDistance = effectiveRadius - behindMargin;
        if (behindDistance < 0) behindDistance = 0;
        
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
    /// Evaluates the center-to-center intercept function for Secant/Newton refinement.
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
        var relativePosition = displacement + targetVelocity * t;
        double distanceToTarget = relativePosition.Length;

        // Projectile travel distance at time t (starts moving after delay)
        double flightTime = Math.Max(0, t - castDelay);
        double projectileDistance = skillshotSpeed * flightTime;

        return distanceToTarget - projectileDistance;
    }

    /// <summary>
    /// Refines an initial time estimate using the Secant Method.
    /// 
    /// The Secant Method is a root-finding algorithm that approximates Newton-Raphson
    /// without requiring explicit derivative calculation:
    /// 
    ///   t_{n+1} = t_n - f(t_n) · (t_n - t_{n-1}) / (f(t_n) - f(t_{n-1}))
    /// 
    /// Convergence: Superlinear (order ≈ 1.618, the golden ratio)
    /// Typically converges in 4-8 iterations to machine precision.
    /// 
    /// Advantages over other methods:
    /// - Faster than Bisection (superlinear vs linear convergence)
    /// - Simpler than Newton-Raphson (no derivative needed)
    /// - More robust than Fixed-Point iteration
    /// </summary>
    /// <param name="displacement">Vector from caster to target at t=0</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="initialGuess">Initial estimate (e.g., from quadratic solver)</param>
    /// <param name="tolerance">Convergence tolerance (default 1e-12 for high precision)</param>
    /// <param name="maxIterations">Maximum iterations (default 20)</param>
    /// <returns>Refined intercept time, or the best estimate if max iterations reached</returns>
    private static double RefineWithSecant(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double initialGuess,
        double tolerance = 1e-12,
        int maxIterations = 20)
    {
        // Secant method needs two initial points
        // Use initialGuess and a small perturbation
        double t0 = initialGuess;
        double t1 = initialGuess * 1.001 + 1e-6; // Small offset to avoid division by zero
        
        double f0 = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, t0);
        double f1 = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, t1);
        
        // If initial guess is already very accurate, return it
        if (Math.Abs(f0) < tolerance)
            return t0;
        
        for (int i = 0; i < maxIterations; i++)
        {
            // Avoid division by zero
            double denominator = f1 - f0;
            if (Math.Abs(denominator) < 1e-15)
                break;
            
            // Secant formula: t_{n+1} = t_n - f(t_n) * (t_n - t_{n-1}) / (f(t_n) - f(t_{n-1}))
            double t2 = t1 - f1 * (t1 - t0) / denominator;
            
            // Ensure t2 is valid (positive, after delay)
            if (t2 < castDelay)
                t2 = castDelay + tolerance;
            
            double f2 = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, t2);
            
            // Check convergence
            if (Math.Abs(f2) < tolerance || Math.Abs(t2 - t1) < tolerance)
                return t2;
            
            // Shift for next iteration
            t0 = t1;
            f0 = f1;
            t1 = t2;
            f1 = f2;
        }
        
        // Return best estimate
        return t1;
    }

    /// <summary>
    /// Solves for center-to-center intercept time using quadratic formula
    /// with Secant Method refinement for maximum precision.
    /// 
    /// Strategy:
    /// 1. Use fast quadratic solver for O(1) initial estimate
    /// 2. Refine with Secant Method for ~1e-12 precision
    /// 
    /// This hybrid approach combines:
    /// - Speed of closed-form solution (instant initial guess)
    /// - Precision of iterative refinement (machine-precision result)
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

        // Step 2: Refine with Secant Method
        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        double refined = RefineWithSecant(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            initialEstimate.Value,
            tolerance);

        // Validate refined result is within range
        double maxTime = castDelay + skillshotRange / skillshotSpeed;
        if (refined < castDelay || refined > maxTime)
            return initialEstimate; // Fall back to quadratic if refinement went out of bounds

        return refined;
    }

    /// <summary>
    /// Solves for behind-target interception with Secant Method refinement.
    /// 
    /// This is the highest-precision method for behind-target prediction:
    /// 1. Calculate virtual target position (offset behind real target)
    /// 2. Solve intercept using quadratic formula (O(1) initial guess)
    /// 3. Refine with Secant Method to ~1e-12 precision
    /// 
    /// The Secant refinement ensures numerical accuracy even in edge cases
    /// where the quadratic formula might have precision loss due to:
    /// - Near-equal roots (discriminant close to zero)
    /// - Large coefficient magnitudes
    /// - Subtraction of similar values
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

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;

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
        double behindDistance = effectiveRadius - behindMargin;
        if (behindDistance < 0) behindDistance = 0;
        
        Vector2D moveDirection = targetVelocity.Normalize();
        Vector2D behindOffset = moveDirection.Negate() * behindDistance;
        
        // Create virtual target position: offset BEHIND the real target
        Point2D virtualTargetPosition = targetPosition + behindOffset;
        
        // Solve intercept with Secant refinement for maximum precision
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
    /// Refines a time estimate using Bisection Method for guaranteed convergence.
    /// 
    /// Bisection is the most robust root-finding method:
    /// - GUARANTEED to converge if root exists in bracket
    /// - Linear convergence: each iteration halves the error
    /// - ~50 iterations for machine precision (1e-15)
    /// 
    /// Used as final refinement step after Secant for absolute precision.
    /// </summary>
    /// <param name="displacement">Vector from caster to target at t=0</param>
    /// <param name="targetVelocity">Target's velocity vector</param>
    /// <param name="skillshotSpeed">Speed of the skillshot</param>
    /// <param name="castDelay">Delay before skillshot launches</param>
    /// <param name="estimate">Estimate from previous refinement (Secant)</param>
    /// <param name="bracketSize">Size of bracket around estimate to search</param>
    /// <param name="tolerance">Convergence tolerance (default 1e-15 for pixel precision)</param>
    /// <param name="maxIterations">Maximum iterations (default 60)</param>
    /// <returns>Refined intercept time with guaranteed precision</returns>
    private static double RefineWithBisection(
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double estimate,
        double bracketSize = 0.01,
        double tolerance = 1e-15,
        int maxIterations = 60)
    {
        // Create bracket around estimate
        double tLow = Math.Max(castDelay, estimate - bracketSize);
        double tHigh = estimate + bracketSize;
        
        double fLow = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, tLow);
        double fHigh = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, tHigh);
        
        // If estimate is already very accurate, return it
        double fEst = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, estimate);
        if (Math.Abs(fEst) < tolerance)
            return estimate;
        
        // If no sign change in bracket, the estimate is likely already optimal
        // Try to find a valid bracket by expanding
        if (fLow * fHigh > 0)
        {
            // Try expanding bracket
            for (int i = 0; i < 5; i++)
            {
                bracketSize *= 2;
                tLow = Math.Max(castDelay, estimate - bracketSize);
                tHigh = estimate + bracketSize;
                fLow = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, tLow);
                fHigh = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, tHigh);
                
                if (fLow * fHigh <= 0)
                    break;
            }
            
            // If still no valid bracket, return the estimate
            if (fLow * fHigh > 0)
                return estimate;
        }
        
        // Ensure fLow corresponds to negative value (before intercept)
        if (fLow > 0)
        {
            (tLow, tHigh) = (tHigh, tLow);
            (fLow, fHigh) = (fHigh, fLow);
        }
        
        // Bisection iterations
        for (int i = 0; i < maxIterations; i++)
        {
            double tMid = (tLow + tHigh) / 2;
            double fMid = EvaluateInterceptFunction(displacement, targetVelocity, skillshotSpeed, castDelay, tMid);
            
            // Check convergence
            if (Math.Abs(fMid) < tolerance || (tHigh - tLow) / 2 < tolerance)
                return tMid;
            
            // Narrow bracket
            if (fMid < 0)
            {
                tLow = tMid;
                fLow = fMid;
            }
            else
            {
                tHigh = tMid;
                fHigh = fMid;
            }
        }
        
        return (tLow + tHigh) / 2;
    }

    /// <summary>
    /// Solves for center-to-center intercept time using triple refinement:
    /// 1. Quadratic formula - O(1) initial estimate
    /// 2. Secant Method - Fast superlinear refinement to ~1e-12
    /// 3. Bisection - Guaranteed convergence to ~1e-15 (sub-pixel precision)
    /// 
    /// This triple-refinement approach provides:
    /// - Speed: Quadratic gives instant starting point
    /// - Efficiency: Secant rapidly improves precision
    /// - Robustness: Bisection guarantees final precision
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

        var displacement = targetPosition - casterPosition;
        var D = new Vector2D(displacement.X, displacement.Y);

        // Step 2: Refine with Secant Method (fast, ~1e-12)
        double secantRefined = RefineWithSecant(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            initialEstimate.Value,
            secantTolerance);

        // Step 3: Final refinement with Bisection (guaranteed, ~1e-15)
        double bisectionRefined = RefineWithBisection(
            D,
            targetVelocity,
            skillshotSpeed,
            castDelay,
            secantRefined,
            bracketSize: 0.001, // Small bracket since Secant already refined
            tolerance: bisectionTolerance);

        // Validate refined result is within range
        double maxTime = castDelay + skillshotRange / skillshotSpeed;
        if (bisectionRefined < castDelay || bisectionRefined > maxTime)
            return secantRefined; // Fall back to Secant if Bisection went out of bounds

        return bisectionRefined;
    }

    /// <summary>
    /// Ultimate solver for behind-target interception with full triple refinement.
    /// 
    /// Achieves PIXEL-PERFECT accuracy for "hit from behind by 1 pixel" strategy:
    /// 1. Quadratic formula - O(1) initial estimate
    /// 2. Secant Method - Fast refinement to ~1e-12
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
        double bisectionTolerance = 1e-15)
    {
        // Input validation
        ValidateInputs(skillshotSpeed, castDelay, skillshotRange);
        ValidateCollisionInputs(targetHitboxRadius, skillshotWidth);
        if (behindMargin < 0)
            throw new ArgumentException($"Behind margin cannot be negative, got {behindMargin}", nameof(behindMargin));

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;

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
        double behindDistance = effectiveRadius - behindMargin;
        if (behindDistance < 0) behindDistance = 0;
        
        Vector2D moveDirection = targetVelocity.Normalize();
        Vector2D behindOffset = moveDirection.Negate() * behindDistance;
        
        // Create virtual target position: offset BEHIND the real target
        Point2D virtualTargetPosition = targetPosition + behindOffset;
        
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
}
