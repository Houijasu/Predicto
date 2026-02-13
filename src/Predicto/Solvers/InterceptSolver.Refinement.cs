using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

public sealed partial class InterceptSolver
{
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
}
