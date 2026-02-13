using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

public sealed partial class InterceptSolver
{

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
        int segmentCount = path.SegmentCount;

        for (int segIdx = 0; segIdx < segmentCount; segIdx++)
        {
            var segment = path.GetSegment(segIdx);

            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
                continue;

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;

            Vector2D segmentVelocity = segment.Direction * path.Speed;

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
        int segmentCount = path.SegmentCount;

        for (int segIdx = 0; segIdx < segmentCount; segIdx++)
        {
            var segment = path.GetSegment(segIdx);

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
        int segmentCount = path.SegmentCount;

        for (int segIdx = 0; segIdx < segmentCount; segIdx++)
        {
            var segment = path.GetSegment(segIdx);

            double segmentLength = segment.Length;
            if (segmentLength < Constants.Epsilon)
                continue;

            double segmentDuration = segmentLength / path.Speed;
            double segmentEndTime = segmentStartTime + segmentDuration;
            Vector2D segmentVelocity = segment.Direction * path.Speed;

            if (!IsSegmentReachable(casterPosition, segment.Start, segmentVelocity, segmentDuration,
                skillshotSpeed, castDelay, skillshotRange, effectiveRadius, segmentStartTime))
            {
                segmentStartTime = segmentEndTime;
                continue;
            }

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
        int segmentCount = path.SegmentCount;

        for (int segIdx = 0; segIdx < segmentCount; segIdx++)
        {
            var segment = path.GetSegment(segIdx);

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

                Point2D predictedPos = path.GetPositionAtTime(interceptTime);
                Vector2D velocityAtIntercept = path.GetVelocityAtTime(interceptTime);

                Vector2D moveDirectionAtIntercept = velocityAtIntercept.Length > Constants.Epsilon
                    ? velocityAtIntercept.Normalize()
                    : moveDirection;

                Point2D aimPoint = predictedPos + (moveDirectionAtIntercept.Negate() * behindDistance);

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
}
