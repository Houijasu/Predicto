using System.Runtime.CompilerServices;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

public sealed partial class InterceptSolver
{


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
}
