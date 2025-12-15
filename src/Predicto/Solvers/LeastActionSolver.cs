using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Solvers;

/// <summary>
/// Solvers based on the Principle of Least Action from physics.
/// 
/// The Least Action Principle states that physical systems evolve along paths that
/// minimize (or make stationary) the action integral S = ∫L dt.
/// 
/// Applied to skillshot prediction, we model:
/// 1. Inertia-Weighted: Target movement that resists sudden direction changes
/// 2. Dodge-Aware: Predicted dodge behavior minimizing effort while escaping threat
/// 3. Optimal Intercept: Aim point minimizing combined time + dodge difficulty
/// 
/// These provide alternative predictions compared to the baseline geometric approach.
/// </summary>
public static class LeastActionSolver
{
    #region Constants

    /// <summary>
    /// Inertia coefficient - higher values mean target resists direction changes more.
    /// Represents the "mass" in the Lagrangian kinetic term (1/2)m|v̇|²
    /// </summary>
    private const double InertiaCoefficient = 0.15;

    /// <summary>
    /// Threat potential strength - how strongly targets are repelled from danger zones.
    /// Represents the potential energy U(x,t) in the Lagrangian.
    /// </summary>
    private const double ThreatPotentialStrength = 200.0;

    /// <summary>
    /// Dodge effort coefficient - penalizes acceleration during dodge maneuvers.
    /// Higher values = lazier dodges, lower values = more aggressive dodges.
    /// </summary>
    private const double DodgeEffortCoefficient = 0.3;

    /// <summary>
    /// Time weight in optimal intercept calculation.
    /// </summary>
    private const double TimeWeight = 1.0;

    /// <summary>
    /// Dodge difficulty weight in optimal intercept calculation.
    /// </summary>
    private const double DodgeDifficultyWeight = 0.5;

    #endregion

    #region Result Types

    /// <summary>
    /// Result from a Least Action prediction.
    /// </summary>
    public readonly record struct LeastActionResult(
        Point2D AimPoint,
        Point2D PredictedTargetPosition,
        double InterceptTime,
        double ActionCost,
        string Method);

    #endregion

    #region 1. Inertia-Weighted Prediction

    /// <summary>
    /// Predicts target position accounting for inertia/momentum.
    /// 
    /// Uses the Euler-Lagrange equation with Lagrangian:
    /// L = (1/2)|ẋ|² - V(x)
    /// 
    /// where V(x) encodes desired positions. The solution gives smooth,
    /// physically plausible paths that resist sudden direction changes.
    /// 
    /// For a target with velocity V moving toward waypoint W, inertia causes
    /// the target to "overshoot" slightly on turns, creating a curved path.
    /// </summary>
    public static LeastActionResult? SolveInertiaWeighted(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (skillshotSpeed <= 0 || skillshotRange <= 0)
            return null;

        double targetSpeed = targetVelocity.Length;
        if (targetSpeed < Constants.Epsilon)
        {
            // Stationary target - use center aim
            return SolveStationary(casterPosition, targetPosition, skillshotSpeed, 
                castDelay, skillshotRange, "Inertia");
        }

        // Get baseline intercept time
        double? baselineTime = InterceptSolver.SolveInterceptTime(
            casterPosition, targetPosition, targetVelocity,
            skillshotSpeed, castDelay, skillshotRange);

        if (!baselineTime.HasValue)
            return null;

        double t = baselineTime.Value;

        // Standard predicted position (linear)
        var linearPredicted = targetPosition + targetVelocity * t;

        // Apply inertia correction: target maintains momentum during turns
        // Model as exponential decay of perpendicular velocity components
        // This simulates the "action cost" of changing direction
        var directionUnit = targetVelocity.Normalize();
        
        // Calculate "momentum carry" - how much of current velocity persists
        double momentumFactor = Math.Exp(-InertiaCoefficient * t);
        
        // The inertia effect pushes the predicted position slightly in the 
        // direction of current velocity (overshooting turns)
        var inertiaBias = directionUnit * (targetSpeed * InertiaCoefficient * t * momentumFactor);
        
        var inertiaWeightedPosition = linearPredicted + inertiaBias;

        // Calculate aim point (behind-target style)
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        var aimPoint = CalculateBehindTargetAim(
            casterPosition, inertiaWeightedPosition, targetVelocity, effectiveRadius);

        // Check range
        double aimDistance = (aimPoint - casterPosition).Length;
        if (aimDistance > skillshotRange)
            return null;

        // Calculate action cost (lower is better)
        double actionCost = CalculateInertiaActionCost(targetVelocity, t, inertiaBias);

        return new LeastActionResult(
            aimPoint, inertiaWeightedPosition, t, actionCost, "Inertia-Weighted");
    }

    /// <summary>
    /// Inertia-weighted prediction for path-following targets.
    /// </summary>
    public static LeastActionResult? SolveInertiaWeightedWithPath(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (skillshotSpeed <= 0 || skillshotRange <= 0 || path.Speed < Constants.Epsilon)
            return null;

        // Get baseline intercept using path
        var baselineResult = InterceptSolver.SolvePathBehindTargetIntercept(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange, 1.0);

        if (!baselineResult.HasValue)
            return null;

        double t = baselineResult.Value.InterceptTime;

        // Get the linear path position
        var linearPredicted = path.GetPositionAtTime(t);
        var currentVelocity = path.GetCurrentVelocity();
        
        // Find velocity at intercept time for inertia calculation
        // This requires looking ahead on the path
        var futureVelocity = path.GetVelocityAtTime(t);
        
        // Inertia correction: blend between current and future velocity direction
        // Targets "lag" behind direction changes due to momentum
        var velocityBlend = currentVelocity * (1 - Math.Exp(-InertiaCoefficient * t)) 
                          + futureVelocity * Math.Exp(-InertiaCoefficient * t);
        
        // Apply inertia bias - the target overshoots toward current velocity
        double momentumCarry = InertiaCoefficient * path.Speed * t * Math.Exp(-t / 2.0);
        var inertiaBias = currentVelocity.Normalize() * momentumCarry;
        
        var inertiaWeightedPosition = linearPredicted + inertiaBias;

        // Calculate aim point
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        var aimPoint = CalculateBehindTargetAim(
            casterPosition, inertiaWeightedPosition, velocityBlend, effectiveRadius);

        // Check range
        double aimDistance = (aimPoint - casterPosition).Length;
        if (aimDistance > skillshotRange)
            return null;

        double actionCost = momentumCarry * momentumCarry; // Squared momentum as action

        return new LeastActionResult(
            aimPoint, inertiaWeightedPosition, t, actionCost, "Inertia-Weighted");
    }

    #endregion

    #region 2. Dodge-Aware Prediction

    /// <summary>
    /// Predicts where target will dodge based on Least Action principle.
    /// 
    /// The target minimizes the action functional:
    /// S = ∫ [(1/2)m|v̇|² + U(x,t)] dt
    /// 
    /// where:
    /// - (1/2)m|v̇|² = effort to accelerate (dodge cost)
    /// - U(x,t) = threat potential from incoming skillshot
    /// 
    /// The result is the path that escapes the threat with minimum effort.
    /// </summary>
    public static LeastActionResult? SolveDodgeAware(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (skillshotSpeed <= 0 || skillshotRange <= 0)
            return null;

        double targetSpeed = targetVelocity.Length;
        if (targetSpeed < Constants.Epsilon)
        {
            // Stationary target - predict perpendicular dodge
            return SolveDodgeFromStationary(casterPosition, targetPosition, 
                skillshotSpeed, castDelay, targetHitboxRadius, skillshotWidth, skillshotRange);
        }

        // Get baseline intercept
        double? baselineTime = InterceptSolver.SolveInterceptTime(
            casterPosition, targetPosition, targetVelocity,
            skillshotSpeed, castDelay, skillshotRange);

        if (!baselineTime.HasValue)
            return null;

        double t = baselineTime.Value;
        var predictedPos = targetPosition + targetVelocity * t;

        // Calculate threat gradient (direction skillshot approaches from)
        var threatDirection = (predictedPos - casterPosition).Normalize();

        // Optimal dodge is perpendicular to threat, in direction that requires
        // least change from current velocity (minimum action)
        var perpendicular1 = new Vector2D(-threatDirection.Y, threatDirection.X);
        var perpendicular2 = new Vector2D(threatDirection.Y, -threatDirection.X);

        // Choose dodge direction with smaller angle to current velocity
        // This minimizes the acceleration (action) required
        double dot1 = perpendicular1.DotProduct(targetVelocity);
        double dot2 = perpendicular2.DotProduct(targetVelocity);
        var dodgeDirection = dot1 > dot2 ? perpendicular1 : perpendicular2;

        // Calculate dodge magnitude based on:
        // 1. Time available to react (less time = smaller dodge)
        // 2. Threat intensity (faster skillshot = larger dodge)
        // 3. Effort cost (proportional to acceleration squared)
        double reactionTime = Math.Max(0, t - Constants.HumanReactionTime);
        double threatIntensity = skillshotSpeed / 1000.0; // Normalized
        double maxDodgeDistance = reactionTime * targetSpeed * 0.5; // Can redirect half velocity

        // The "Least Action" dodge: just enough to escape, no more
        double escapeRadius = targetHitboxRadius + skillshotWidth / 2 + 10; // 10 unit buffer
        double dodgeMagnitude = Math.Min(escapeRadius * threatIntensity, maxDodgeDistance);

        // Apply effort penalty (higher coefficient = smaller dodge)
        dodgeMagnitude *= (1.0 / (1.0 + DodgeEffortCoefficient));

        var dodgeOffset = dodgeDirection * dodgeMagnitude;
        var dodgePredictedPosition = predictedPos + dodgeOffset;

        // Aim ahead of where they'll dodge to
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        var aimPoint = CalculateBehindTargetAim(
            casterPosition, dodgePredictedPosition, targetVelocity + dodgeDirection * dodgeMagnitude / t, effectiveRadius);

        // Check range
        double aimDistance = (aimPoint - casterPosition).Length;
        if (aimDistance > skillshotRange)
            return null;

        // Action cost: acceleration squared integrated over time
        double accelerationMag = dodgeMagnitude / (reactionTime * reactionTime + 0.01);
        double actionCost = accelerationMag * accelerationMag * reactionTime;

        return new LeastActionResult(
            aimPoint, dodgePredictedPosition, t, actionCost, "Dodge-Aware");
    }

    /// <summary>
    /// Dodge prediction for path-following targets.
    /// </summary>
    public static LeastActionResult? SolveDodgeAwareWithPath(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (skillshotSpeed <= 0 || skillshotRange <= 0 || path.Speed < Constants.Epsilon)
            return null;

        var currentVelocity = path.GetCurrentVelocity();
        
        // Get baseline intercept
        var baselineResult = InterceptSolver.SolvePathBehindTargetIntercept(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange, 1.0);

        if (!baselineResult.HasValue)
            return null;

        double t = baselineResult.Value.InterceptTime;
        var predictedPos = path.GetPositionAtTime(t);

        // Threat direction
        var threatDirection = (predictedPos - casterPosition).Normalize();

        // Perpendicular dodge options
        var perpendicular1 = new Vector2D(-threatDirection.Y, threatDirection.X);
        var perpendicular2 = new Vector2D(threatDirection.Y, -threatDirection.X);

        // Choose direction closer to current velocity
        double dot1 = perpendicular1.DotProduct(currentVelocity);
        double dot2 = perpendicular2.DotProduct(currentVelocity);
        var dodgeDirection = dot1 > dot2 ? perpendicular1 : perpendicular2;

        // Calculate dodge with path awareness
        double reactionTime = Math.Max(0, t - Constants.HumanReactionTime);
        double remainingPathTime = path.GetRemainingPathTime();
        
        // Near path end, less likely to dodge (committed to destination)
        double pathCommitment = Math.Max(0, 1.0 - remainingPathTime / 2.0);
        double dodgeLikelihood = 1.0 - pathCommitment * 0.7;

        double threatIntensity = skillshotSpeed / 1000.0;
        double maxDodgeDistance = reactionTime * path.Speed * 0.5 * dodgeLikelihood;
        double escapeRadius = targetHitboxRadius + skillshotWidth / 2 + 10;
        double dodgeMagnitude = Math.Min(escapeRadius * threatIntensity, maxDodgeDistance);
        dodgeMagnitude *= (1.0 / (1.0 + DodgeEffortCoefficient));

        var dodgeOffset = dodgeDirection * dodgeMagnitude;
        var dodgePredictedPosition = predictedPos + dodgeOffset;

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        var aimPoint = CalculateBehindTargetAim(
            casterPosition, dodgePredictedPosition, 
            currentVelocity + dodgeDirection * dodgeMagnitude / t, effectiveRadius);

        double aimDistance = (aimPoint - casterPosition).Length;
        if (aimDistance > skillshotRange)
            return null;

        double accelerationMag = dodgeMagnitude / (reactionTime * reactionTime + 0.01);
        double actionCost = accelerationMag * accelerationMag * reactionTime;

        return new LeastActionResult(
            aimPoint, dodgePredictedPosition, t, actionCost, "Dodge-Aware");
    }

    #endregion

    #region 3. Optimal Intercept (Combined Cost Minimization)

    /// <summary>
    /// Finds the aim point that minimizes combined cost:
    /// Cost = TimeWeight × InterceptTime + DodgeDifficultyWeight × (1/DodgeDifficulty)
    /// 
    /// This is a variational approach that finds the "path of least action"
    /// for the skillshot itself, balancing speed vs. difficulty to dodge.
    /// </summary>
    public static LeastActionResult? SolveOptimalIntercept(
        Point2D casterPosition,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (skillshotSpeed <= 0 || skillshotRange <= 0)
            return null;

        double targetSpeed = targetVelocity.Length;
        if (targetSpeed < Constants.Epsilon)
        {
            return SolveStationary(casterPosition, targetPosition, skillshotSpeed,
                castDelay, skillshotRange, "Optimal");
        }

        // Get baseline intercept time
        double? baselineTime = InterceptSolver.SolveInterceptTime(
            casterPosition, targetPosition, targetVelocity,
            skillshotSpeed, castDelay, skillshotRange);

        if (!baselineTime.HasValue)
            return null;

        double baseT = baselineTime.Value;
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;

        // Sample multiple aim strategies and find minimum action
        var strategies = new (string Name, Func<double, Point2D> GetAim)[]
        {
            ("Center", t => targetPosition + targetVelocity * t),
            ("Behind", t => {
                var pos = targetPosition + targetVelocity * t;
                return CalculateBehindTargetAim(casterPosition, pos, targetVelocity, effectiveRadius);
            }),
            ("Lead", t => {
                var pos = targetPosition + targetVelocity * t;
                // Aim ahead of target (opposite of behind)
                var moveDir = targetVelocity.Normalize();
                return pos + moveDir * effectiveRadius * 0.5;
            }),
            ("Perpendicular+", t => {
                var pos = targetPosition + targetVelocity * t;
                var moveDir = targetVelocity.Normalize();
                var perp = new Vector2D(-moveDir.Y, moveDir.X);
                return pos + perp * effectiveRadius * 0.3;
            }),
            ("Perpendicular-", t => {
                var pos = targetPosition + targetVelocity * t;
                var moveDir = targetVelocity.Normalize();
                var perp = new Vector2D(moveDir.Y, -moveDir.X);
                return pos + perp * effectiveRadius * 0.3;
            }),
        };

        LeastActionResult? bestResult = null;
        double bestCost = double.MaxValue;

        foreach (var (strategyName, getAim) in strategies)
        {
            // Find intercept time for this strategy
            var aimPoint = getAim(baseT);
            
            // Validate aim is in range
            double aimDistance = (aimPoint - casterPosition).Length;
            if (aimDistance > skillshotRange)
                continue;

            // Calculate flight time
            double flightTime = aimDistance / skillshotSpeed;
            double totalTime = castDelay + flightTime;

            // Calculate dodge difficulty (higher = harder to dodge)
            double dodgeDifficulty = CalculateDodgeDifficulty(
                casterPosition, aimPoint, targetPosition, targetVelocity, 
                totalTime, targetHitboxRadius, skillshotWidth);

            // Combined action cost
            double cost = TimeWeight * totalTime + DodgeDifficultyWeight / (dodgeDifficulty + 0.1);

            if (cost < bestCost)
            {
                bestCost = cost;
                var predictedPos = targetPosition + targetVelocity * totalTime;
                bestResult = new LeastActionResult(
                    aimPoint, predictedPos, totalTime, cost, $"Optimal-{strategyName}");
            }
        }

        return bestResult;
    }

    /// <summary>
    /// Optimal intercept for path-following targets.
    /// </summary>
    public static LeastActionResult? SolveOptimalInterceptWithPath(
        Point2D casterPosition,
        TargetPath path,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        if (skillshotSpeed <= 0 || skillshotRange <= 0 || path.Speed < Constants.Epsilon)
            return null;

        // Get baseline intercept
        var baselineResult = InterceptSolver.SolvePathBehindTargetIntercept(
            casterPosition, path, skillshotSpeed, castDelay,
            targetHitboxRadius, skillshotWidth, skillshotRange, 1.0);

        if (!baselineResult.HasValue)
            return null;

        double baseT = baselineResult.Value.InterceptTime;
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        var currentVelocity = path.GetCurrentVelocity();

        // Sample strategies
        var strategies = new (string Name, Func<double, Point2D> GetAim)[]
        {
            ("Center", t => path.GetPositionAtTime(t)),
            ("Behind", t => {
                var pos = path.GetPositionAtTime(t);
                var vel = path.GetVelocityAtTime(t);
                return CalculateBehindTargetAim(casterPosition, pos, vel, effectiveRadius);
            }),
            ("Lead", t => {
                var pos = path.GetPositionAtTime(t);
                var vel = path.GetVelocityAtTime(t);
                if (vel.Length < Constants.Epsilon) return pos;
                return pos + vel.Normalize() * effectiveRadius * 0.5;
            }),
        };

        LeastActionResult? bestResult = null;
        double bestCost = double.MaxValue;

        foreach (var (strategyName, getAim) in strategies)
        {
            var aimPoint = getAim(baseT);
            
            double aimDistance = (aimPoint - casterPosition).Length;
            if (aimDistance > skillshotRange)
                continue;

            double flightTime = aimDistance / skillshotSpeed;
            double totalTime = castDelay + flightTime;

            double dodgeDifficulty = CalculateDodgeDifficulty(
                casterPosition, aimPoint, path.CurrentPosition, currentVelocity,
                totalTime, targetHitboxRadius, skillshotWidth);

            double cost = TimeWeight * totalTime + DodgeDifficultyWeight / (dodgeDifficulty + 0.1);

            if (cost < bestCost)
            {
                bestCost = cost;
                var predictedPos = path.GetPositionAtTime(totalTime);
                bestResult = new LeastActionResult(
                    aimPoint, predictedPos, totalTime, cost, $"Optimal-{strategyName}");
            }
        }

        return bestResult;
    }

    #endregion

    #region Helper Methods

    private static LeastActionResult? SolveStationary(
        Point2D casterPosition,
        Point2D targetPosition,
        double skillshotSpeed,
        double castDelay,
        double skillshotRange,
        string method)
    {
        double distance = (targetPosition - casterPosition).Length;
        if (distance > skillshotRange)
            return null;

        double flightTime = distance / skillshotSpeed;
        double totalTime = castDelay + flightTime;

        return new LeastActionResult(
            targetPosition, targetPosition, totalTime, 0, method);
    }

    private static LeastActionResult? SolveDodgeFromStationary(
        Point2D casterPosition,
        Point2D targetPosition,
        double skillshotSpeed,
        double castDelay,
        double targetHitboxRadius,
        double skillshotWidth,
        double skillshotRange)
    {
        double distance = (targetPosition - casterPosition).Length;
        if (distance > skillshotRange)
            return null;

        // For stationary target, predict dodge perpendicular to skillshot
        var threatDir = (targetPosition - casterPosition).Normalize();
        var dodgeDir = new Vector2D(-threatDir.Y, threatDir.X); // Perpendicular

        // Minimal dodge - just escape the collision zone
        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        double dodgeDistance = effectiveRadius * 0.3; // Predict small dodge

        var dodgePosition = targetPosition + dodgeDir * dodgeDistance;
        double flightTime = (dodgePosition - casterPosition).Length / skillshotSpeed;
        double totalTime = castDelay + flightTime;

        return new LeastActionResult(
            dodgePosition, dodgePosition, totalTime, dodgeDistance * dodgeDistance, "Dodge-Aware");
    }

    private static Point2D CalculateBehindTargetAim(
        Point2D casterPosition,
        Point2D predictedTargetPosition,
        Vector2D targetVelocity,
        double effectiveRadius)
    {
        if (targetVelocity.Length < Constants.Epsilon)
            return predictedTargetPosition;

        // Behind = opposite to movement direction
        var moveDirection = targetVelocity.Normalize();
        var behindOffset = moveDirection * (-effectiveRadius * 0.9); // Slightly inside
        
        return predictedTargetPosition + behindOffset;
    }

    private static double CalculateInertiaActionCost(
        Vector2D velocity,
        double time,
        Vector2D inertiaBias)
    {
        // Action = ∫ L dt ≈ kinetic energy × time + potential from bias
        double kineticEnergy = 0.5 * velocity.Length * velocity.Length;
        double biasCost = inertiaBias.Length * inertiaBias.Length;
        
        return kineticEnergy * time + biasCost;
    }

    private static double CalculateDodgeDifficulty(
        Point2D casterPosition,
        Point2D aimPoint,
        Point2D targetPosition,
        Vector2D targetVelocity,
        double flightTime,
        double targetHitboxRadius,
        double skillshotWidth)
    {
        // Factors that make dodging difficult:
        // 1. Skillshot arrives from behind (target can't see it)
        // 2. Short flight time (less reaction time)
        // 3. Wide skillshot (more area to escape)
        // 4. Target moving toward the aim point

        double effectiveRadius = targetHitboxRadius + skillshotWidth / 2;
        
        // Direction factors
        var skillshotDir = (aimPoint - casterPosition).Normalize();
        var moveDir = targetVelocity.Length > Constants.Epsilon 
            ? targetVelocity.Normalize() 
            : new Vector2D(0, 0);

        // Dot product: +1 = from behind, -1 = from front
        double approachAngle = skillshotDir.DotProduct(moveDir);
        double behindFactor = (approachAngle + 1) / 2; // 0 to 1

        // Time factor: less time = harder to dodge
        double timeFactor = 1.0 / (flightTime + 0.1);

        // Width factor: wider = harder
        double widthFactor = skillshotWidth / 100.0;

        // Combined difficulty score
        return behindFactor * 0.4 + timeFactor * 0.4 + widthFactor * 0.2;
    }

    #endregion
}
