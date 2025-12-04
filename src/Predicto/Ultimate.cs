using MathNet.Spatial.Euclidean;
using Predicto.Models;
using Predicto.Solvers;

namespace Predicto;

/// <summary>
/// Ultimate prediction engine - state-of-the-art implementation.
/// 
/// Uses BEHIND-TARGET strategy: aims BEHIND the target relative to their movement.
/// If target is moving north, we aim south of their predicted position.
/// The projectile arrives from behind - target cannot see it coming.
/// 
/// Example:
///   Target moving: ↑ (north)
///   Predicted position: ●
///   Aim point (1px south): ○ ← projectile hits here, from behind
/// 
/// Why behind-target is optimal:
/// - Target cannot see projectile approaching from their movement direction
/// - Must completely REVERSE direction to dodge
/// - Psychologically much harder to react to
/// - Projectile "catches up" from behind
/// 
/// Technical features - Triple Refinement for PIXEL-PERFECT accuracy:
/// 1. Quadratic formula - O(1) initial estimate
/// 2. Secant Method - Fast superlinear refinement to ~1e-12
/// 3. Bisection - Guaranteed convergence to ~1e-15 (sub-pixel precision)
/// 
/// Performance optimizations:
/// - Early exit for obvious misses (out of range + moving away)
/// - Sanity checks for extreme velocities
/// - Adaptive refinement: skip triple refinement for easy cases
/// - Adaptive margin: dynamic behind-margin based on target speed/distance
/// 
/// This ensures the projectile hits EXACTLY 1 pixel inside the collision zone
/// from behind, achieving the intended "hit from behind by 1 pixel" strategy.
/// </summary>
public sealed class Ultimate : IPrediction
{
    /// <inheritdoc />
    public PredictionResult Predict(PredictionInput input)
    {
        // === Input Validation ===
        if (input.Skillshot.Speed <= Constants.Epsilon)
            return new PredictionResult.Unreachable("Skillshot speed must be positive");

        if (input.Skillshot.Range <= Constants.Epsilon)
            return new PredictionResult.Unreachable("Skillshot range must be positive");

        // === Use Path-Based Prediction if Available ===
        if (input.HasPath)
        {
            return PredictWithPath(input);
        }

        // === Velocity-Based Prediction (original logic) ===
        return PredictWithVelocity(input);
    }

    /// <summary>
    /// Path-based prediction for multi-waypoint target movement.
    /// Iterates through path segments to find optimal interception point.
    /// </summary>
    private PredictionResult PredictWithPath(PredictionInput input)
    {
        var path = input.TargetPath!;

        // === Velocity Sanity Checks ===
        if (path.Speed > Constants.MaxReasonableVelocity)
            return new PredictionResult.Unreachable("Target velocity exceeds reasonable bounds");

        if (input.Skillshot.Speed > Constants.MaxReasonableSkillshotSpeed)
            return new PredictionResult.Unreachable("Skillshot speed exceeds reasonable bounds");

        // === Geometry Setup (cache commonly used values) ===
        var currentVelocity = path.GetCurrentVelocity();
        var displacement = path.CurrentPosition - input.CasterPosition;
        double distance = displacement.Length;
        double effectiveRadius = input.TargetHitboxRadius + input.Skillshot.Width / 2;

        // === Early Out: Target clearly out of range and moving away ===
        if (distance > input.Skillshot.Range + effectiveRadius)
        {
            double dotProduct = displacement.DotProduct(currentVelocity);
            if (dotProduct > 0) // Moving away
            {
                double maxReachTime = (input.Skillshot.Range + effectiveRadius) / input.Skillshot.Speed + input.Skillshot.Delay;
                var futurePosition = path.GetPositionAtTime(maxReachTime);
                double futureDistance = (futurePosition - input.CasterPosition).Length;

                if (futureDistance > input.Skillshot.Range + effectiveRadius)
                    return new PredictionResult.OutOfRange(distance - effectiveRadius, input.Skillshot.Range);
            }
        }

        // === Stationary Target ===
        if (path.Speed < Constants.MinVelocity)
        {
            return PredictStationary(input, distance, effectiveRadius);
        }

        // === Calculate Adaptive Margin ===
        double adaptiveMargin = CalculateAdaptiveMargin(effectiveRadius);

        // === Solve Path-Based BEHIND-TARGET Interception ===
        var result = InterceptSolver.SolvePathBehindTargetIntercept(
            input.CasterPosition,
            path,
            input.Skillshot.Speed,
            input.Skillshot.Delay,
            input.TargetHitboxRadius,
            input.Skillshot.Width,
            input.Skillshot.Range,
            behindMargin: adaptiveMargin);

        if (!result.HasValue)
        {
            return new PredictionResult.Unreachable("No valid interception solution exists along target path");
        }

        var interceptResult = result.Value;

        // === Range Validation ===
        // Use epsilon tolerance to prevent false out-of-range due to floating-point precision
        double flightTime = Math.Max(0, interceptResult.InterceptTime - input.Skillshot.Delay);
        double travelDistance = input.Skillshot.Speed * flightTime;

        if (travelDistance > input.Skillshot.Range + Constants.RangeTolerance)
        {
            return new PredictionResult.OutOfRange(travelDistance, input.Skillshot.Range);
        }

        // === Calculate Enhanced Confidence ===
        double confidence = CalculateEnhancedConfidence(
            interceptResult.InterceptTime,
            distance,
            path.Speed,
            input.Skillshot.Speed,
            input.Skillshot.Range,
            input.Skillshot.Delay,
            displacement,
            currentVelocity);

        // === Confidence Penalty for Late Waypoints ===
        // Predictions on later waypoints are less reliable (more time for target to change path)
        if (interceptResult.WaypointIndex > path.CurrentWaypointIndex)
        {
            int waypointDelta = interceptResult.WaypointIndex - path.CurrentWaypointIndex;
            double waypointPenalty = Math.Pow(0.5, waypointDelta); // 50% reduction per waypoint
            confidence *= waypointPenalty;
        }

        // === Confidence Penalty for Segment Distance ===
        // Even on the current segment, predictions far along the path are less reliable
        // Calculate how far along the path the intercept occurs
        double remainingPathTime = path.GetRemainingPathTime();
        if (remainingPathTime > Constants.Epsilon)
        {
            // Clamp segmentProgress to [0, 1] to prevent negative penalties
            // when interceptTime > remainingPathTime (target stopped at final waypoint)
            double segmentProgress = Math.Clamp(interceptResult.InterceptTime / remainingPathTime, 0, 1);
            // Apply mild penalty for predictions deep into the path (linear decay)
            // At 100% path progress: 0.5x confidence, at 50%: 0.75x
            double segmentPenalty = 1 - 0.5 * segmentProgress;
            confidence *= segmentPenalty;
        }

        return new PredictionResult.Hit(
            CastPosition: interceptResult.AimPoint,
            PredictedTargetPosition: interceptResult.PredictedTargetPosition,
            InterceptTime: interceptResult.InterceptTime,
            Confidence: Math.Clamp(confidence, Constants.Epsilon, 1.0));
    }

    /// <summary>
    /// Velocity-based prediction for simple linear movement.
    /// </summary>
    private PredictionResult PredictWithVelocity(PredictionInput input)
    {
        // === Velocity Sanity Checks ===
        double targetSpeed = input.TargetVelocity.Length;
        if (targetSpeed > Constants.MaxReasonableVelocity)
            return new PredictionResult.Unreachable("Target velocity exceeds reasonable bounds");

        if (input.Skillshot.Speed > Constants.MaxReasonableSkillshotSpeed)
            return new PredictionResult.Unreachable("Skillshot speed exceeds reasonable bounds");

        // === Geometry Setup (cache commonly used values) ===
        var displacement = input.TargetPosition - input.CasterPosition;
        double distance = displacement.Length;
        double effectiveRadius = input.TargetHitboxRadius + input.Skillshot.Width / 2;

        // === Early Out: Target clearly out of range ===
        // If target is beyond max possible reach and moving away, no solution exists
        if (distance > input.Skillshot.Range + effectiveRadius)
        {
            double dotProduct = displacement.DotProduct(input.TargetVelocity);
            bool movingAway = dotProduct > 0;

            if (movingAway)
            {
                // Calculate if target can possibly be caught using accurate vector math
                // Max time = (range + effectiveRadius) / skillshotSpeed + delay
                double maxReachTime = (input.Skillshot.Range + effectiveRadius) / input.Skillshot.Speed + input.Skillshot.Delay;
                
                // Compute exact future position using vector math instead of magnitude approximation
                // This handles angled movement correctly
                var futureTargetPos = input.TargetPosition + input.TargetVelocity * maxReachTime;
                double futureDistance = (futureTargetPos - input.CasterPosition).Length;

                if (futureDistance > input.Skillshot.Range + effectiveRadius)
                    return new PredictionResult.OutOfRange(distance - effectiveRadius, input.Skillshot.Range);
            }
        }

        // === Early Out: Target too far and too fast to catch ===
        // If target speed > skillshot speed and moving away, can never catch
        if (targetSpeed >= input.Skillshot.Speed)
        {
            double dotProduct = displacement.DotProduct(input.TargetVelocity);
            if (dotProduct > 0 && distance > effectiveRadius)
            {
                // Target outrunning skillshot
                return new PredictionResult.Unreachable("Target is outrunning the skillshot");
            }
        }

        // === Stationary Target Optimization ===
        if (targetSpeed < Constants.MinVelocity)
        {
            return PredictStationary(input, distance, effectiveRadius);
        }

        // === Calculate Adaptive Margin ===
        // Use half of spell width as base margin for more reliable hits
        double adaptiveMargin = CalculateAdaptiveMargin(effectiveRadius);

        // === Determine Refinement Strategy ===
        // Easy cases: close range, slow target, or target moving toward caster
        bool isEasyCase = IsEasyCase(distance, targetSpeed, displacement, input.TargetVelocity, input.Skillshot.Speed);

        // === Solve BEHIND-TARGET Interception ===
        (Point2D AimPoint, Point2D PredictedTargetPosition, double InterceptTime)? result;

        if (isEasyCase)
        {
            // Use Secant refinement only for easy cases (faster)
            result = InterceptSolver.SolveBehindTargetWithSecantRefinement(
                input.CasterPosition,
                input.TargetPosition,
                input.TargetVelocity,
                input.Skillshot.Speed,
                input.Skillshot.Delay,
                input.TargetHitboxRadius,
                input.Skillshot.Width,
                input.Skillshot.Range,
                behindMargin: adaptiveMargin,
                tolerance: 1e-10);
        }
        else
        {
            // Full Triple Refinement for difficult cases (pixel-perfect precision)
            // 1. Quadratic formula for O(1) initial estimate
            // 2. Secant Method refinement for ~1e-12 precision
            // 3. Bisection for guaranteed ~1e-15 sub-pixel precision
            result = InterceptSolver.SolveBehindTargetWithFullRefinement(
                input.CasterPosition,
                input.TargetPosition,
                input.TargetVelocity,
                input.Skillshot.Speed,
                input.Skillshot.Delay,
                input.TargetHitboxRadius,
                input.Skillshot.Width,
                input.Skillshot.Range,
                behindMargin: adaptiveMargin,
                secantTolerance: 1e-12,
                bisectionTolerance: 1e-15);
        }

        if (!result.HasValue)
        {
            return new PredictionResult.Unreachable("No valid interception solution exists");
        }

        var (aimPoint, predictedTargetPos, interceptTime) = result.Value;

        // === Range Validation ===
        // Use epsilon tolerance to prevent false out-of-range due to floating-point precision
        double flightTime = Math.Max(0, interceptTime - input.Skillshot.Delay);
        double travelDistance = input.Skillshot.Speed * flightTime;

        if (travelDistance > input.Skillshot.Range + Constants.RangeTolerance)
        {
            return new PredictionResult.OutOfRange(travelDistance, input.Skillshot.Range);
        }

        // === Calculate Enhanced Confidence ===
        double confidence = CalculateEnhancedConfidence(
            interceptTime,
            distance,
            targetSpeed,
            input.Skillshot.Speed,
            input.Skillshot.Range,
            input.Skillshot.Delay,
            displacement,
            input.TargetVelocity);

        // === Return Result ===
        // CastPosition = where to aim (behind target)
        // PredictedTargetPosition = where target will actually be
        return new PredictionResult.Hit(
            CastPosition: aimPoint,
            PredictedTargetPosition: predictedTargetPos,
            InterceptTime: interceptTime,
            Confidence: confidence);
    }

    /// <summary>
    /// Determines if this is an "easy" prediction case that doesn't need full refinement.
    /// 
    /// Easy cases use only Secant refinement (faster).
    /// Hard cases use full Triple Refinement (Quadratic → Secant → Bisection).
    /// 
    /// Criteria for easy case:
    /// 1. Target moves less than half a unit per server tick (nearly stationary)
    /// 2. Target moving significantly toward caster (easier geometry)
    /// </summary>
    private static bool IsEasyCase(
        double distance,
        double targetSpeed,
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed)
    {
        // Criterion 1: Target moves negligibly per tick
        // If target moves < 0.5 units per tick, the prediction is trivial
        // Formula: targetSpeed * tickDuration < 0.5
        double movementPerTick = targetSpeed * Constants.TickDuration;
        if (movementPerTick < 0.5)
            return true;

        // Criterion 2: Target moving toward caster (easier to hit)
        // Dot product < 0 means moving toward caster
        // We require significant approach: |cos(angle)| > 0.5 (within 60° of direct approach)
        double dotProduct = displacement.DotProduct(targetVelocity);
        if (dotProduct < 0 && distance > Constants.Epsilon)
        {
            double cosAngle = Math.Abs(dotProduct) / (targetSpeed * distance);
            if (cosAngle > 0.5)
                return true;
        }

        return false;
    }

    /// <summary>
    /// Calculates adaptive margin for the "behind target" strategy.
    /// 
    /// For linear skillshots, we aim at the trailing edge with a small margin for maximum
    /// aggressiveness. LoL uses swept collision detection between ticks, so edge hits register.
    /// 
    /// The margin uses a continuous function to avoid discontinuities at threshold boundaries.
    /// For very small effective radii, the margin scales proportionally to maintain valid geometry.
    /// For normal radii, we use a minimal but non-zero margin (1 game unit) for reliability.
    /// 
    /// Note: The InterceptSolver also clamps margin to ensure it's less than effectiveRadius,
    /// but we apply defensive clamping here as well for clarity.
    /// </summary>
    /// <param name="effectiveRadius">Combined collision radius (target hitbox + skillshot width/2)</param>
    /// <returns>Margin in game units, guaranteed to be less than effectiveRadius</returns>
    private static double CalculateAdaptiveMargin(double effectiveRadius)
    {
        // Handle degenerate case
        if (effectiveRadius <= Constants.Epsilon)
            return 0;

        double margin;

        // Use proportional margin (50% of effective radius) for small radii,
        // transitioning smoothly to a fixed margin for normal gameplay radii.
        // The threshold ensures smooth transition: at SmallRadiusThreshold, both formulas give the same value.
        if (effectiveRadius <= Constants.SmallRadiusThreshold)
        {
            margin = effectiveRadius * 0.5;
        }
        else
        {
            // Use TrailingEdgeMargin (1 game unit) for normal gameplay scenarios
            // This is aggressive (barely inside collision zone) while maintaining reliability
            margin = Constants.TrailingEdgeMargin;
        }

        // Defensive clamp: margin must be strictly less than effectiveRadius
        // This ensures the aim point is always inside the collision zone
        return Math.Min(margin, effectiveRadius - Constants.Epsilon);
    }

    /// <summary>
    /// Enhanced confidence calculation that factors in approach angle, movement patterns,
    /// and human reaction time limitations.
    /// </summary>
    private static double CalculateEnhancedConfidence(
        double interceptTime,
        double distance,
        double targetSpeed,
        double skillshotSpeed,
        double skillshotRange,
        double castDelay,
        Vector2D displacement,
        Vector2D targetVelocity)
    {
        // Start with base confidence calculation
        double baseConfidence = InterceptSolver.CalculateConfidence(
            interceptTime,
            distance,
            targetSpeed,
            skillshotSpeed,
            skillshotRange,
            castDelay);

        // === Approach Angle Factor ===
        // Targets moving perpendicular to skillshot path are harder to predict
        // Use cosine directly: 1 = parallel (predictable), 0 = perpendicular (unpredictable)
        double approachAngleFactor = 1.0;
        if (targetSpeed > Constants.MinVelocity && displacement.Length > Constants.Epsilon)
        {
            double displacementLength = displacement.Length;
            double velocityLength = targetVelocity.Length;

            if (displacementLength > Constants.Epsilon && velocityLength > Constants.Epsilon)
            {
                double dotProduct = displacement.DotProduct(targetVelocity);
                double cosAngle = Math.Abs(dotProduct / (displacementLength * velocityLength));

                // Scale confidence: perpendicular (cos=0) → 0.5, parallel (cos=1) → 1.0
                approachAngleFactor = 0.5 + 0.5 * cosAngle;
            }
        }

        // === Movement Consistency Factor ===
        // Higher speeds relative to max suggest dashes/abilities - less predictable
        // Scale: at max velocity → 0.5x confidence
        double speedRatio = targetSpeed / Constants.MaxReasonableVelocity;
        double consistencyFactor = 1.0 - speedRatio * 0.5;

        // === Reaction Time Factor ===
        // If projectile arrives before target can react, confidence increases
        // Flight time = intercept time - cast delay
        double flightTime = Math.Max(0, interceptTime - castDelay);
        double reactionTimeFactor = Constants.GetReactionTimeFactor(flightTime);

        // Combine factors
        double enhancedConfidence = baseConfidence * approachAngleFactor * consistencyFactor * reactionTimeFactor;

        return Math.Clamp(enhancedConfidence, Constants.Epsilon, 1.0);
    }

    /// <summary>
    /// Optimized prediction for stationary targets.
    /// For stationary targets, there's no "behind" direction - aim at center.
    /// </summary>
    private static PredictionResult PredictStationary(
        PredictionInput input,
        double distance,
        double effectiveRadius)
    {
        // Already within collision radius - instant hit
        if (distance <= effectiveRadius)
        {
            return new PredictionResult.Hit(
                CastPosition: input.TargetPosition,
                PredictedTargetPosition: input.TargetPosition,
                InterceptTime: input.Skillshot.Delay,
                Confidence: 1.0);
        }

        // Check range
        if (distance - effectiveRadius > input.Skillshot.Range)
        {
            return new PredictionResult.OutOfRange(distance - effectiveRadius, input.Skillshot.Range);
        }

        // For stationary target: travel distance to edge = distance - effectiveRadius
        double travelDistance = distance - effectiveRadius;
        double flightTime = travelDistance / input.Skillshot.Speed;
        double totalTime = input.Skillshot.Delay + flightTime;

        return new PredictionResult.Hit(
            CastPosition: input.TargetPosition,
            PredictedTargetPosition: input.TargetPosition,
            InterceptTime: totalTime,
            Confidence: 1.0);
    }

    #region Circular Skillshot Prediction

    /// <inheritdoc />
    public PredictionResult PredictCircular(CircularPredictionInput input)
    {
        // === Input Validation ===
        if (input.Skillshot.Radius <= Constants.Epsilon)
            return new PredictionResult.Unreachable("Skillshot radius must be positive");

        if (input.Skillshot.Range <= Constants.Epsilon)
            return new PredictionResult.Unreachable("Skillshot range must be positive");

        if (input.Skillshot.Delay < 0)
            return new PredictionResult.Unreachable("Skillshot delay cannot be negative");

        // === Use Path-Based Prediction if Available ===
        if (input.HasPath)
        {
            return PredictCircularWithPath(input);
        }

        // === Velocity-Based Prediction ===
        return PredictCircularWithVelocity(input);
    }

    /// <summary>
    /// Circular prediction with path-based target movement.
    /// </summary>
    private PredictionResult PredictCircularWithPath(CircularPredictionInput input)
    {
        var path = input.TargetPath!;

        // === Velocity Sanity Checks ===
        if (path.Speed > Constants.MaxReasonableVelocity)
            return new PredictionResult.Unreachable("Target velocity exceeds reasonable bounds");

        double effectiveRadius = input.Skillshot.Radius + input.TargetHitboxRadius;

        // === Use Behind-Target Strategy ===
        var result = InterceptSolver.SolveCircularPathBehindTarget(
            input.CasterPosition,
            path,
            input.Skillshot.Radius,
            input.Skillshot.Delay,
            input.TargetHitboxRadius,
            input.Skillshot.Range,
            behindMargin: CalculateCircularAdaptiveMargin(effectiveRadius));

        if (!result.HasValue)
        {
            return new PredictionResult.OutOfRange(
                (path.GetPositionAtTime(input.Skillshot.Delay) - input.CasterPosition).Length,
                input.Skillshot.Range);
        }

        var interceptResult = result.Value;

        // === Calculate Confidence ===
        double confidence = InterceptSolver.CalculateCircularConfidence(
            input.Skillshot.Delay,
            path.Speed,
            input.Skillshot.Radius,
            input.TargetHitboxRadius);

        // Apply path uncertainty penalty
        double remainingPathTime = path.GetRemainingPathTime();
        if (remainingPathTime > Constants.Epsilon && input.Skillshot.Delay > 0)
        {
            double pathProgress = input.Skillshot.Delay / remainingPathTime;
            double pathPenalty = 1 - 0.5 * pathProgress;
            confidence *= pathPenalty;
        }

        return new PredictionResult.Hit(
            CastPosition: interceptResult.CastPosition,
            PredictedTargetPosition: interceptResult.PredictedTargetPosition,
            InterceptTime: interceptResult.DetonationTime,
            Confidence: Math.Clamp(confidence, Constants.Epsilon, 1.0));
    }

    /// <summary>
    /// Circular prediction with velocity-based target movement.
    /// </summary>
    private PredictionResult PredictCircularWithVelocity(CircularPredictionInput input)
    {
        // === Velocity Sanity Checks ===
        double targetSpeed = input.TargetVelocity.Length;
        if (targetSpeed > Constants.MaxReasonableVelocity)
            return new PredictionResult.Unreachable("Target velocity exceeds reasonable bounds");

        // === Geometry Setup ===
        var displacement = input.TargetPosition - input.CasterPosition;
        double distance = displacement.Length;
        double effectiveRadius = input.Skillshot.Radius + input.TargetHitboxRadius;

        // === Stationary Target ===
        if (targetSpeed < Constants.MinVelocity)
        {
            return PredictCircularStationary(input, distance, effectiveRadius);
        }

        // === Calculate Adaptive Margin ===
        double adaptiveMargin = CalculateCircularAdaptiveMargin(effectiveRadius);

        // === Solve Behind-Target Circular Intercept ===
        var result = InterceptSolver.SolveCircularBehindTarget(
            input.CasterPosition,
            input.TargetPosition,
            input.TargetVelocity,
            input.Skillshot.Radius,
            input.Skillshot.Delay,
            input.TargetHitboxRadius,
            input.Skillshot.Range,
            behindMargin: adaptiveMargin);

        if (!result.HasValue)
        {
            // Calculate where target will be for error reporting
            Point2D predictedPos = input.TargetPosition + input.TargetVelocity * input.Skillshot.Delay;
            double predictedDistance = (predictedPos - input.CasterPosition).Length;
            return new PredictionResult.OutOfRange(predictedDistance, input.Skillshot.Range);
        }

        var interceptResult = result.Value;

        // === Calculate Confidence ===
        double confidence = InterceptSolver.CalculateCircularConfidence(
            input.Skillshot.Delay,
            targetSpeed,
            input.Skillshot.Radius,
            input.TargetHitboxRadius);

        // === Apply Movement Direction Factor ===
        // Perpendicular movement is harder to predict for circular spells
        if (targetSpeed > Constants.MinVelocity && displacement.Length > Constants.Epsilon)
        {
            double dotProduct = displacement.DotProduct(input.TargetVelocity);
            double cosAngle = dotProduct / (displacement.Length * targetSpeed);
            cosAngle = Math.Clamp(cosAngle, -1.0, 1.0);

            // Targets moving toward/away from caster are easier to predict
            // Targets moving perpendicular are harder (use same formula as linear)
            double angleFactor = 0.5 + 0.5 * Math.Abs(cosAngle);
            confidence *= angleFactor;
        }

        return new PredictionResult.Hit(
            CastPosition: interceptResult.CastPosition,
            PredictedTargetPosition: interceptResult.PredictedTargetPosition,
            InterceptTime: interceptResult.DetonationTime,
            Confidence: Math.Clamp(confidence, Constants.Epsilon, 1.0));
    }

    /// <summary>
    /// Calculates adaptive margin for circular skillshots.
    /// Circular spells have different considerations than linear ones:
    /// - Only delay matters (no flight time)
    /// - Larger effective radius typically
    /// 
    /// For circular spells, aiming at the exact trailing edge (Epsilon margin)
    /// is risky because the target is moving away from the spell center.
    /// A grazing hit might miss if the target moves even slightly further.
    /// 
    /// Instead, we aim halfway between the center and the trailing edge.
    /// This maintains the "behind target" strategy (catching stops/reversals)
    /// while ensuring a solid hit on moving targets.
    /// </summary>
    /// <param name="effectiveRadius">Combined collision radius (spell radius + target hitbox)</param>
    /// <returns>Margin in game units, guaranteed to be less than effectiveRadius</returns>
    private static double CalculateCircularAdaptiveMargin(double effectiveRadius)
    {
        // Handle degenerate case
        if (effectiveRadius <= Constants.Epsilon)
            return 0;

        // For circular spells, use 50% of effective radius as margin.
        // This places the target halfway between spell center and edge,
        // providing a balance between the "behind target" strategy and hit reliability.
        double margin = effectiveRadius * 0.5;

        // Defensive clamp: margin must be strictly less than effectiveRadius
        return Math.Min(margin, effectiveRadius - Constants.Epsilon);
    }

    /// <summary>
    /// Circular prediction for stationary targets.
    /// </summary>
    private static PredictionResult PredictCircularStationary(
        CircularPredictionInput input,
        double distance,
        double effectiveRadius)
    {
        // Check if predicted position is within range
        if (distance > input.Skillshot.Range + Constants.RangeTolerance)
        {
            return new PredictionResult.OutOfRange(distance, input.Skillshot.Range);
        }

        // For stationary target, aim directly at current position
        return new PredictionResult.Hit(
            CastPosition: input.TargetPosition,
            PredictedTargetPosition: input.TargetPosition,
            InterceptTime: input.Skillshot.Delay,
            Confidence: 1.0);
    }

    #endregion

    #region Multi-Target Priority Selection

    /// <summary>
    /// Evaluates multiple targets and returns them ranked by hit probability.
    /// 
    /// This method is useful for team fights or scenarios with multiple potential targets.
    /// It considers:
    /// - Prediction confidence (hit probability)
    /// - Target priority weight (optional, for focusing high-value targets)
    /// - Range efficiency (closer targets preferred when confidence is similar)
    /// 
    /// Example usage:
    /// <code>
    /// var targets = new[] { enemyAdc, enemySupport, enemyMid };
    /// var ranked = ultimate.RankTargets(casterPos, skillshot, targets);
    /// var bestTarget = ranked.FirstOrDefault(t => t.Result is PredictionResult.Hit);
    /// </code>
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets with their movement data</param>
    /// <returns>Targets ranked by priority score (highest first)</returns>
    public IReadOnlyList<RankedTarget> RankTargets(
        Point2D casterPosition,
        LinearSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        if (targets.IsEmpty)
            return Array.Empty<RankedTarget>();

        var results = new List<RankedTarget>(targets.Length);

        foreach (var target in targets)
        {
            var input = new PredictionInput(
                casterPosition,
                target.Position,
                target.Velocity,
                skillshot,
                target.HitboxRadius,
                target.Path);

            var result = Predict(input);
            double priorityScore = CalculatePriorityScore(result, target.PriorityWeight, casterPosition, skillshot.Range);

            results.Add(new RankedTarget(target, result, priorityScore));
        }

        // Sort by priority score descending (highest priority first)
        results.Sort((a, b) => b.PriorityScore.CompareTo(a.PriorityScore));

        return results;
    }

    /// <summary>
    /// Evaluates multiple targets for a circular skillshot and returns them ranked by hit probability.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The circular skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets with their movement data</param>
    /// <returns>Targets ranked by priority score (highest first)</returns>
    public IReadOnlyList<RankedTarget> RankTargetsCircular(
        Point2D casterPosition,
        CircularSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        if (targets.IsEmpty)
            return Array.Empty<RankedTarget>();

        var results = new List<RankedTarget>(targets.Length);

        foreach (var target in targets)
        {
            var input = new CircularPredictionInput(
                casterPosition,
                target.Position,
                target.Velocity,
                skillshot,
                target.HitboxRadius,
                target.Path);

            var result = PredictCircular(input);
            double priorityScore = CalculatePriorityScore(result, target.PriorityWeight, casterPosition, skillshot.Range);

            results.Add(new RankedTarget(target, result, priorityScore));
        }

        // Sort by priority score descending (highest priority first)
        results.Sort((a, b) => b.PriorityScore.CompareTo(a.PriorityScore));

        return results;
    }

    /// <summary>
    /// Gets the single best target from a collection of candidates.
    /// Returns null if no hittable targets exist.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets</param>
    /// <returns>The best target to aim for, or null if none are hittable</returns>
    public RankedTarget? GetBestTarget(
        Point2D casterPosition,
        LinearSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        var ranked = RankTargets(casterPosition, skillshot, targets);

        // Return first hittable target (already sorted by priority)
        foreach (var target in ranked)
        {
            if (target.Result is PredictionResult.Hit)
                return target;
        }

        return null;
    }

    /// <summary>
    /// Gets the single best target for a circular skillshot.
    /// Returns null if no hittable targets exist.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The circular skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets</param>
    /// <returns>The best target to aim for, or null if none are hittable</returns>
    public RankedTarget? GetBestTargetCircular(
        Point2D casterPosition,
        CircularSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        var ranked = RankTargetsCircular(casterPosition, skillshot, targets);

        // Return first hittable target (already sorted by priority)
        foreach (var target in ranked)
        {
            if (target.Result is PredictionResult.Hit)
                return target;
        }

        return null;
    }

    /// <summary>
    /// Calculates priority score for ranking targets.
    /// 
    /// Score components:
    /// - Confidence (0-1): Base hit probability
    /// - Priority weight (0-∞): User-defined target importance (e.g., ADC = 2.0, Tank = 0.5)
    /// - Range efficiency (0-1): Bonus for closer targets (reduces risk of interception)
    /// 
    /// Formula: Score = Confidence * PriorityWeight * (0.8 + 0.2 * RangeEfficiency)
    /// 
    /// The range efficiency contributes only 20% to allow priority and confidence to dominate,
    /// but still prefers closer targets when other factors are equal.
    /// </summary>
    private static double CalculatePriorityScore(
        PredictionResult result,
        double priorityWeight,
        Point2D casterPosition,
        double skillshotRange)
    {
        if (result is not PredictionResult.Hit hit)
        {
            // Non-hittable targets get score based on how close they are to being hittable
            if (result is PredictionResult.OutOfRange outOfRange)
            {
                // Small negative score based on how far out of range
                double overRange = outOfRange.Distance - outOfRange.MaxRange;
                return -overRange / skillshotRange;
            }

            // Unreachable targets get lowest priority
            return -1000;
        }

        // === Base Score: Confidence ===
        double score = hit.Confidence;

        // === Apply Priority Weight ===
        // Default weight is 1.0, higher values = more important target
        score *= Math.Max(0.1, priorityWeight);

        // === Range Efficiency Bonus ===
        // Closer targets are preferred (less travel time = less chance of missing)
        double distance = (hit.CastPosition - casterPosition).Length;
        double rangeEfficiency = 1.0 - Math.Clamp(distance / skillshotRange, 0, 1);

        // Range efficiency contributes 20% to final score
        score *= 0.8 + 0.2 * rangeEfficiency;

        return score;
    }

    #endregion
}
