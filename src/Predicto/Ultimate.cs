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

        // === Geometry Setup ===
        var displacement = path.CurrentPosition - input.CasterPosition;
        double distance = displacement.Length;
        double effectiveRadius = input.TargetHitboxRadius + input.Skillshot.Width / 2;

        // === Early Out: Target clearly out of range and moving away ===
        if (distance > input.Skillshot.Range + effectiveRadius)
        {
            double dotProduct = displacement.DotProduct(path.GetCurrentVelocity());
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
        double adaptiveMargin = CalculateAdaptiveMargin(
            path.Speed,
            distance,
            input.Skillshot.Speed,
            input.Skillshot.Width,
            effectiveRadius);

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
        var currentVelocity = path.GetCurrentVelocity();
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
            double waypointPenalty = Math.Pow(0.9, waypointDelta); // 10% reduction per waypoint
            confidence *= waypointPenalty;
        }

        // === Confidence Penalty for Segment Distance ===
        // Even on the current segment, predictions far along the path are less reliable
        // Calculate how far along the path the intercept occurs
        double remainingPathTime = path.GetRemainingPathTime();
        if (remainingPathTime > Constants.Epsilon)
        {
            double segmentProgress = interceptResult.InterceptTime / remainingPathTime;
            // Apply mild penalty for predictions deep into the path (exponential decay)
            // At 100% path progress: 0.85x confidence, at 50%: ~0.92x
            double segmentPenalty = Math.Exp(-0.16 * segmentProgress);
            confidence *= segmentPenalty;
        }

        return new PredictionResult.Hit(
            CastPosition: interceptResult.AimPoint,
            PredictedTargetPosition: interceptResult.PredictedTargetPosition,
            InterceptTime: interceptResult.InterceptTime,
            Confidence: Math.Clamp(confidence, 0.1, 1.0));
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
                // Calculate if target can possibly be caught
                // Max time = (range + effectiveRadius) / skillshotSpeed
                double maxReachTime = (input.Skillshot.Range + effectiveRadius) / input.Skillshot.Speed + input.Skillshot.Delay;
                double futureDistance = distance + targetSpeed * maxReachTime;
                
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
        double adaptiveMargin = CalculateAdaptiveMargin(
            targetSpeed, 
            distance, 
            input.Skillshot.Speed,
            input.Skillshot.Width,
            effectiveRadius);

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
    /// </summary>
    private static bool IsEasyCase(
        double distance,
        double targetSpeed,
        Vector2D displacement,
        Vector2D targetVelocity,
        double skillshotSpeed)
    {
        // Close range shots are easy
        if (distance < Constants.EasyShotDistanceThreshold)
            return true;

        // Slow targets are easy
        if (targetSpeed < skillshotSpeed * 0.1)
            return true;

        // Target moving toward caster (easier to hit)
        double dotProduct = displacement.DotProduct(targetVelocity);
        if (dotProduct < 0 && Math.Abs(dotProduct) > targetSpeed * distance * 0.5)
            return true;

        return false;
    }

    /// <summary>
    /// Calculates adaptive margin based on prediction uncertainty factors.
    /// 
    /// The margin determines how far "inside" the collision zone we aim.
    /// Larger margins = more reliable hits but less optimal positioning.
    /// Smaller margins = more aggressive but risk missing due to:
    ///   - Target velocity prediction errors
    ///   - Network latency variations
    ///   - Floating-point precision accumulation
    /// 
    /// Factors considered:
    /// - Target speed: Faster targets have more prediction uncertainty
    /// - Distance: Farther targets accumulate more positional error over flight time
    /// - Speed ratio: When target speed approaches skillshot speed, small errors amplify
    /// </summary>
    private static double CalculateAdaptiveMargin(
        double targetSpeed,
        double distance,
        double skillshotSpeed,
        double skillshotWidth,
        double effectiveRadius)
    {
        // Base margin: minimum for numerical stability
        const double baseMargin = 0.1;
        
        // For very small effective radii, scale proportionally
        if (effectiveRadius < 1.0)
            return effectiveRadius * 0.5;
        
        // === Speed-based uncertainty ===
        // Faster targets have more prediction uncertainty
        // Scale: 0 at rest, up to ~2 pixels at max reasonable velocity
        double speedUncertainty = targetSpeed / Constants.MaxReasonableVelocity * 2.0;
        
        // === Distance-based uncertainty ===
        // Longer flight times accumulate more error
        // Estimate flight time and scale uncertainty
        double estimatedFlightTime = distance / skillshotSpeed;
        double distanceUncertainty = estimatedFlightTime * 0.5; // ~0.5 pixels per second of flight
        
        // === Speed ratio factor ===
        // When target speed approaches skillshot speed, prediction becomes unstable
        // (the quadratic discriminant approaches zero, amplifying numerical errors)
        double speedRatio = targetSpeed / (skillshotSpeed + Constants.Epsilon);
        double ratioUncertainty = 0.0;
        if (speedRatio > 0.7)
        {
            // Exponential increase as we approach equal speeds
            ratioUncertainty = Math.Pow((speedRatio - 0.7) / 0.3, 2) * 3.0;
        }
        
        // Combine uncertainties (not additive - use RMS-like combination)
        double totalUncertainty = Math.Sqrt(
            speedUncertainty * speedUncertainty + 
            distanceUncertainty * distanceUncertainty + 
            ratioUncertainty * ratioUncertainty);
        
        // Calculate final margin
        double margin = baseMargin + totalUncertainty;
        
        // Clamp to reasonable bounds
        double minMargin = effectiveRadius * Constants.MinAdaptiveMarginFraction;
        double maxMargin = effectiveRadius * Constants.MaxAdaptiveMarginFraction;
        
        return Math.Clamp(margin, minMargin, maxMargin);
    }

    /// <summary>
    /// Enhanced confidence calculation that factors in approach angle and movement patterns.
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
        // (they might suddenly stop or reverse)
        double approachAngleFactor = 1.0;
        if (targetSpeed > Constants.MinVelocity && displacement.Length > Constants.Epsilon)
        {
            // Calculate angle between displacement and velocity
            double displacementLength = displacement.Length;
            double velocityLength = targetVelocity.Length;
            
            if (displacementLength > Constants.Epsilon && velocityLength > Constants.Epsilon)
            {
                double dotProduct = displacement.DotProduct(targetVelocity);
                double cosAngle = dotProduct / (displacementLength * velocityLength);
                cosAngle = Math.Clamp(cosAngle, -1.0, 1.0);
                double angle = Math.Acos(Math.Abs(cosAngle));

                // Perpendicular movement (angle near 90°) reduces confidence
                // Parallel movement (toward or away) is more predictable
                if (angle > Constants.GrazingAngleThreshold)
                {
                    // Reduce confidence for grazing angles
                    approachAngleFactor = 0.7 + 0.3 * (1.0 - (angle - Constants.GrazingAngleThreshold) / (Math.PI / 2 - Constants.GrazingAngleThreshold));
                    approachAngleFactor = Math.Max(approachAngleFactor, 0.5);
                }
            }
        }

        // === Movement Consistency Factor ===
        // Targets moving at consistent speed in consistent direction are more predictable
        // This is a simplified version - real implementation would track historical data
        double consistencyFactor = 1.0;
        
        // Very high speeds suggest dashes/abilities - less predictable
        if (targetSpeed > 500)
        {
            consistencyFactor = Math.Max(0.6, 1.0 - (targetSpeed - 500) / 1500);
        }

        // Combine factors
        double enhancedConfidence = baseConfidence * approachAngleFactor * consistencyFactor;

        return Math.Clamp(enhancedConfidence, 0.1, 1.0);
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
            behindMargin: CalculateCircularAdaptiveMargin(path.Speed, input.Skillshot.Delay, effectiveRadius));

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
            double pathPenalty = Math.Exp(-0.2 * pathProgress);
            confidence *= pathPenalty;
        }

        return new PredictionResult.Hit(
            CastPosition: interceptResult.CastPosition,
            PredictedTargetPosition: interceptResult.PredictedTargetPosition,
            InterceptTime: interceptResult.DetonationTime,
            Confidence: Math.Clamp(confidence, 0.1, 1.0));
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
        double adaptiveMargin = CalculateCircularAdaptiveMargin(targetSpeed, input.Skillshot.Delay, effectiveRadius);

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
            // Targets moving perpendicular are harder
            double angleFactor = 0.7 + 0.3 * Math.Abs(cosAngle);
            confidence *= angleFactor;
        }

        return new PredictionResult.Hit(
            CastPosition: interceptResult.CastPosition,
            PredictedTargetPosition: interceptResult.PredictedTargetPosition,
            InterceptTime: interceptResult.DetonationTime,
            Confidence: Math.Clamp(confidence, 0.1, 1.0));
    }

    /// <summary>
    /// Calculates adaptive margin for circular skillshots.
    /// Circular spells have different considerations than linear ones:
    /// - Only delay matters (no flight time)
    /// - Larger effective radius typically
    /// </summary>
    private static double CalculateCircularAdaptiveMargin(
        double targetSpeed,
        double castDelay,
        double effectiveRadius)
    {
        // Base margin for numerical stability
        const double baseMargin = 0.1;

        if (effectiveRadius < 1.0)
            return effectiveRadius * 0.5;

        // How far can target travel during delay?
        double travelDuringDelay = targetSpeed * castDelay;

        // If target can travel more than half the effective radius,
        // increase margin proportionally
        double travelRatio = travelDuringDelay / effectiveRadius;
        double travelUncertainty = Math.Min(travelRatio * 2.0, 5.0);

        double margin = baseMargin + travelUncertainty;

        // Clamp to reasonable bounds
        double minMargin = effectiveRadius * Constants.MinAdaptiveMarginFraction;
        double maxMargin = effectiveRadius * Constants.MaxAdaptiveMarginFraction;

        return Math.Clamp(margin, minMargin, maxMargin);
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
}
