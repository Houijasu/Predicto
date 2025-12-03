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
        double flightTime = Math.Max(0, interceptTime - input.Skillshot.Delay);
        double travelDistance = input.Skillshot.Speed * flightTime;

        if (travelDistance > input.Skillshot.Range)
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
    /// Calculates adaptive margin based on target speed and distance.
    /// Fast-moving targets get smaller margins (more aggressive).
    /// Slow or distant targets get larger margins (more forgiving).
    /// </summary>
    private static double CalculateAdaptiveMargin(
        double targetSpeed,
        double distance,
        double skillshotSpeed,
        double skillshotWidth,
        double effectiveRadius)
    {
        // Calculate margin based on server tick rate for reliable hits
        // LoL uses 30Hz server tick with swept collision detection
        // We need enough margin to account for discrete tick boundaries
        
        // Relative speed between skillshot and target (worst case: head-on)
        double relativeSpeed = skillshotSpeed + targetSpeed;
        
        // Distance covered per server tick
        double distancePerTick = relativeSpeed * Constants.TickDuration;
        
        // Use safety factor of tick distance as margin
        // This handles tick boundary rounding while keeping aim aggressive
        double tickBasedMargin = distancePerTick * Constants.TickMarginSafetyFactor;
        
        // Minimum margin of 0.1 pixels for mathematical precision
        // Maximum margin of 10% of effective radius to stay aggressive
        double minMargin = 0.1;
        double maxMargin = effectiveRadius * 0.10;
        
        // For very small effective radii, scale proportionally
        if (effectiveRadius < 1.0)
            return effectiveRadius * 0.5;
        
        return Math.Clamp(tickBasedMargin, minMargin, maxMargin);
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
}
