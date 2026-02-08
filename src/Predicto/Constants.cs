namespace Predicto;

/// <summary>
/// Constants used throughout the prediction engine.
/// Based on League of Legends game mechanics.
///
/// DESIGN RULE: Only allowed math literals are: epsilon, 0.5, 1, 2
/// Game data values (tick rate, hitbox radius, etc.) are exempt as they are facts, not tuning parameters.
/// </summary>
public static class Constants
{
    /// <summary>
    /// Numerical epsilon for floating-point comparisons.
    /// Aligned with MathNet.Numerics Precision.DoublePrecision (~1.11e-16).
    /// </summary>
    public const double Epsilon = 1.1102230246251565E-16;

    /// <summary>
    /// Tolerance for range comparisons to account for floating-point precision.
    /// Uses same epsilon as general comparisons.
    /// </summary>
    public const double RangeTolerance = Epsilon;

    /// <summary>
    /// Minimum velocity threshold - below this, target is considered stationary.
    /// </summary>
    public const double MinVelocity = 1.0;

    /// <summary>
    /// League of Legends server tick rate in Hz.
    /// This is game data, not a tuning parameter.
    /// </summary>
    public const double ServerTickRate = 30.0;

    /// <summary>
    /// Duration of one server tick in seconds.
    /// Derived from ServerTickRate.
    /// </summary>
    public const double TickDuration = 1.0 / ServerTickRate;

    /// <summary>
    /// Default champion hitbox radius in units.
    /// This is game data from League of Legends.
    /// </summary>
    public const double DefaultHitboxRadius = 65.0;

    /// <summary>
    /// Default caster hitbox radius in units.
    /// Used for origin offset in minimal time calculations.
    /// </summary>
    public const double DefaultCasterHitboxRadius = 65.0;

    /// <summary>
    /// Pixel margin for trailing edge collision (1 game unit ≈ 1 pixel).
    /// Used to hit target "from behind" - just barely inside collision zone.
    /// </summary>
    public const double TrailingEdgeMargin = 1.0;

    /// <summary>
    /// Threshold for "small radius" in adaptive margin calculations.
    /// Below this, margin scales proportionally (50% of radius).
    /// Above this, margin is fixed at TrailingEdgeMargin.
    /// Chosen so that at exactly this threshold, both formulas yield the same value (1.0).
    /// </summary>
    public const double SmallRadiusThreshold = 2.0;

    // === Performance & Sanity Thresholds (Game Data) ===

    /// <summary>
    /// Maximum reasonable target velocity (units/second).
    /// Champions with highest base MS + Ghost + Boots + Abilities cap around 700-800.
    /// This is game data representing physical limits.
    /// </summary>
    public const double MaxReasonableVelocity = 2000.0;

    /// <summary>
    /// Maximum reasonable skillshot speed (units/second).
    /// Fastest skillshots in LoL are around 2500 (e.g., Jinx W).
    /// This is game data representing physical limits.
    /// </summary>
    public const double MaxReasonableSkillshotSpeed = 5000.0;

    // === Reaction Time Constants (Game Research Data) ===

    /// <summary>
    /// Average human visual reaction time in seconds.
    /// Based on research: average is 250ms, competitive gamers 180-220ms.
    /// This is game research data representing human performance.
    /// </summary>
    public const double AverageReactionTime = 0.25;

    /// <summary>
    /// Human reaction time used for dodge prediction calculations.
    /// Alias for AverageReactionTime for clarity in Least Action solver.
    /// </summary>
    public const double HumanReactionTime = AverageReactionTime;

    /// <summary>
    /// Minimum possible human reaction time (elite performance) in seconds.
    /// Professional esports players can achieve 150-180ms in optimal conditions.
    /// </summary>
    public const double MinReactionTime = 0.15;

    /// <summary>
    /// Maximum reasonable reaction time in seconds.
    /// Beyond this, target is considered to have fully reacted to any visible threat.
    /// </summary>
    public const double MaxReactionTime = 0.5;

    // === Path-End Blending Constants ===

    /// <summary>
    /// Time threshold (in seconds) for blending from behind-target to center aim.
    /// When the target's remaining path time minus estimated intercept time is less than
    /// this threshold, we start blending toward center aim for faster interception.
    /// At exactly 0 seconds remaining, we aim at center (fastest intercept).
    /// </summary>
    public const double PathEndBlendThreshold = 0.5;

    // === Waypoint Confidence Penalty Constants ===

    /// <summary>
    /// Base factor for waypoint penalty calculation.
    /// Confidence decays exponentially: penalty = WaypointPenaltyBase ^ waypointDelta.
    /// Each additional waypoint halves the confidence (0.5^1 = 50%, 0.5^2 = 25%, etc.).
    /// </summary>
    public const double WaypointPenaltyBase = 0.5;

    /// <summary>
    /// Maximum confidence reduction factor for path segment progress.
    /// At 100% path progress, confidence is multiplied by (1 - SegmentProgressPenaltyFactor) = 0.5.
    /// At 50% progress, penalty is 0.75x. Linear interpolation.
    /// </summary>
    public const double SegmentProgressPenaltyFactor = 0.5;

    // === Hitscan Confidence ===

    /// <summary>
    /// Confidence multiplier applied to hitscan predictions.
    /// Hitscan is more reliable due to instant travel, so confidence gets a small boost.
    /// </summary>
    public const double HitscanConfidenceBoost = 1.1;

    // === Pre-Filter & Multi-Target Thresholds ===

    /// <summary>
    /// Generous distance buffer added to range when pre-filtering targets.
    /// Accounts for hitbox radius and movement uncertainty.
    /// </summary>
    public const double PrefilterBuffer = 1000.0;

    // === Solver Tuning Constants ===

    /// <summary>
    /// Maximum array length for stack allocation via stackalloc.
    /// Above this threshold, heap allocation is used instead.
    /// </summary>
    public const int StackallocThreshold = 256;

    /// <summary>
    /// Number of time steps in the Gagong strategy coarse grid search.
    /// </summary>
    public const int GagongTimeSteps = 20;

    /// <summary>
    /// Number of angle steps in the Gagong strategy coarse grid search.
    /// Covers the behind hemisphere [-π/2, +π/2] relative to movement direction.
    /// </summary>
    public const int GagongAngleSteps = 12;

    /// <summary>
    /// Maximum iterations for Golden Section Search refinement.
    /// </summary>
    public const int GoldenSectionMaxIterations = 20;

    // === Helper Methods ===

    /// <summary>
    /// Calculate maximum prediction time for a linear skillshot.
    /// Time = Range / Speed (how long until projectile reaches max range).
    /// Fallback uses 2x range/speed for safety margin.
    /// </summary>
    public static double GetMaxPredictionTime(double range, double speed)
        => speed > Epsilon ? range / speed : range * 2;

    /// <summary>
    /// Calculate maximum prediction time for a circular skillshot.
    /// For instant-detonation spells, this is just the delay.
    /// </summary>
    public static double GetMaxPredictionTime(double delay)
        => delay;

    /// <summary>
    /// Calculate how far target moves in one server tick.
    /// Used for margin calculations.
    /// </summary>
    public static double GetTickMovement(double targetSpeed)
        => targetSpeed * TickDuration;

    /// <summary>
    /// Calculate reaction time factor for confidence adjustment.
    /// Shorter flight times than reaction time = higher hit probability.
    ///
    /// Formula: factor = clamp(1 - (flightTime - reactionTime) / reactionTime, 0.5, 1.0)
    ///
    /// - Flight time ≤ reaction time: factor = 1.0 (target can't react)
    /// - Flight time = 2x reaction time: factor = 0.5 (target has time to dodge)
    /// - Flight time ≥ 2x reaction time: factor = 0.5 (minimum confidence)
    /// </summary>
    /// <param name="flightTime">Time from launch to impact (excludes cast delay)</param>
    /// <param name="reactionTime">Assumed reaction time (default: average)</param>
    /// <returns>Factor between 0.5 and 1.0</returns>
    public static double GetReactionTimeFactor(double flightTime, double reactionTime = AverageReactionTime)
    {
        if (reactionTime < Epsilon)
            return 1.0;

        // If flight time is less than reaction time, target can't react
        if (flightTime <= reactionTime)
            return 1.0;

        // Linear decay from 1.0 to 0.5 as flight time increases
        double excessTime = flightTime - reactionTime;
        double factor = 1.0 - (excessTime / reactionTime * 0.5);

        return Math.Max(0.5, factor);
    }

    /// <summary>
    /// Hermite interpolation (smoothstep) for smooth transitions.
    /// Returns 0 when x ≤ edge0, 1 when x ≥ edge1, and smoothly interpolates between.
    /// The result has zero first derivative at both edges (C1 continuous).
    ///
    /// Formula: t² * (3 - 2t) where t = clamp((x - edge0) / (edge1 - edge0), 0, 1)
    /// </summary>
    /// <param name="edge0">Lower edge of the transition</param>
    /// <param name="edge1">Upper edge of the transition</param>
    /// <param name="x">Input value</param>
    /// <returns>Smoothly interpolated value between 0 and 1</returns>
    public static double Smoothstep(double edge0, double edge1, double x)
    {
        if (Math.Abs(edge1 - edge0) < Epsilon)
            return x < edge0 ? 0.0 : 1.0;

        double t = Math.Clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        return t * t * (3.0 - (2.0 * t));
    }

    #region Physics Simulation Constants (Least Action Principle)

    /// <summary>
    /// Default danger amplitude for potential fields.
    /// This is the peak "danger value" at the center of a skillshot path.
    /// Higher values cause stronger dodge responses.
    /// </summary>
    public const double DefaultDangerAmplitude = 1000.0;

    /// <summary>
    /// Danger falloff multiplier relative to skillshot width.
    /// FalloffWidth = SkillshotWidth * DangerFalloffMultiplier.
    /// Larger values create wider danger zones.
    /// </summary>
    public const double DangerFalloffMultiplier = 2.0;

    /// <summary>
    /// Default falloff multiplier relative to radius for circular danger fields.
    /// </summary>
    public const double DefaultCircularDangerFalloff = 1.5;

    /// <summary>
    /// Default anticipation window (seconds) for circular danger fields.
    /// </summary>
    public const double DefaultAnticipationWindow = 0.3;

    /// <summary>
    /// Maximum acceleration for dodge maneuvers (units/second²).
    /// Based on typical champion turn rates in League of Legends.
    /// Champions can change direction very quickly, approximately
    /// reaching max speed in ~0.1-0.2 seconds.
    /// </summary>
    public const double MaxDodgeAcceleration = 3000.0;

    /// <summary>
    /// Default friction coefficient for physics simulation.
    /// Controls how quickly a target decelerates when not actively moving.
    /// Higher values = faster stopping.
    /// </summary>
    public const double DefaultFriction = 5.0;

    /// <summary>
    /// Time step for physics simulation (seconds).
    /// Smaller values are more accurate but slower.
    /// 1/60 second provides good balance of accuracy and performance.
    /// </summary>
    public const double PhysicsTimeStep = 1.0 / 60.0;

    /// <summary>
    /// Maximum physics simulation time (seconds).
    /// Prevents runaway simulations.
    /// </summary>
    public const double MaxPhysicsSimulationTime = 2.0;

    /// <summary>
    /// Danger potential threshold below which a position is considered safe.
    /// Used to determine when a dodge simulation can terminate early.
    /// </summary>
    public const double SafetyPotentialThreshold = 10.0;

    #endregion
}
