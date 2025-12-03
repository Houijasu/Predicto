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
    /// </summary>
    public const double Epsilon = 1e-9;

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
    /// Pixel margin for trailing edge collision (1 game unit ≈ 1 pixel).
    /// Used to hit target "from behind" - just barely inside collision zone.
    /// </summary>
    public const double TrailingEdgeMargin = 1.0;

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
}
