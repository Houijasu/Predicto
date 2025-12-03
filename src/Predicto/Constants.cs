namespace Predicto;

/// <summary>
/// Constants used throughout the prediction engine.
/// Based on League of Legends game mechanics.
/// </summary>
public static class Constants
{
    /// <summary>
    /// Numerical epsilon for floating-point comparisons.
    /// </summary>
    public const double Epsilon = 1e-9;

    /// <summary>
    /// Minimum velocity threshold - below this, target is considered stationary.
    /// </summary>
    public const double MinVelocity = 1.0;

    /// <summary>
    /// League of Legends server tick rate in Hz.
    /// </summary>
    public const double ServerTickRate = 30.0;

    /// <summary>
    /// Duration of one server tick in seconds.
    /// </summary>
    public const double TickDuration = 1.0 / ServerTickRate;

    /// <summary>
    /// Default champion hitbox radius in units.
    /// </summary>
    public const double DefaultHitboxRadius = 65.0;

    /// <summary>
    /// Maximum prediction time horizon in seconds.
    /// </summary>
    public const double MaxPredictionTime = 3.0;


    /// <summary>
    /// Pixel margin for trailing edge collision (1 game unit ≈ 1 pixel).
    /// Used to hit target "from behind" - just barely inside collision zone.
    /// This makes dodging extremely difficult as target would need to reverse direction.
    /// </summary>
    public const double TrailingEdgeMargin = 1.0;

    // === Performance & Sanity Thresholds ===

    /// <summary>
    /// Maximum reasonable target velocity (units/second).
    /// Champions with highest base MS + Ghost + Boots + Abilities cap around 700-800.
    /// We use 2000 to allow for extreme edge cases.
    /// </summary>
    public const double MaxReasonableVelocity = 2000.0;

    /// <summary>
    /// Maximum reasonable skillshot speed (units/second).
    /// Fastest skillshots in LoL are around 2500 (e.g., Jinx W).
    /// We use 5000 to allow for custom implementations.
    /// </summary>
    public const double MaxReasonableSkillshotSpeed = 5000.0;

    /// <summary>
    /// Tolerance threshold for determining if quadratic solution is "good enough".
    /// If initial quadratic estimate produces error below this, skip further refinement.
    /// </summary>
    public const double AdaptiveRefinementThreshold = 0.5;

    /// <summary>
    /// Distance threshold for "easy" shots that don't need full refinement.
    /// Point-blank range shots have low prediction uncertainty.
    /// </summary>
    public const double EasyShotDistanceThreshold = 200.0;

    /// <summary>
    /// Grazing angle threshold (radians). When target moves nearly perpendicular
    /// to displacement (> ~80 degrees), prediction becomes less reliable.
    /// </summary>
    public const double GrazingAngleThreshold = 1.4; // ~80 degrees

    /// <summary>
    /// Minimum adaptive margin as a fraction of effective radius.
    /// Prevents margin from being too small for fast targets.
    /// </summary>
    public const double MinAdaptiveMarginFraction = 0.01;

    /// <summary>
    /// Maximum adaptive margin as a fraction of effective radius.
    /// Prevents margin from being too large for slow/distant targets.
    /// </summary>
    public const double MaxAdaptiveMarginFraction = 0.5;
    /// <summary>
    /// Safety factor for tick-based margin calculation (fraction of tick distance).
    /// Used to ensure we hit inside the collision zone despite discrete time steps.
    /// 5% gives ~3-4 pixels margin at typical speeds, enough for 30Hz/60Hz reliability.
    /// </summary>
    public const double TickMarginSafetyFactor = 0.05;
}
