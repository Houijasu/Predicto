namespace Predicto.Models;

/// <summary>
/// Parameters defining a circular (ground-targeted) skillshot.
/// These abilities have a delay before detonating at a fixed location.
/// Examples: Xerath W, Veigar W, Cho'Gath Q, Ziggs W/E/R
/// 
/// Key characteristics:
/// - No travel time (instant hit at cast location after delay)
/// - Circular hitbox with radius
/// - Fixed range from caster
/// 
/// Mathematical model:
/// - Target position at time t: P + V*t
/// - Spell detonates at: CastPosition after Delay seconds
/// - Hit if: |TargetPos(Delay) - CastPosition| ≤ Radius + TargetHitboxRadius
/// </summary>
/// <param name="Radius">Radius of the circular effect zone in units</param>
/// <param name="Range">Maximum cast range from caster in units</param>
/// <param name="Delay">Time from cast to detonation in seconds</param>
public readonly record struct CircularSkillshot(
    double Radius,
    double Range,
    double Delay)
{
    /// <summary>
    /// Creates a CircularSkillshot with common defaults.
    /// </summary>
    public static CircularSkillshot Create(
        double radius,
        double range,
        double delay) => new(radius, range, delay);

    /// <summary>
    /// Xerath W - Eye of Destruction
    /// Inner circle: 50 radius (25% bonus damage + slow)
    /// Outer circle: 200 radius (normal damage + slow)
    /// Cast range: 1100
    /// Delay: 0.5s (from cast to detonation)
    /// </summary>
    public static CircularSkillshot XerathW => new(Radius: 200, Range: 1100, Delay: 0.5);

    /// <summary>
    /// Xerath W inner circle for maximum damage.
    /// </summary>
    public static CircularSkillshot XerathWCenter => new(Radius: 50, Range: 1100, Delay: 0.5);

    /// <summary>
    /// Veigar W - Dark Matter
    /// Radius: 112
    /// Range: 950
    /// Delay: 1.2s
    /// </summary>
    public static CircularSkillshot VeigarW => new(Radius: 112, Range: 950, Delay: 1.2);

    /// <summary>
    /// Cho'Gath Q - Rupture
    /// Radius: 175
    /// Range: 950
    /// Delay: 0.5s (knock up after 0.625s total)
    /// </summary>
    public static CircularSkillshot ChoGathQ => new(Radius: 175, Range: 950, Delay: 0.5);

    /// <summary>
    /// Ziggs W - Satchel Charge
    /// Radius: 150
    /// Range: 1000
    /// Delay: ~0.25s (can be detonated early)
    /// </summary>
    public static CircularSkillshot ZiggsW => new(Radius: 150, Range: 1000, Delay: 0.25);

    /// <summary>
    /// Ziggs E - Hexplosive Minefield
    /// Each mine radius: 60
    /// Scatter radius: 250
    /// Range: 900
    /// Arm delay: 0.25s
    /// </summary>
    public static CircularSkillshot ZiggsE => new(Radius: 250, Range: 900, Delay: 0.25);
}
