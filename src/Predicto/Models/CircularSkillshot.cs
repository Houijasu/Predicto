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

    #region Mage Abilities

    /// <summary>
    /// Xerath W - Eye of Destruction
    /// Outer circle: 200 radius, Inner: 50 radius (25% bonus damage)
    /// Range: 1100, Delay: 0.5s
    /// </summary>
    public static CircularSkillshot XerathW => new(Radius: 200, Range: 1100, Delay: 0.5);

    /// <summary>
    /// Xerath W inner circle for maximum damage.
    /// </summary>
    public static CircularSkillshot XerathWCenter => new(Radius: 50, Range: 1100, Delay: 0.5);

    /// <summary>
    /// Veigar W - Dark Matter
    /// Radius: 112, Range: 950, Delay: 1.2s
    /// Note: Very long delay, best used on CC'd targets.
    /// </summary>
    public static CircularSkillshot VeigarW => new(Radius: 112, Range: 950, Delay: 1.2);

    /// <summary>
    /// Brand W - Pillar of Flame
    /// Radius: 260, Range: 900, Delay: 0.625s
    /// Note: Deals 25% bonus damage to ablaze targets.
    /// </summary>
    public static CircularSkillshot BrandW => new(Radius: 260, Range: 900, Delay: 0.625);

    /// <summary>
    /// Syndra W - Force of Will
    /// Radius: 225, Range: 925, Delay: 0.25s
    /// Note: Throws grabbed object. Slows enemies.
    /// </summary>
    public static CircularSkillshot SyndraW => new(Radius: 225, Range: 925, Delay: 0.25);

    /// <summary>
    /// Syndra Q - Dark Sphere
    /// Radius: 200, Range: 800, Delay: 0.6s
    /// Note: Creates a sphere that can be manipulated.
    /// </summary>
    public static CircularSkillshot SyndraQ => new(Radius: 200, Range: 800, Delay: 0.6);

    /// <summary>
    /// Lux E - Lucent Singularity
    /// Radius: 310, Range: 1100, Delay: Variable (can detonate early)
    /// Note: Slows enemies in zone. Using minimum delay.
    /// </summary>
    public static CircularSkillshot LuxE => new(Radius: 310, Range: 1100, Delay: 0.25);

    /// <summary>
    /// Orianna Q - Command: Attack (Ball destination)
    /// Radius: 175, Range: 825, Delay: 0.0s (ball travel time separate)
    /// Note: Ball travels at 1400 speed. This is the impact zone.
    /// </summary>
    public static CircularSkillshot OriannaQ => new(Radius: 175, Range: 825, Delay: 0.0);

    /// <summary>
    /// Cassiopeia W - Miasma
    /// Radius: 160, Range: 700, Delay: 0.4s
    /// Note: Grounds enemies, preventing dashes/flashes.
    /// </summary>
    public static CircularSkillshot CassiopeiaW => new(Radius: 160, Range: 700, Delay: 0.4);

    /// <summary>
    /// Viktor E - Death Ray (Aftershock)
    /// Radius: 90 (along line), Range: 1025, Delay: 1.0s
    /// Note: Delayed explosion along the laser path.
    /// </summary>
    public static CircularSkillshot ViktorEAftershock => new(Radius: 90, Range: 1025, Delay: 1.0);

    #endregion

    #region Tank/Fighter Abilities

    /// <summary>
    /// Cho'Gath Q - Rupture
    /// Radius: 175, Range: 950, Delay: 0.5s
    /// Note: Knocks up and slows.
    /// </summary>
    public static CircularSkillshot ChoGathQ => new(Radius: 175, Range: 950, Delay: 0.5);

    /// <summary>
    /// Maokai E - Sapling Toss
    /// Radius: 200 (explosion), Range: 1100, Delay: 0.25s
    /// Note: Saplings chase enemies in brush.
    /// </summary>
    public static CircularSkillshot MaokaiE => new(Radius: 200, Range: 1100, Delay: 0.25);

    /// <summary>
    /// Gragas Q - Barrel Roll
    /// Radius: 250, Range: 850, Delay: Variable (0.5-2.0s ferment time)
    /// Note: Damage increases with ferment time. Using minimum.
    /// </summary>
    public static CircularSkillshot GragasQ => new(Radius: 250, Range: 850, Delay: 0.5);

    /// <summary>
    /// Zac E - Elastic Slingshot
    /// Radius: 300, Range: 1200-1800 (charge), Delay: Variable
    /// Note: Knocks up enemies. Using max range.
    /// </summary>
    public static CircularSkillshot ZacE => new(Radius: 300, Range: 1800, Delay: 0.9);

    /// <summary>
    /// Sion Q - Decimating Smash
    /// Width: 400 (at max charge), Range: 750, Delay: 0.6-2.0s
    /// Note: Knock-up duration scales with charge. Using minimum.
    /// </summary>
    public static CircularSkillshot SionQ => new(Radius: 200, Range: 750, Delay: 0.6);

    #endregion

    #region Support/Control Abilities

    /// <summary>
    /// Ziggs W - Satchel Charge
    /// Radius: 150, Range: 1000, Delay: ~0.25s
    /// Note: Can be detonated early. Knocks back enemies and Ziggs.
    /// </summary>
    public static CircularSkillshot ZiggsW => new(Radius: 150, Range: 1000, Delay: 0.25);

    /// <summary>
    /// Ziggs E - Hexplosive Minefield
    /// Total radius: 250, Range: 900, Delay: 0.25s
    /// Note: Individual mines have 60 radius each.
    /// </summary>
    public static CircularSkillshot ZiggsE => new(Radius: 250, Range: 900, Delay: 0.25);

    /// <summary>
    /// Veigar E - Event Horizon (Cage)
    /// Radius: 375, Range: 725, Delay: 0.5s
    /// Note: Stuns enemies touching the edge. Interior is safe.
    /// </summary>
    public static CircularSkillshot VeigarE => new(Radius: 375, Range: 725, Delay: 0.5);

    /// <summary>
    /// Soraka E - Equinox
    /// Radius: 260, Range: 925, Delay: 1.5s (to root)
    /// Note: Silences immediately, roots after delay.
    /// </summary>
    public static CircularSkillshot SorakaE => new(Radius: 260, Range: 925, Delay: 1.5);

    /// <summary>
    /// Anivia R - Glacial Storm
    /// Radius: 200-400 (expands), Range: 750, Delay: 0.0s (instant)
    /// Note: Zone expands over 1.5s. Using max radius.
    /// </summary>
    public static CircularSkillshot AniviaR => new(Radius: 400, Range: 750, Delay: 0.0);

    /// <summary>
    /// Nami Q - Aqua Prison
    /// Radius: 180, Range: 875, Delay: 0.85s
    /// Note: Suspends enemies hit for 1.5s.
    /// </summary>
    public static CircularSkillshot NamiQ => new(Radius: 180, Range: 875, Delay: 0.85);

    /// <summary>
    /// Janna Q - Howling Gale (Uncharged)
    /// Radius: 120 (width), Range: 1750, Delay: 0.25s
    /// Note: Tornado travels. This is for immediate release.
    /// </summary>
    public static CircularSkillshot JannaQ => new(Radius: 120, Range: 1750, Delay: 0.25);

    #endregion

    #region Ultimate Abilities

    /// <summary>
    /// Ziggs R - Mega Inferno Bomb
    /// Inner radius: 275, Outer: 550, Range: 5300, Delay: ~1.5s (travel time)
    /// Note: Travel time varies with distance. Inner deals full damage.
    /// </summary>
    public static CircularSkillshot ZiggsR => new(Radius: 550, Range: 5300, Delay: 1.5);

    /// <summary>
    /// Xerath R - Rite of the Arcane
    /// Radius: 200 per shot, Range: 5000, Delay: 0.5s per shot
    /// Note: Fires 3-5 shots based on rank.
    /// </summary>
    public static CircularSkillshot XerathR => new(Radius: 200, Range: 5000, Delay: 0.5);

    /// <summary>
    /// Lissandra R - Frozen Tomb (Self-Cast AoE)
    /// Radius: 550, Range: 0 (self), Delay: 0.0s
    /// Note: Can also be cast on enemies with different effect.
    /// </summary>
    public static CircularSkillshot LissandraRSelf => new(Radius: 550, Range: 0, Delay: 0.0);

    /// <summary>
    /// Gangplank R - Cannon Barrage
    /// Radius: 600, Range: Global, Delay: Variable (waves)
    /// Note: 12 waves over 8 seconds. Each wave has 0.67s delay.
    /// </summary>
    public static CircularSkillshot GangplankR => new(Radius: 600, Range: 25000, Delay: 0.67);

    /// <summary>
    /// Karthus R - Requiem
    /// Radius: Global, Range: Global, Delay: 3.0s
    /// Note: Hits all enemy champions. Using 0 radius (targeted).
    /// </summary>
    public static CircularSkillshot KarthusR => new(Radius: 0, Range: 25000, Delay: 3.0);

    #endregion
}
