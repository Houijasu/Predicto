namespace Predicto.Models;

/// <summary>
/// Parameters defining a linear (line) skillshot.
/// Uses readonly record struct for stack allocation and zero GC pressure.
/// </summary>
/// <param name="Speed">Projectile speed in units per second</param>
/// <param name="Range">Maximum travel distance in units</param>
/// <param name="Width">Skillshot width (diameter) in units</param>
/// <param name="Delay">Cast time before projectile launches in seconds</param>
public readonly record struct LinearSkillshot(
    double Speed,
    double Range,
    double Width,
    double Delay,
    double Acceleration = 0.0)
{

    /// <summary>
    /// Threshold for considering a skillshot as "hitscan" (instant beam).
    /// Any speed above this value is treated as instant.
    /// </summary>
    private const double HitscanSpeedThreshold = 1_000_000;

    /// <summary>
    /// Returns true if this skillshot is a hitscan/instant beam (Speed >= threshold or MaxValue).
    /// Hitscan skillshots fire instantly after cast delay with no travel time.
    /// Examples: Lux R, Xerath Q.
    /// </summary>
    public bool IsHitscan => Speed >= HitscanSpeedThreshold || double.IsPositiveInfinity(Speed);

    /// <summary>
    /// Creates a LinearSkillshot with common defaults.
    /// </summary>
    public static LinearSkillshot Create(
        double speed,
        double range,
        double width = 70.0,
        double delay = 0.25) => new(speed, range, width, delay);

    #region Hook/Pull Abilities

    /// <summary>
    /// Blitzcrank Q - Rocket Grab
    /// Speed: 1800, Range: 1150, Width: 70, Delay: 0.25s
    /// Note: Stuns for 0.65s and pulls target to Blitzcrank.
    /// </summary>
    public static LinearSkillshot BlitzcrankQ => new(Speed: 1800, Range: 1150, Width: 70, Delay: 0.25);

    /// <summary>
    /// Thresh Q - Death Sentence
    /// Speed: 1900, Range: 1100, Width: 70, Delay: 0.5s
    /// Note: Can recast to dash to hooked target.
    /// </summary>
    public static LinearSkillshot ThreshQ => new(Speed: 1900, Range: 1100, Width: 70, Delay: 0.5);

    /// <summary>
    /// Nautilus Q - Dredge Line
    /// Speed: 2000, Range: 1100, Width: 90, Delay: 0.25s
    /// Note: Can hook terrain.
    /// </summary>
    public static LinearSkillshot NautilusQ => new(Speed: 2000, Range: 1100, Width: 90, Delay: 0.25);

    /// <summary>
    /// Pyke Q - Bone Skewer (Charged/Thrown)
    /// Speed: 2500, Range: 1100, Width: 70, Delay: 0.2s
    /// Note: Can be tapped for close-range stab instead.
    /// </summary>
    public static LinearSkillshot PykeQ => new(Speed: 2500, Range: 1100, Width: 70, Delay: 0.2);

    #endregion

    #region Bind/Root Abilities

    /// <summary>
    /// Morgana Q - Dark Binding
    /// Speed: 1200, Range: 1300, Width: 70, Delay: 0.25s
    /// Note: Roots for up to 3 seconds based on rank.
    /// </summary>
    public static LinearSkillshot MorganaQ => new(Speed: 1200, Range: 1300, Width: 70, Delay: 0.25);

    /// <summary>
    /// Lux Q - Light Binding
    /// Speed: 1200, Range: 1300, Width: 70, Delay: 0.25s
    /// Note: Can bind up to 2 targets.
    /// </summary>
    public static LinearSkillshot LuxQ => new(Speed: 1200, Range: 1300, Width: 70, Delay: 0.25);

    /// <summary>
    /// Neeko E - Tangle-Barbs
    /// Speed: 1300, Range: 1000, Width: 70, Delay: 0.25s
    /// Note: Root duration increases when passing through enemies.
    /// </summary>
    public static LinearSkillshot NeekoE => new(Speed: 1300, Range: 1000, Width: 70, Delay: 0.25);

    /// <summary>
    /// Zyra E - Grasping Roots
    /// Speed: 1150, Range: 1100, Width: 70, Delay: 0.25s
    /// Note: Spawns plants when passing through seeds.
    /// </summary>
    public static LinearSkillshot ZyraE => new(Speed: 1150, Range: 1100, Width: 70, Delay: 0.25);

    #endregion

    #region Damage/Poke Abilities

    /// <summary>
    /// Ezreal Q - Mystic Shot
    /// Speed: 2000, Range: 1200, Width: 60, Delay: 0.25s
    /// Note: Applies on-hit effects, reduces cooldowns by 1.5s on hit.
    /// </summary>
    public static LinearSkillshot EzrealQ => new(Speed: 2000, Range: 1200, Width: 60, Delay: 0.25);

    /// <summary>
    /// Nidalee Q - Javelin Toss (Human Form)
    /// Speed: 1300, Range: 1500, Width: 40, Delay: 0.25s
    /// Note: Damage increases with distance traveled (up to 200%).
    /// </summary>
    public static LinearSkillshot NidaleeQ => new(Speed: 1300, Range: 1500, Width: 40, Delay: 0.25);

    /// <summary>
    /// Xerath Q - Arcanopulse
    /// Speed: Instant (fires after charge), Range: 750-1400, Width: 100, Delay: 0.5-1.5s (charge time)
    /// Note: Range increases with charge time. Using 1200 range average.
    /// </summary>
    public static LinearSkillshot XerathQ => new(Speed: double.MaxValue, Range: 1200, Width: 100, Delay: 0.5);

    /// <summary>
    /// Varus Q - Piercing Arrow
    /// Speed: 1900, Range: 925-1625, Width: 70, Delay: 0-2.0s (charge time)
    /// Note: Damage and range increase with charge. Using max range.
    /// </summary>
    public static LinearSkillshot VarusQ => new(Speed: 1900, Range: 1625, Width: 70, Delay: 0.25);

    /// <summary>
    /// Jinx W - Zap!
    /// Speed: 3300, Range: 1500, Width: 60, Delay: 0.6s
    /// Note: Reveals and slows target.
    /// </summary>
    public static LinearSkillshot JinxW => new(Speed: 3300, Range: 1500, Width: 60, Delay: 0.6);

    /// <summary>
    /// Caitlyn Q - Piltover Peacemaker
    /// Speed: 2200, Range: 1300, Width: 90, Delay: 0.625s
    /// Note: Passes through enemies, reduced damage after first hit.
    /// </summary>
    public static LinearSkillshot CaitlynQ => new(Speed: 2200, Range: 1300, Width: 90, Delay: 0.625);

    /// <summary>
    /// Kai'Sa W - Void Seeker
    /// Speed: 1750, Range: 3000, Width: 100, Delay: 0.4s
    /// Note: Applies 2 stacks of Plasma, reveals target.
    /// </summary>
    public static LinearSkillshot KaiSaW => new(Speed: 1750, Range: 3000, Width: 100, Delay: 0.4);

    /// <summary>
    /// Zoe E - Sleepy Trouble Bubble
    /// Speed: 1850, Range: 800 (+max 650 terrain), Width: 50, Delay: 0.25s
    /// Note: Range extends through terrain. Can put enemies to sleep.
    /// </summary>
    public static LinearSkillshot ZoeE => new(Speed: 1850, Range: 800, Width: 50, Delay: 0.25);

    #endregion

    #region Ultimate Abilities

    /// <summary>
    /// Ashe R - Enchanted Crystal Arrow
    /// Speed: 1600, Range: Global, Width: 130, Delay: 0.25s
    /// Note: Stun duration scales with distance (1-3.5s).
    /// </summary>
    public static LinearSkillshot AsheR => new(Speed: 1600, Range: 25000, Width: 130, Delay: 0.25);

    /// <summary>
    /// Ezreal R - Trueshot Barrage
    /// Speed: 2000, Range: Global, Width: 160, Delay: 1.0s
    /// Note: Damage reduced per enemy hit (minimum 30%).
    /// </summary>
    public static LinearSkillshot EzrealR => new(Speed: 2000, Range: 25000, Width: 160, Delay: 1.0);

    /// <summary>
    /// Jinx R - Super Mega Death Rocket!
    /// Speed: 1700 (accelerates to 2600), Range: Global, Width: 140, Delay: 0.6s
    /// Note: Damage scales with missing health and distance. Using average speed.
    /// </summary>
    public static LinearSkillshot JinxR => new(Speed: 2150, Range: 25000, Width: 140, Delay: 0.6);

    /// <summary>
    /// Lux R - Final Spark
    /// Speed: Instant, Range: 3400, Width: 100, Delay: 1.0s
    /// Note: Fires after 1s charge animation.
    /// </summary>
    public static LinearSkillshot LuxR => new(Speed: double.MaxValue, Range: 3400, Width: 100, Delay: 1.0);

    /// <summary>
    /// Senna R - Dawning Shadow
    /// Speed: 20000, Range: Global, Width: 180 (center), Delay: 1.0s
    /// Note: Center beam deals damage, outer area shields allies.
    /// </summary>
    public static LinearSkillshot SennaR => new(Speed: 20000, Range: 25000, Width: 180, Delay: 1.0);

    #endregion
}
