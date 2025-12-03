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
    double Delay)
{
    /// <summary>
    /// Creates a LinearSkillshot with common defaults.
    /// </summary>
    public static LinearSkillshot Create(
        double speed,
        double range,
        double width = 70.0,
        double delay = 0.25) => new(speed, range, width, delay);

    /// <summary>
    /// Blitzcrank Q - Rocket Grab
    /// Speed: 1800
    /// Range: 1150
    /// Width: 70
    /// Delay: 0.25s (cast time)
    /// Note: Stuns for 0.65s and pulls target to 75 units in front of Blitzcrank.
    /// </summary>
    public static LinearSkillshot BlitzcrankQ => new(Speed: 1800, Range: 1150, Width: 70, Delay: 0.25);
}
