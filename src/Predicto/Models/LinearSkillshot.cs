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
}
