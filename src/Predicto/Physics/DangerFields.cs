using System.Runtime.CompilerServices;
using MathNet.Spatial.Euclidean;
using Predicto.Models;

namespace Predicto.Physics;

/// <summary>
/// Interface for danger potential fields that represent threats from skillshots.
/// Implementations compute the repulsive force that pushes targets away from danger.
/// </summary>
public interface IDangerField
{
    /// <summary>
    /// Computes the danger potential at a given position and time.
    /// Higher values indicate more dangerous positions.
    /// </summary>
    /// <param name="position">Position to evaluate</param>
    /// <param name="time">Current simulation time</param>
    /// <returns>Potential value (0 = safe, higher = more dangerous)</returns>
    double Potential(Point2D position, double time);

    /// <summary>
    /// Computes the gradient of the danger field (direction of steepest ascent).
    /// The force on the target is F = -∇U (points away from danger).
    /// </summary>
    /// <param name="position">Position to evaluate</param>
    /// <param name="time">Current simulation time</param>
    /// <param name="gradX">X component of gradient</param>
    /// <param name="gradY">Y component of gradient</param>
    void Gradient(Point2D position, double time, out double gradX, out double gradY);

    /// <summary>
    /// Computes the repulsive force on a target at the given position.
    /// This is the negative gradient: F = -∇U.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void Force(Point2D position, double time, out double forceX, out double forceY)
    {
        Gradient(position, time, out double gx, out double gy);
        forceX = -gx;
        forceY = -gy;
    }
}

/// <summary>
/// Danger field for linear (projectile) skillshots.
/// Creates a moving tube of danger that follows the projectile path.
/// Uses Lorentzian falloff for fast computation: U = A / (1 + d²/σ²)
/// </summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields",
    Justification = "Performance-critical readonly struct uses public fields for zero-allocation physics simulation")]
public readonly struct LinearDangerField : IDangerField
{
    /// <summary>Caster position (skillshot origin)</summary>
    public readonly Point2D Origin;

    /// <summary>Normalized skillshot direction</summary>
    public readonly Vector2D Direction;

    /// <summary>Skillshot travel speed (units/second)</summary>
    public readonly double Speed;

    /// <summary>Cast delay before projectile starts moving</summary>
    public readonly double Delay;

    /// <summary>Maximum range of the skillshot</summary>
    public readonly double Range;

    /// <summary>Danger amplitude (peak potential value)</summary>
    public readonly double Amplitude;

    /// <summary>Squared falloff width for potential calculation</summary>
    public readonly double SigmaSq;

    /// <summary>
    /// Creates a danger field for a linear skillshot.
    /// </summary>
    /// <param name="origin">Caster position</param>
    /// <param name="direction">Skillshot direction (will be normalized)</param>
    /// <param name="skillshot">Skillshot parameters</param>
    /// <param name="amplitude">Danger amplitude (default: 1000)</param>
    /// <param name="falloffMultiplier">Falloff width as multiple of skillshot width (default: 2)</param>
    public LinearDangerField(
        Point2D origin,
        Vector2D direction,
        LinearSkillshot skillshot,
        double amplitude = Constants.DefaultDangerAmplitude,
        double falloffMultiplier = Constants.DangerFalloffMultiplier)
    {
        Origin = origin;
        Direction = direction.Normalize();
        Speed = skillshot.Speed;
        Delay = skillshot.Delay;
        Range = skillshot.Range;
        Amplitude = amplitude;

        // Falloff width based on skillshot width
        double sigma = skillshot.Width * falloffMultiplier;
        SigmaSq = sigma * sigma;
    }

    /// <summary>
    /// Gets the current position of the projectile at the given time.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Point2D GetProjectilePosition(double time)
    {
        double flightTime = Math.Max(0, time - Delay);
        double distance = Math.Min(Speed * flightTime, Range);
        return Origin + (Direction * distance);
    }

    /// <summary>
    /// Checks if the skillshot is still active (hasn't reached max range).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsActive(double time)
    {
        double flightTime = Math.Max(0, time - Delay);
        return Speed * flightTime < Range;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public double Potential(Point2D position, double time)
    {
        if (!IsActive(time))
            return 0;

        // Get perpendicular distance to skillshot path
        // The danger is highest near the projectile's current position
        // and along the path ahead of it

        Point2D projectilePos = GetProjectilePosition(time);

        // Vector from projectile to target
        var displacement = position - projectilePos;

        // Project onto skillshot direction to get "along path" distance
        double alongPath = displacement.DotProduct(Direction);

        // Only dangerous ahead of projectile (or slightly behind due to width)
        if (alongPath < -SigmaSq * 0.5)
            return 0;

        // Perpendicular distance
        double perpDist = FastMath.AbsPerpendicularDistance(position, projectilePos, Direction);

        // Lorentzian potential based on perpendicular distance
        double potential = Amplitude * FastMath.LorentzianFalloff(perpDist * perpDist / SigmaSq);

        // Attenuate based on distance along path (danger decreases further ahead)
        if (alongPath > 0)
        {
            double alongFactor = FastMath.LorentzianFalloff(alongPath, Range * 0.5);
            potential *= alongFactor;
        }

        return potential;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Gradient(Point2D position, double time, out double gradX, out double gradY)
    {
        if (!IsActive(time))
        {
            gradX = 0;
            gradY = 0;
            return;
        }

        Point2D projectilePos = GetProjectilePosition(time);

        // Vector from projectile to target
        var displacement = position - projectilePos;

        // Project onto skillshot direction
        double alongPath = displacement.DotProduct(Direction);

        if (alongPath < -SigmaSq * 0.5)
        {
            gradX = 0;
            gradY = 0;
            return;
        }

        // Perpendicular component (direction away from line)
        var perp = displacement - (Direction * alongPath);
        double perpDistSq = perp.DotProduct(perp);

        // Gradient of Lorentzian: d/dr [A/(1 + r²/σ²)] = -2A*r / (σ² * (1 + r²/σ²)²)
        double normalizedDistSq = perpDistSq / SigmaSq;
        double denom = 1.0 + normalizedDistSq;
        double denomSq = denom * denom;
        double factor = -2.0 * Amplitude / (SigmaSq * denomSq);

        // Apply along-path attenuation
        double alongFactor = 1.0;
        if (alongPath > 0)
        {
            alongFactor = FastMath.LorentzianFalloff(alongPath, Range * 0.5);
        }

        gradX = factor * perp.X * alongFactor;
        gradY = factor * perp.Y * alongFactor;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Force(Point2D position, double time, out double forceX, out double forceY)
    {
        Gradient(position, time, out double gx, out double gy);
        forceX = -gx;
        forceY = -gy;
    }
}

/// <summary>
/// Danger field for circular (ground-targeted) skillshots.
/// Creates a radial danger zone that activates at detonation time.
/// </summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields",
    Justification = "Performance-critical readonly struct uses public fields for zero-allocation physics simulation")]
public readonly struct CircularDangerField : IDangerField
{
    /// <summary>Center of the circular ability</summary>
    public readonly Point2D Center;

    /// <summary>Radius of the danger zone</summary>
    public readonly double Radius;

    /// <summary>Time when the ability detonates</summary>
    public readonly double DetonationTime;

    /// <summary>Danger amplitude</summary>
    public readonly double Amplitude;

    /// <summary>Squared falloff width</summary>
    public readonly double SigmaSq;

    /// <summary>How early the danger starts ramping up (anticipation window)</summary>
    public readonly double AnticipationWindow;

    /// <summary>
    /// Creates a danger field for a circular skillshot.
    /// </summary>
    /// <param name="center">Center of the ability</param>
    /// <param name="skillshot">Circular skillshot parameters</param>
    /// <param name="amplitude">Danger amplitude</param>
    /// <param name="falloffMultiplier">Falloff as multiple of radius</param>
    /// <param name="anticipationWindow">Time before detonation when danger starts</param>
    public CircularDangerField(
        Point2D center,
        CircularSkillshot skillshot,
        double amplitude = Constants.DefaultDangerAmplitude,
        double falloffMultiplier = Constants.DefaultCircularDangerFalloff,
        double anticipationWindow = Constants.DefaultAnticipationWindow)
    {
        Center = center;
        Radius = skillshot.Radius;
        DetonationTime = skillshot.Delay;
        Amplitude = amplitude;
        AnticipationWindow = anticipationWindow;

        double sigma = skillshot.Radius * falloffMultiplier;
        SigmaSq = sigma * sigma;
    }

    /// <summary>
    /// Gets the time-dependent amplitude multiplier.
    /// Danger ramps up as detonation approaches.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private double GetTimeMultiplier(double time)
    {
        double timeUntilDetonation = DetonationTime - time;

        // After detonation, danger drops rapidly
        if (timeUntilDetonation < 0)
            return FastMath.LorentzianFalloff(-timeUntilDetonation, 0.1);

        // Before anticipation window, no danger
        if (timeUntilDetonation > AnticipationWindow)
            return 0;

        // Ramp up danger as detonation approaches
        double progress = 1.0 - (timeUntilDetonation / AnticipationWindow);
        return progress * progress; // Quadratic ramp
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public double Potential(Point2D position, double time)
    {
        double timeMult = GetTimeMultiplier(time);
        if (timeMult < Constants.Epsilon)
            return 0;

        var displacement = position - Center;
        double distSq = displacement.DotProduct(displacement);

        // Lorentzian potential centered on ability
        double potential = Amplitude * FastMath.LorentzianFalloff(distSq / SigmaSq);

        return potential * timeMult;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Gradient(Point2D position, double time, out double gradX, out double gradY)
    {
        double timeMult = GetTimeMultiplier(time);
        if (timeMult < Constants.Epsilon)
        {
            gradX = 0;
            gradY = 0;
            return;
        }

        var displacement = position - Center;
        FastMath.LorentzianGradient(displacement.X, displacement.Y, Amplitude * timeMult, SigmaSq, out gradX, out gradY);
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Force(Point2D position, double time, out double forceX, out double forceY)
    {
        Gradient(position, time, out double gx, out double gy);
        forceX = -gx;
        forceY = -gy;
    }
}
