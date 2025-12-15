using System.Runtime.CompilerServices;
using MathNet.Spatial.Euclidean;

namespace Predicto.Physics;

/// <summary>
/// High-performance, zero-allocation math utilities for physics calculations.
/// All operations are designed to work with stack-allocated structs and avoid heap allocations.
/// </summary>
public static class FastMath
{
    /// <summary>
    /// Fast approximation of 1/(1 + x²) - Lorentzian/Cauchy distribution.
    /// Much faster than exp(-x²) and provides similar falloff behavior.
    /// </summary>
    /// <param name="x">Input value</param>
    /// <returns>Value in range (0, 1]</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double LorentzianFalloff(double x)
        => 1.0 / (1.0 + x * x);

    /// <summary>
    /// Fast approximation of 1/(1 + x²/σ²) with configurable width.
    /// </summary>
    /// <param name="x">Input value (distance)</param>
    /// <param name="sigma">Falloff width parameter</param>
    /// <returns>Value in range (0, 1]</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double LorentzianFalloff(double x, double sigma)
    {
        double normalized = x / sigma;
        return 1.0 / (1.0 + normalized * normalized);
    }

    /// <summary>
    /// Computes squared distance between two points without square root.
    /// Use this when comparing distances to avoid expensive sqrt operations.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double DistanceSquared(Point2D a, Point2D b)
    {
        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        return dx * dx + dy * dy;
    }

    /// <summary>
    /// Computes the perpendicular (signed) distance from a point to a line.
    /// Positive = left of line direction, negative = right of line direction.
    /// </summary>
    /// <param name="point">The point to measure from</param>
    /// <param name="lineOrigin">A point on the line</param>
    /// <param name="lineDirection">Normalized direction vector of the line</param>
    /// <returns>Signed perpendicular distance</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double PerpendicularDistance(Point2D point, Point2D lineOrigin, Vector2D lineDirection)
    {
        // Vector from line origin to point
        double dx = point.X - lineOrigin.X;
        double dy = point.Y - lineOrigin.Y;

        // Cross product gives perpendicular distance (2D cross product = scalar)
        // cross = dx * dirY - dy * dirX
        return dx * lineDirection.Y - dy * lineDirection.X;
    }

    /// <summary>
    /// Computes the absolute perpendicular distance from a point to a line.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double AbsPerpendicularDistance(Point2D point, Point2D lineOrigin, Vector2D lineDirection)
        => Math.Abs(PerpendicularDistance(point, lineOrigin, lineDirection));

    /// <summary>
    /// Projects a point onto a line and returns the parameter t along the line.
    /// Result: lineOrigin + t * lineDirection = closest point on line.
    /// </summary>
    /// <param name="point">The point to project</param>
    /// <param name="lineOrigin">A point on the line</param>
    /// <param name="lineDirection">Normalized direction vector of the line</param>
    /// <returns>Parameter t (can be negative or > 1 for points beyond segment)</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double ProjectOntoLine(Point2D point, Point2D lineOrigin, Vector2D lineDirection)
    {
        double dx = point.X - lineOrigin.X;
        double dy = point.Y - lineOrigin.Y;

        // Dot product with normalized direction gives projection length
        return dx * lineDirection.X + dy * lineDirection.Y;
    }

    /// <summary>
    /// Clamps a value between min and max bounds.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Clamp(double value, double min, double max)
        => value < min ? min : (value > max ? max : value);

    /// <summary>
    /// Clamps the magnitude of a vector without allocating.
    /// Returns the clamped vector components.
    /// </summary>
    /// <param name="x">X component</param>
    /// <param name="y">Y component</param>
    /// <param name="maxMagnitude">Maximum allowed magnitude</param>
    /// <param name="outX">Clamped X component</param>
    /// <param name="outY">Clamped Y component</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ClampMagnitude(double x, double y, double maxMagnitude, out double outX, out double outY)
    {
        double magnitudeSq = x * x + y * y;
        double maxSq = maxMagnitude * maxMagnitude;

        if (magnitudeSq <= maxSq)
        {
            outX = x;
            outY = y;
            return;
        }

        double scale = maxMagnitude / Math.Sqrt(magnitudeSq);
        outX = x * scale;
        outY = y * scale;
    }

    /// <summary>
    /// Normalizes a vector in-place, returning components.
    /// If the vector is zero-length, returns (0, 0).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Normalize(double x, double y, out double outX, out double outY)
    {
        double magnitudeSq = x * x + y * y;

        if (magnitudeSq < Constants.Epsilon * Constants.Epsilon)
        {
            outX = 0;
            outY = 0;
            return;
        }

        double invMagnitude = 1.0 / Math.Sqrt(magnitudeSq);
        outX = x * invMagnitude;
        outY = y * invMagnitude;
    }

    /// <summary>
    /// Linear interpolation between two values.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Lerp(double a, double b, double t)
        => a + (b - a) * t;

    /// <summary>
    /// Computes the gradient of the Lorentzian potential field.
    /// For U(r) = A / (1 + r²/σ²), the gradient is:
    /// ∇U = -2A * r / (σ² * (1 + r²/σ²)²)
    /// This points toward the center (attractive). Negate for repulsive.
    /// </summary>
    /// <param name="dx">X displacement from field center</param>
    /// <param name="dy">Y displacement from field center</param>
    /// <param name="amplitude">Field amplitude A</param>
    /// <param name="sigmaSq">Squared falloff width σ²</param>
    /// <param name="gradX">Output gradient X component</param>
    /// <param name="gradY">Output gradient Y component</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void LorentzianGradient(
        double dx, double dy,
        double amplitude, double sigmaSq,
        out double gradX, out double gradY)
    {
        double rSq = dx * dx + dy * dy;
        double denom = 1.0 + rSq / sigmaSq;
        double denomSq = denom * denom;

        // Gradient magnitude factor: -2A / (σ² * denom²)
        double factor = -2.0 * amplitude / (sigmaSq * denomSq);

        gradX = factor * dx;
        gradY = factor * dy;
    }
}
