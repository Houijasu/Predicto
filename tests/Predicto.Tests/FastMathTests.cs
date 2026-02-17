using MathNet.Spatial.Euclidean;
using Predicto.Physics;

namespace Predicto.Tests;

public class FastMathTests
{
    private const double Epsilon = 1e-10;

    #region LorentzianFalloff

    [Fact]
    public void LorentzianFalloff_Zero_Returns1()
    {
        Assert.Equal(1.0, FastMath.LorentzianFalloff(0));
    }

    [Theory]
    [InlineData(1.0, 0.5)]
    [InlineData(2.0, 0.2)]
    [InlineData(3.0, 0.1)]
    public void LorentzianFalloff_KnownValues(double x, double expected)
    {
        Assert.Equal(expected, FastMath.LorentzianFalloff(x), Epsilon);
    }

    [Fact]
    public void LorentzianFalloff_Symmetric()
    {
        Assert.Equal(
            FastMath.LorentzianFalloff(3.5),
            FastMath.LorentzianFalloff(-3.5),
            Epsilon);
    }

    [Theory]
    [InlineData(0, 2.0, 1.0)]
    [InlineData(2.0, 2.0, 0.5)]
    public void LorentzianFalloff_WithSigma_KnownValues(double x, double sigma, double expected)
    {
        Assert.Equal(expected, FastMath.LorentzianFalloff(x, sigma), Epsilon);
    }

    [Fact]
    public void LorentzianFalloff_AlwaysPositive()
    {
        for (double x = -100; x <= 100; x += 7.3)
        {
            Assert.True(FastMath.LorentzianFalloff(x) > 0);
        }
    }

    #endregion

    #region DistanceSquared

    [Fact]
    public void DistanceSquared_SamePoint_ReturnsZero()
    {
        var p = new Point2D(5, 10);
        Assert.Equal(0.0, FastMath.DistanceSquared(p, p), Epsilon);
    }

    [Fact]
    public void DistanceSquared_KnownTriangle()
    {
        // 3-4-5 triangle
        var a = new Point2D(0, 0);
        var b = new Point2D(3, 4);
        Assert.Equal(25.0, FastMath.DistanceSquared(a, b), Epsilon);
    }

    [Fact]
    public void DistanceSquared_Symmetric()
    {
        var a = new Point2D(1, 2);
        var b = new Point2D(4, 6);
        Assert.Equal(FastMath.DistanceSquared(a, b), FastMath.DistanceSquared(b, a), Epsilon);
    }

    #endregion

    #region PerpendicularDistance

    [Fact]
    public void PerpendicularDistance_PointOnLine_ReturnsZero()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0);
        var point = new Point2D(5, 0);
        Assert.Equal(0.0, FastMath.PerpendicularDistance(point, origin, dir), Epsilon);
    }

    [Fact]
    public void PerpendicularDistance_PointAboveLine_ReturnsNegative()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0); // right
        var point = new Point2D(5, 3); // above (positive Y)
        // Cross product: dx*dirY - dy*dirX = 5*0 - 3*1 = -3
        Assert.Equal(-3.0, FastMath.PerpendicularDistance(point, origin, dir), Epsilon);
    }

    [Fact]
    public void PerpendicularDistance_PointBelowLine_ReturnsPositive()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0); // right
        var point = new Point2D(5, -3); // below (negative Y)
        // Cross product: dx*dirY - dy*dirX = 5*0 - (-3)*1 = 3
        Assert.Equal(3.0, FastMath.PerpendicularDistance(point, origin, dir), Epsilon);
    }

    [Fact]
    public void AbsPerpendicularDistance_AlwaysPositive()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0);
        var point = new Point2D(5, -3);
        Assert.Equal(3.0, FastMath.AbsPerpendicularDistance(point, origin, dir), Epsilon);
    }

    #endregion

    #region ProjectOntoLine

    [Fact]
    public void ProjectOntoLine_PointAtOrigin_ReturnsZero()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0);
        Assert.Equal(0.0, FastMath.ProjectOntoLine(origin, origin, dir), Epsilon);
    }

    [Fact]
    public void ProjectOntoLine_PointAlongDirection_ReturnsDistance()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0);
        var point = new Point2D(7, 0);
        Assert.Equal(7.0, FastMath.ProjectOntoLine(point, origin, dir), Epsilon);
    }

    [Fact]
    public void ProjectOntoLine_PointPerpendicular_ReturnsZero()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0);
        var point = new Point2D(0, 5);
        Assert.Equal(0.0, FastMath.ProjectOntoLine(point, origin, dir), Epsilon);
    }

    [Fact]
    public void ProjectOntoLine_PointBehindOrigin_ReturnsNegative()
    {
        var origin = new Point2D(0, 0);
        var dir = new Vector2D(1, 0);
        var point = new Point2D(-3, 2);
        Assert.Equal(-3.0, FastMath.ProjectOntoLine(point, origin, dir), Epsilon);
    }

    #endregion

    #region Clamp

    [Theory]
    [InlineData(5, 0, 10, 5)]
    [InlineData(-1, 0, 10, 0)]
    [InlineData(15, 0, 10, 10)]
    [InlineData(0, 0, 10, 0)]
    [InlineData(10, 0, 10, 10)]
    public void Clamp_KnownValues(double value, double min, double max, double expected)
    {
        Assert.Equal(expected, FastMath.Clamp(value, min, max));
    }

    #endregion

    #region ClampMagnitude

    [Fact]
    public void ClampMagnitude_WithinLimit_Unchanged()
    {
        FastMath.ClampMagnitude(3, 4, 10, out var x, out var y);
        Assert.Equal(3.0, x, Epsilon);
        Assert.Equal(4.0, y, Epsilon);
    }

    [Fact]
    public void ClampMagnitude_ExceedsLimit_Scaled()
    {
        // (6, 8) has magnitude 10; clamp to 5
        FastMath.ClampMagnitude(6, 8, 5, out var x, out var y);
        double mag = Math.Sqrt((x * x) + (y * y));
        Assert.Equal(5.0, mag, Epsilon);
        // Direction preserved
        Assert.Equal(6.0 / 10.0 * 5.0, x, Epsilon);
        Assert.Equal(8.0 / 10.0 * 5.0, y, Epsilon);
    }

    [Fact]
    public void ClampMagnitude_ZeroVector_Unchanged()
    {
        FastMath.ClampMagnitude(0, 0, 5, out var x, out var y);
        Assert.Equal(0.0, x);
        Assert.Equal(0.0, y);
    }

    #endregion

    #region Normalize

    [Fact]
    public void Normalize_UnitVector_Unchanged()
    {
        FastMath.Normalize(1, 0, out var x, out var y);
        Assert.Equal(1.0, x, Epsilon);
        Assert.Equal(0.0, y, Epsilon);
    }

    [Fact]
    public void Normalize_ArbitraryVector_UnitLength()
    {
        FastMath.Normalize(3, 4, out var x, out var y);
        double mag = Math.Sqrt((x * x) + (y * y));
        Assert.Equal(1.0, mag, Epsilon);
    }

    [Fact]
    public void Normalize_ZeroVector_ReturnsZero()
    {
        FastMath.Normalize(0, 0, out var x, out var y);
        Assert.Equal(0.0, x);
        Assert.Equal(0.0, y);
    }

    #endregion

    #region Lerp

    [Theory]
    [InlineData(0, 10, 0, 0)]
    [InlineData(0, 10, 1, 10)]
    [InlineData(0, 10, 0.5, 5)]
    [InlineData(0, 10, 0.25, 2.5)]
    public void Lerp_KnownValues(double a, double b, double t, double expected)
    {
        Assert.Equal(expected, FastMath.Lerp(a, b, t), Epsilon);
    }

    #endregion

    #region LorentzianGradient

    [Fact]
    public void LorentzianGradient_AtOrigin_ReturnsZero()
    {
        FastMath.LorentzianGradient(0, 0, 1.0, 1.0, out var gx, out var gy);
        Assert.Equal(0.0, gx, Epsilon);
        Assert.Equal(0.0, gy, Epsilon);
    }

    [Fact]
    public void LorentzianGradient_PointsTowardCenter()
    {
        // Positive amplitude = attractive
        FastMath.LorentzianGradient(1, 0, 1.0, 1.0, out var gx, out _);
        // Gradient should be negative (pointing toward center)
        Assert.True(gx < 0);
    }

    [Fact]
    public void LorentzianGradient_Symmetric()
    {
        FastMath.LorentzianGradient(1, 0, 1.0, 1.0, out var gx1, out _);
        FastMath.LorentzianGradient(-1, 0, 1.0, 1.0, out var gx2, out _);
        Assert.Equal(-gx1, gx2, Epsilon);
    }

    #endregion

    #region DistanceToSegment

    [Fact]
    public void DistanceToSegment_PointOnSegment_ReturnsZero()
    {
        var a = new Point2D(0, 0);
        var b = new Point2D(10, 0);
        var point = new Point2D(5, 0);
        Assert.Equal(0.0, FastMath.DistanceToSegment(point, a, b), Epsilon);
    }

    [Fact]
    public void DistanceToSegment_PointPerpendicular_ReturnsDistance()
    {
        var a = new Point2D(0, 0);
        var b = new Point2D(10, 0);
        var point = new Point2D(5, 3);
        Assert.Equal(3.0, FastMath.DistanceToSegment(point, a, b), Epsilon);
    }

    [Fact]
    public void DistanceToSegment_PointBeyondEndpoint_ReturnsDistanceToEndpoint()
    {
        var a = new Point2D(0, 0);
        var b = new Point2D(10, 0);
        var point = new Point2D(13, 4); // closest to b
        Assert.Equal(5.0, FastMath.DistanceToSegment(point, a, b), Epsilon);
    }

    #endregion

    #region GagongOffsetTables

    [Fact]
    public void GagongOffsetTables_CorrectLength()
    {
        Assert.Equal(Constants.GagongAngleSteps + 1, FastMath.GagongOffsetCos.Length);
        Assert.Equal(Constants.GagongAngleSteps + 1, FastMath.GagongOffsetSin.Length);
    }

    [Fact]
    public void GagongOffsetTables_FirstEntry_IsNegativeHalfPi()
    {
        // First entry = cos(-π/2) = 0, sin(-π/2) = -1
        Assert.Equal(0.0, FastMath.GagongOffsetCos[0], Epsilon);
        Assert.Equal(-1.0, FastMath.GagongOffsetSin[0], Epsilon);
    }

    [Fact]
    public void GagongOffsetTables_LastEntry_IsPositiveHalfPi()
    {
        int last = Constants.GagongAngleSteps;
        // Last entry = cos(π/2) = 0, sin(π/2) = 1
        Assert.Equal(0.0, FastMath.GagongOffsetCos[last], Epsilon);
        Assert.Equal(1.0, FastMath.GagongOffsetSin[last], Epsilon);
    }

    [Fact]
    public void GagongOffsetTables_UnitCircle_SinSqPlusCosSqEqualsOne()
    {
        for (int i = 0; i <= Constants.GagongAngleSteps; i++)
        {
            double c = FastMath.GagongOffsetCos[i];
            double s = FastMath.GagongOffsetSin[i];
            Assert.Equal(1.0, (c * c) + (s * s), Epsilon);
        }
    }

    #endregion
}
