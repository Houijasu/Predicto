using Predicto.Solvers;

namespace Predicto.Tests;

public class ZeroAllocRootFinderTests
{
    private const double Accuracy = 1e-12;
    private const int MaxIterations = 100;

    #region Test function structs

    /// <summary>
    /// f(x) = x² - 4, roots at x = ±2
    /// </summary>
    private struct QuadraticFunc : IDoubleFunction
    {
        public double Evaluate(double x) => (x * x) - 4;
    }

    /// <summary>
    /// f'(x) = 2x (derivative of x² - 4)
    /// </summary>
    private struct QuadraticDerivative : IDoubleFunction
    {
        public double Evaluate(double x) => 2 * x;
    }

    /// <summary>
    /// f(x) = sin(x), roots at x = nπ
    /// </summary>
    private struct SinFunc : IDoubleFunction
    {
        public double Evaluate(double x) => Math.Sin(x);
    }

    /// <summary>
    /// f'(x) = cos(x) (derivative of sin(x))
    /// </summary>
    private struct CosFunc : IDoubleFunction
    {
        public double Evaluate(double x) => Math.Cos(x);
    }

    /// <summary>
    /// f(x) = x³ - x, roots at x = -1, 0, 1
    /// </summary>
    private struct CubicFunc : IDoubleFunction
    {
        public double Evaluate(double x) => (x * x * x) - x;
    }

    /// <summary>
    /// f'(x) = 3x² - 1
    /// </summary>
    private struct CubicDerivative : IDoubleFunction
    {
        public double Evaluate(double x) => (3 * x * x) - 1;
    }

    /// <summary>
    /// f(x) = x + 5, root at x = -5 (no sign change in [0, 10])
    /// </summary>
    private struct NoRootInBracketFunc : IDoubleFunction
    {
        public double Evaluate(double x) => x + 5;
    }

    /// <summary>
    /// f(x) = x³, root at x = 0 with flat derivative near root.
    /// </summary>
    private struct FlatCubicFunc : IDoubleFunction
    {
        public double Evaluate(double x) => x * x * x;
    }

    /// <summary>
    /// f'(x) = 3x².
    /// </summary>
    private struct FlatCubicDerivative : IDoubleFunction
    {
        public double Evaluate(double x) => 3 * x * x;
    }

    /// <summary>
    /// f(x) = (x - 1)(x - 2), roots at x = 1 and x = 2.
    /// f(0) and f(3) are both positive, which exercises same-sign subdivision scanning.
    /// </summary>
    private struct TwoRootsSameSignBoundsFunc : IDoubleFunction
    {
        public double Evaluate(double x) => (x - 1) * (x - 2);
    }

    /// <summary>
    /// f'(x) = 2x - 3.
    /// </summary>
    private struct TwoRootsSameSignBoundsDerivative : IDoubleFunction
    {
        public double Evaluate(double x) => (2 * x) - 3;
    }

    #endregion

    #region TryBisection

    [Fact]
    public void Bisection_QuadraticFunction_FindsPositiveRoot()
    {
        var f = new QuadraticFunc();
        bool found = ZeroAllocRootFinder.TryBisection(ref f, 0, 5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void Bisection_QuadraticFunction_FindsNegativeRoot()
    {
        var f = new QuadraticFunc();
        bool found = ZeroAllocRootFinder.TryBisection(ref f, -5, 0, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(-2.0, root, Accuracy);
    }

    [Fact]
    public void Bisection_RootAtBoundary_ReturnsImmediately()
    {
        var f = new QuadraticFunc();
        // lo = 2 is exactly a root
        bool found = ZeroAllocRootFinder.TryBisection(ref f, 2, 5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void Bisection_NoSignChange_ReturnsFalse()
    {
        var f = new NoRootInBracketFunc();
        bool found = ZeroAllocRootFinder.TryBisection(ref f, 0, 10, Accuracy, MaxIterations, out _);

        Assert.False(found);
    }

    [Fact]
    public void Bisection_SinFunction_FindsRoot()
    {
        var f = new SinFunc();
        bool found = ZeroAllocRootFinder.TryBisection(ref f, 2, 4, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(Math.PI, root, Accuracy);
    }

    #endregion

    #region TryBrent

    [Fact]
    public void Brent_QuadraticFunction_FindsRoot()
    {
        var f = new QuadraticFunc();
        bool found = ZeroAllocRootFinder.TryBrent(ref f, 0, 5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void Brent_SinFunction_FindsRoot()
    {
        var f = new SinFunc();
        bool found = ZeroAllocRootFinder.TryBrent(ref f, 2, 4, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(Math.PI, root, Accuracy);
    }

    [Fact]
    public void Brent_RootAtBoundary_ReturnsImmediately()
    {
        var f = new QuadraticFunc();
        bool found = ZeroAllocRootFinder.TryBrent(ref f, 2, 5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void Brent_NoSignChange_ReturnsFalse()
    {
        var f = new NoRootInBracketFunc();
        bool found = ZeroAllocRootFinder.TryBrent(ref f, 0, 10, Accuracy, MaxIterations, out _);

        Assert.False(found);
    }

    [Fact]
    public void Brent_CubicFunction_FindsRootAtZero()
    {
        var f = new CubicFunc();
        bool found = ZeroAllocRootFinder.TryBrent(ref f, -0.5, 0.5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(0.0, root, Accuracy);
    }

    #endregion

    #region TryNewtonRaphson

    [Fact]
    public void NewtonRaphson_QuadraticFunction_FindsRoot()
    {
        var f = new QuadraticFunc();
        var df = new QuadraticDerivative();
        bool found = ZeroAllocRootFinder.TryNewtonRaphson(ref f, ref df, 3, 0, 5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void NewtonRaphson_SinFunction_FindsRoot()
    {
        var f = new SinFunc();
        var df = new CosFunc();
        bool found = ZeroAllocRootFinder.TryNewtonRaphson(ref f, ref df, 3, 2, 4, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(Math.PI, root, Accuracy);
    }

    [Fact]
    public void NewtonRaphson_GuessIsRoot_ReturnsImmediately()
    {
        var f = new QuadraticFunc();
        var df = new QuadraticDerivative();
        bool found = ZeroAllocRootFinder.TryNewtonRaphson(ref f, ref df, 2, 0, 5, Accuracy, MaxIterations, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    #endregion

    #region TryRobustNewtonRaphson

    [Fact]
    public void RobustNewton_QuadraticFunction_FindsRoot()
    {
        var f = new QuadraticFunc();
        var df = new QuadraticDerivative();
        bool found = ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f, ref df, 0, 5, Accuracy, MaxIterations, 20, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void RobustNewton_SinFunction_FindsRoot()
    {
        var f = new SinFunc();
        var df = new CosFunc();
        bool found = ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f, ref df, 2, 4, Accuracy, MaxIterations, 20, out var root);

        Assert.True(found);
        Assert.Equal(Math.PI, root, Accuracy);
    }

    [Fact]
    public void RobustNewton_RootAtBoundary_ReturnsImmediately()
    {
        var f = new QuadraticFunc();
        var df = new QuadraticDerivative();
        bool found = ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f, ref df, 2, 5, Accuracy, MaxIterations, 20, out var root);

        Assert.True(found);
        Assert.Equal(2.0, root, Accuracy);
    }

    [Fact]
    public void RobustNewton_CubicFunction_FindsRoot()
    {
        var f = new CubicFunc();
        var df = new CubicDerivative();
        bool found = ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f, ref df, 0.5, 1.5, Accuracy, MaxIterations, 20, out var root);

        Assert.True(found);
        Assert.Equal(1.0, root, Accuracy);
    }

    [Fact]
    public void RobustNewton_FlatDerivativeAtRoot_StillConverges()
    {
        var f = new FlatCubicFunc();
        var df = new FlatCubicDerivative();

        bool found = ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f, ref df, -1, 1, Accuracy, MaxIterations, 20, out var root);

        Assert.True(found);
        Assert.True(Math.Abs(root) < 1e-8);
        Assert.True(Math.Abs(f.Evaluate(root)) < 1e-10);
    }

    [Fact]
    public void RobustNewton_SameSignBounds_UsesSubdivisionAndFindsRoot()
    {
        var f = new TwoRootsSameSignBoundsFunc();
        var df = new TwoRootsSameSignBoundsDerivative();

        bool found = ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f, ref df, 0, 3, Accuracy, MaxIterations, 20, out var root);

        Assert.True(found);
        Assert.True(Math.Abs(root - 1.0) < 1e-8 || Math.Abs(root - 2.0) < 1e-8);
        Assert.True(Math.Abs(f.Evaluate(root)) < 1e-10);
    }

    #endregion

    #region Consistency checks

    [Fact]
    public void AllMethods_QuadraticFunction_FindSameRoot()
    {
        var f1 = new QuadraticFunc();
        var f2 = new QuadraticFunc();
        var f3 = new QuadraticFunc();
        var f4 = new QuadraticFunc();
        var df3 = new QuadraticDerivative();
        var df4 = new QuadraticDerivative();

        ZeroAllocRootFinder.TryBisection(ref f1, 0, 5, Accuracy, MaxIterations, out var bisectionRoot);
        ZeroAllocRootFinder.TryBrent(ref f2, 0, 5, Accuracy, MaxIterations, out var brentRoot);
        ZeroAllocRootFinder.TryNewtonRaphson(ref f3, ref df3, 3, 0, 5, Accuracy, MaxIterations, out var newtonRoot);
        ZeroAllocRootFinder.TryRobustNewtonRaphson(ref f4, ref df4, 0, 5, Accuracy, MaxIterations, 20, out var robustRoot);

        Assert.Equal(2.0, bisectionRoot, 1e-8);
        Assert.Equal(2.0, brentRoot, 1e-8);
        Assert.Equal(2.0, newtonRoot, 1e-8);
        Assert.Equal(2.0, robustRoot, 1e-8);
    }

    [Fact]
    public void Bisection_HighAccuracy_AchievesTargetPrecision()
    {
        var f = new QuadraticFunc();
        double tightAccuracy = 1e-15;
        bool found = ZeroAllocRootFinder.TryBisection(ref f, 0, 5, tightAccuracy, 200, out var root);

        Assert.True(found);
        // Verify function value is near zero
        Assert.True(Math.Abs((root * root) - 4) < 1e-12);
    }

    #endregion
}
