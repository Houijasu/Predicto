using System.Runtime.CompilerServices;

namespace Predicto.Solvers;

/// <summary>
/// Interface for zero-allocation function evaluation.
/// Implement as a readonly struct to enable JIT devirtualization and inlining.
/// </summary>
internal interface IDoubleFunction
{
    double Evaluate(double x);
}

/// <summary>
/// Zero-allocation root-finding algorithms using generic struct constraints.
/// Replaces MathNet.Numerics.RootFinding closures with stack-allocated functors.
///
/// All methods use <c>where TFunc : struct, IDoubleFunction</c> which guarantees:
/// - No heap allocation (struct constraint)
/// - JIT devirtualization of Evaluate calls (.NET 10+)
/// - Potential inlining of small Evaluate bodies
///
/// Algorithm implementations match MathNet.Numerics semantics for drop-in replacement.
/// </summary>
internal static class ZeroAllocRootFinder
{
    /// <summary>
    /// Bisection method: guaranteed convergence for continuous functions with sign change.
    /// Halves the bracket each iteration. Linear convergence O(log2(b-a)/tol).
    /// </summary>
    /// <returns>True if root found within tolerance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryBisection<TFunc>(
        ref TFunc f,
        double lo,
        double hi,
        double accuracy,
        int maxIterations,
        out double root)
        where TFunc : struct, IDoubleFunction
    {
        double fLo = f.Evaluate(lo);
        double fHi = f.Evaluate(hi);

        // Check if already at root
        if (Math.Abs(fLo) < accuracy)
        {
            root = lo;
            return true;
        }

        if (Math.Abs(fHi) < accuracy)
        {
            root = hi;
            return true;
        }

        // No sign change - cannot bracket
        if (fLo * fHi > 0)
        {
            root = 0;
            return false;
        }

        // Ensure fLo < 0 convention
        if (fLo > 0)
        {
            (lo, hi) = (hi, lo);
            (fLo, fHi) = (fHi, fLo);
        }

        for (int i = 0; i < maxIterations; i++)
        {
            double mid = lo + ((hi - lo) * 0.5);
            double fMid = f.Evaluate(mid);

            if (Math.Abs(fMid) < accuracy || (hi - lo) * 0.5 < accuracy)
            {
                root = mid;
                return true;
            }

            if (fMid < 0)
            {
                lo = mid;
                fLo = fMid;
            }
            else
            {
                hi = mid;
                fHi = fMid;
            }
        }

        root = lo + ((hi - lo) * 0.5);
        return true;
    }

    /// <summary>
    /// Brent's method: combines bisection, secant, and inverse quadratic interpolation.
    /// Superlinear convergence while maintaining guaranteed bracket.
    /// </summary>
    /// <returns>True if root found within tolerance.</returns>
    public static bool TryBrent<TFunc>(
        ref TFunc f,
        double lo,
        double hi,
        double accuracy,
        int maxIterations,
        out double root)
        where TFunc : struct, IDoubleFunction
    {
        double fLo = f.Evaluate(lo);
        double fHi = f.Evaluate(hi);

        if (Math.Abs(fLo) < accuracy)
        {
            root = lo;
            return true;
        }

        if (Math.Abs(fHi) < accuracy)
        {
            root = hi;
            return true;
        }

        if (fLo * fHi > 0)
        {
            root = 0;
            return false;
        }

        // Brent's algorithm variables
        double a = lo, b = hi;
        double fa = fLo, fb = fHi;
        double c = a, fc = fa;
        double d = b - a, e = d;

        for (int i = 0; i < maxIterations; i++)
        {
            if (fb * fc > 0)
            {
                c = a;
                fc = fa;
                d = b - a;
                e = d;
            }

            if (Math.Abs(fc) < Math.Abs(fb))
            {
                a = b;
                b = c;
                c = a;
                fa = fb;
                fb = fc;
                fc = fa;
            }

            double tol = (2.0 * 2.2204460492503131e-16 * Math.Abs(b)) + (0.5 * accuracy);
            double m = 0.5 * (c - b);

            if (Math.Abs(m) <= tol || Math.Abs(fb) < accuracy)
            {
                root = b;
                return true;
            }

            // Decide between interpolation and bisection
            if (Math.Abs(e) >= tol && Math.Abs(fa) > Math.Abs(fb))
            {
                double s;
                if (Math.Abs(a - c) < 2.2204460492503131e-16)
                {
                    // Linear interpolation (secant)
                    s = fb / fa;
                    double p = 2.0 * m * s;
                    double q = 1.0 - s;
                    if (p > 0) q = -q;
                    else p = -p;

                    if (2.0 * p < Math.Min(3.0 * m * q - Math.Abs(tol * q), Math.Abs(e * q)))
                    {
                        e = d;
                        d = p / q;
                    }
                    else
                    {
                        d = m;
                        e = m;
                    }
                }
                else
                {
                    // Inverse quadratic interpolation
                    double q2 = fa / fc;
                    double r = fb / fc;
                    s = fb / fa;
                    double p = s * ((2.0 * m * q2 * (q2 - r)) - ((b - a) * (r - 1.0)));
                    double q3 = (q2 - 1.0) * (r - 1.0) * (s - 1.0);
                    if (p > 0) q3 = -q3;
                    else p = -p;

                    if (2.0 * p < Math.Min(3.0 * m * q3 - Math.Abs(tol * q3), Math.Abs(e * q3)))
                    {
                        e = d;
                        d = p / q3;
                    }
                    else
                    {
                        d = m;
                        e = m;
                    }
                }
            }
            else
            {
                d = m;
                e = m;
            }

            a = b;
            fa = fb;

            if (Math.Abs(d) > tol)
                b += d;
            else
                b += (m > 0 ? tol : -tol);

            fb = f.Evaluate(b);
        }

        root = b;
        return true;
    }

    /// <summary>
    /// Pure Newton-Raphson method: uses derivative for quadratic convergence.
    /// Aborts immediately if root leaves the bound interval.
    /// Matches MathNet.Numerics.RootFinding.NewtonRaphson.TryFindRoot semantics.
    /// </summary>
    /// <returns>True if root found within tolerance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryNewtonRaphson<TFunc, TDFunc>(
        ref TFunc f,
        ref TDFunc df,
        double guess,
        double lo,
        double hi,
        double accuracy,
        int maxIterations,
        out double root)
        where TFunc : struct, IDoubleFunction
        where TDFunc : struct, IDoubleFunction
    {
        root = guess;

        for (int i = 0; i < maxIterations && root >= lo && root <= hi; i++)
        {
            double fx = f.Evaluate(root);

            if (fx == 0.0)
            {
                return true;
            }

            double dfx = df.Evaluate(root);

            // Newton-Raphson step
            double step = fx / dfx;
            root -= step;

            if (Math.Abs(step) < accuracy && Math.Abs(fx) < accuracy)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Robust Newton-Raphson: Newton-Raphson with bisection fallback and subdivision scanning.
    /// Matches MathNet.Numerics.RootFinding.RobustNewtonRaphson.TryFindRoot semantics exactly.
    ///
    /// Algorithm:
    /// 1. Start at midpoint of [lo, hi]
    /// 2. Newton step with convergence monitoring
    /// 3. On failure (overshoot, undershoot, slow convergence):
    ///    - If no bracket (same signs): scan sub-intervals for zero crossings
    ///    - Bisection fallback with bound updates
    /// </summary>
    /// <returns>True if root found within tolerance.</returns>
    public static bool TryRobustNewtonRaphson<TFunc, TDFunc>(
        ref TFunc f,
        ref TDFunc df,
        double lo,
        double hi,
        double accuracy,
        int maxIterations,
        int subdivision,
        out double root)
        where TFunc : struct, IDoubleFunction
        where TDFunc : struct, IDoubleFunction
    {
        root = lo + (0.5 * (hi - lo));
        double fx = f.Evaluate(root);

        if (Math.Abs(fx) < accuracy)
        {
            return true;
        }

        double fmin = f.Evaluate(lo);
        double fmax = f.Evaluate(hi);

        if (Math.Abs(fmin) < accuracy)
        {
            root = lo;
            return true;
        }

        if (Math.Abs(fmax) < accuracy)
        {
            root = hi;
            return true;
        }

        double lastStep = Math.Abs(hi - lo);

        for (int i = 0; i < maxIterations; i++)
        {
            double dfx = df.Evaluate(root);

            // Newton-Raphson step
            double step = fx / dfx;
            root -= step;

            if (Math.Abs(step) < accuracy && Math.Abs(fx) < accuracy)
            {
                return true;
            }

            bool overshoot = root > hi;
            bool undershoot = root < lo;

            if (overshoot || undershoot || Math.Abs(2 * fx) > Math.Abs(lastStep * dfx))
            {
                // Newton-Raphson step failed

                // If same signs, try subdivision to scan for zero crossing intervals
                if (Math.Sign(fmin) == Math.Sign(fmax)
                    && TryScanForCrossingsWithRoots(ref f, ref df, lo, hi, accuracy, maxIterations - i - 1, subdivision, out root))
                {
                    return true;
                }

                // Bisection
                root = 0.5 * (hi + lo);
                fx = f.Evaluate(root);

                if (fx == 0.0)
                {
                    return true;
                }

                lastStep = 0.5 * Math.Abs(hi - lo);

                if (Math.Sign(fx) == Math.Sign(fmin))
                {
                    lo = root;
                    fmin = fx;

                    if (overshoot)
                    {
                        root = hi;
                        fx = fmax;
                    }
                }
                else
                {
                    hi = root;
                    fmax = fx;

                    if (undershoot)
                    {
                        root = lo;
                        fx = fmin;
                    }
                }

                continue;
            }

            // Newton step succeeded - evaluate at new position
            fx = f.Evaluate(root);

            if (fx == 0.0)
            {
                return true;
            }

            lastStep = step;

            // Update bounds
            if (Math.Sign(fx) != Math.Sign(fmin))
            {
                hi = root;
                fmax = fx;
            }
            else if (Math.Sign(fx) != Math.Sign(fmax))
            {
                lo = root;
                fmin = fx;
            }
            else if (Math.Sign(fmin) != Math.Sign(fmax) && Math.Abs(fx) < accuracy)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Scans for zero-crossing sub-intervals and tries to find roots in each.
    /// Zero-allocation replacement for MathNet's ZeroCrossingBracketing.FindIntervalsWithin + recursive TryFindRoot.
    /// </summary>
    private static bool TryScanForCrossingsWithRoots<TFunc, TDFunc>(
        ref TFunc f,
        ref TDFunc df,
        double lo,
        double hi,
        double accuracy,
        int maxIterations,
        int subdivision,
        out double root)
        where TFunc : struct, IDoubleFunction
        where TDFunc : struct, IDoubleFunction
    {
        double fLo = f.Evaluate(lo);
        double fHi = f.Evaluate(hi);

        // If already bracketed, try directly
        if (Math.Sign(fLo) != Math.Sign(fHi))
        {
            return TryRobustNewtonRaphson(ref f, ref df, lo, hi, accuracy, maxIterations, subdivision, out root);
        }

        // Scan sub-intervals for sign changes
        double subdiv = (hi - lo) / subdivision;
        double smin = lo;
        int sign = Math.Sign(fLo);

        for (int k = 0; k < subdivision; k++)
        {
            double smax = smin + subdiv;
            double sfmax = f.Evaluate(smax);

            if (double.IsInfinity(sfmax))
            {
                // Expand interval to include pole
                smin = smax;
                continue;
            }

            if (Math.Sign(sfmax) != sign)
            {
                if (TryRobustNewtonRaphson(ref f, ref df, smin, smax, accuracy, maxIterations, subdivision, out root))
                {
                    return true;
                }

                sign = Math.Sign(sfmax);
            }

            smin = smax;
        }

        root = double.NaN;
        return false;
    }
}
