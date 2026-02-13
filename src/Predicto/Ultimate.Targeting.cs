using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using MathNet.Spatial.Euclidean;
using Predicto.Models;
using Predicto.Solvers;

namespace Predicto;

public sealed partial class Ultimate
{
    /// <summary>
    /// Evaluates multiple targets and returns them ranked by hit probability.
    ///
    /// This method is useful for team fights or scenarios with multiple potential targets.
    /// It considers:
    /// - Prediction confidence (hit probability)
    /// - Target priority weight (optional, for focusing high-value targets)
    /// - Range efficiency (closer targets preferred when confidence is similar)
    ///
    /// Example usage:
    /// <code>
    /// var targets = new[] { enemyAdc, enemySupport, enemyMid };
    /// var ranked = ultimate.RankTargets(casterPos, skillshot, targets);
    /// var bestTarget = ranked.FirstOrDefault(t => t.Result is PredictionResult.Hit);
    /// </code>
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets with their movement data</param>
    /// <returns>Targets ranked by priority score (highest first)</returns>
    public IReadOnlyList<RankedTarget> RankTargets(
        Point2D casterPosition,
        LinearSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        if (targets.IsEmpty)
            return Array.Empty<RankedTarget>();

        var results = new RankedTarget[targets.Length];

        // SIMD-vectorized pre-filtering for range check
        Span<bool> reachable = targets.Length <= Constants.StackallocThreshold ? stackalloc bool[targets.Length] : new bool[targets.Length];
        PreFilterTargetsSimd(casterPosition, skillshot.Range, targets, reachable);

        for (int i = 0; i < targets.Length; i++)
        {
            var target = targets[i];
            PredictionResult result;

            if (!reachable[i])
            {
                // Instant out of range
                result = new PredictionResult.OutOfRange((target.Position - casterPosition).Length, skillshot.Range);
            }
            else
            {
                var input = new PredictionInput(
                    casterPosition,
                    target.Position,
                    target.Velocity,
                    skillshot,
                    target.HitboxRadius,
                    target.Path,
                    Strategy: target.Strategy);

                result = Predict(input);
            }

            double priorityScore = CalculatePriorityScore(result, target.PriorityWeight, casterPosition, skillshot.Range);
            results[i] = new RankedTarget(target, result, priorityScore);
        }

        // Sort by priority score descending (highest priority first)
        Array.Sort(results, static (a, b) => b.PriorityScore.CompareTo(a.PriorityScore));

        return results;
    }

    /// <summary>
    /// Evaluates multiple targets for a circular skillshot and returns them ranked by hit probability.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The circular skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets with their movement data</param>
    /// <returns>Targets ranked by priority score (highest first)</returns>
    public IReadOnlyList<RankedTarget> RankTargetsCircular(
        Point2D casterPosition,
        CircularSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        if (targets.IsEmpty)
            return Array.Empty<RankedTarget>();

        var results = new RankedTarget[targets.Length];

        // SIMD-vectorized pre-filtering for range check
        Span<bool> reachable = targets.Length <= Constants.StackallocThreshold ? stackalloc bool[targets.Length] : new bool[targets.Length];
        PreFilterTargetsSimd(casterPosition, skillshot.Range, targets, reachable);

        for (int i = 0; i < targets.Length; i++)
        {
            var target = targets[i];
            PredictionResult result;

            if (!reachable[i])
            {
                result = new PredictionResult.OutOfRange((target.Position - casterPosition).Length, skillshot.Range);
            }
            else
            {
                var input = new CircularPredictionInput(
                    casterPosition,
                    target.Position,
                    target.Velocity,
                    skillshot,
                    target.HitboxRadius,
                    target.Path,
                    Strategy: target.Strategy);

                result = PredictCircular(input);
            }

            double priorityScore = CalculatePriorityScore(result, target.PriorityWeight, casterPosition, skillshot.Range);
            results[i] = new RankedTarget(target, result, priorityScore);
        }

        // Sort by priority score descending (highest priority first)
        Array.Sort(results, static (a, b) => b.PriorityScore.CompareTo(a.PriorityScore));

        return results;
    }

    /// <summary>
    /// Gets the single best target from a collection of candidates.
    /// Returns null if no hittable targets exist.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets</param>
    /// <returns>The best target to aim for, or null if none are hittable</returns>
    public RankedTarget? GetBestTarget(
        Point2D casterPosition,
        LinearSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        var ranked = RankTargets(casterPosition, skillshot, targets);

        // Return first hittable target (already sorted by priority)
        foreach (var target in ranked)
        {
            if (target.Result is PredictionResult.Hit)
                return target;
        }

        return null;
    }

    /// <summary>
    /// Gets the single best target for a circular skillshot.
    /// Returns null if no hittable targets exist.
    /// </summary>
    /// <param name="casterPosition">Position of the caster</param>
    /// <param name="skillshot">The circular skillshot to evaluate</param>
    /// <param name="targets">Array of potential targets</param>
    /// <returns>The best target to aim for, or null if none are hittable</returns>
    public RankedTarget? GetBestTargetCircular(
        Point2D casterPosition,
        CircularSkillshot skillshot,
        ReadOnlySpan<TargetCandidate> targets)
    {
        var ranked = RankTargetsCircular(casterPosition, skillshot, targets);

        // Return first hittable target (already sorted by priority)
        foreach (var target in ranked)
        {
            if (target.Result is PredictionResult.Hit)
                return target;
        }

        return null;
    }

    /// <summary>
    /// Calculates priority score for ranking targets.
    ///
    /// Score components:
    /// - Confidence (0-1): Base hit probability
    /// - Priority weight (0-∞): User-defined target importance (e.g., ADC = 2.0, Tank = 0.5)
    /// - Range efficiency (0-1): Bonus for closer targets (reduces risk of interception)
    ///
    /// Formula: Score = Confidence * PriorityWeight * (0.8 + 0.2 * RangeEfficiency)
    ///
    /// The range efficiency contributes only 20% to allow priority and confidence to dominate,
    /// but still prefers closer targets when other factors are equal.
    /// </summary>
    private static double CalculatePriorityScore(
        PredictionResult result,
        double priorityWeight,
        Point2D casterPosition,
        double skillshotRange)
    {
        if (result is not PredictionResult.Hit hit)
        {
            // Non-hittable targets get score based on how close they are to being hittable
            if (result is PredictionResult.OutOfRange outOfRange)
            {
                // Small negative score based on how far out of range
                double overRange = outOfRange.Distance - outOfRange.MaxRange;
                return -overRange / skillshotRange;
            }

            // Unreachable targets get lowest priority
            return -1000;
        }

        // === Base Score: Confidence ===
        double score = hit.Confidence;

        // === Apply Priority Weight ===
        // Default weight is 1.0, higher values = more important target
        score *= Math.Max(0.1, priorityWeight);

        // === Range Efficiency Bonus ===
        // Closer targets are preferred (less travel time = less chance of missing)
        double distance = (hit.CastPosition - casterPosition).Length;
        double rangeEfficiency = 1.0 - Math.Clamp(distance / skillshotRange, 0, 1);

        // Range efficiency contributes 20% to final score
        score *= 0.8 + (0.2 * rangeEfficiency);

        return score;
    }

    /// <summary>
    /// Performs SIMD-accelerated pre-filtering of targets to improve ranking speed.
    /// Filters out targets that are mathematically impossible to hit based on range.
    ///
    /// Both SIMD and scalar paths use squared-distance comparison:
    ///   distSq ≤ (maxRange + hitboxRadius + PrefilterBuffer)²
    /// The SIMD path batches 4 targets at a time and uses per-lane hitbox radii.
    /// </summary>
    private static void PreFilterTargetsSimd(
        Point2D casterPos,
        double maxRange,
        ReadOnlySpan<TargetCandidate> targets,
        Span<bool> outReachable)
    {
        if (!Avx.IsSupported || targets.Length < 4)
        {
            // Scalar fallback: squared-distance comparison with per-target hitbox
            for (int i = 0; i < targets.Length; i++)
            {
                var disp = targets[i].Position - casterPos;
                double distSq = disp.DotProduct(disp);
                double limit = maxRange + targets[i].HitboxRadius + Constants.PrefilterBuffer;
                outReachable[i] = distSq <= limit * limit;
            }
            return;
        }

        int iSimd = 0;
        Vector256<double> cx = Vector256.Create(casterPos.X);
        Vector256<double> cy = Vector256.Create(casterPos.Y);
        Vector256<double> baseRange = Vector256.Create(maxRange + Constants.PrefilterBuffer);

        for (; iSimd <= targets.Length - 4; iSimd += 4)
        {
            Vector256<double> tx = Vector256.Create(targets[iSimd].Position.X, targets[iSimd + 1].Position.X, targets[iSimd + 2].Position.X, targets[iSimd + 3].Position.X);
            Vector256<double> ty = Vector256.Create(targets[iSimd].Position.Y, targets[iSimd + 1].Position.Y, targets[iSimd + 2].Position.Y, targets[iSimd + 3].Position.Y);

            Vector256<double> dx = Avx.Subtract(tx, cx);
            Vector256<double> dy = Avx.Subtract(ty, cy);
            Vector256<double> distSq = Avx.Add(Avx.Multiply(dx, dx), Avx.Multiply(dy, dy));

            Vector256<double> hitbox = Vector256.Create(
                targets[iSimd].HitboxRadius,
                targets[iSimd + 1].HitboxRadius,
                targets[iSimd + 2].HitboxRadius,
                targets[iSimd + 3].HitboxRadius);

            Vector256<double> rangeLimit = Avx.Add(baseRange, hitbox);
            Vector256<double> limitSq = Avx.Multiply(rangeLimit, rangeLimit);

            Vector256<double> mask = Avx.CompareLessThanOrEqual(distSq, limitSq);

            outReachable[iSimd] = mask.GetElement(0) != 0;
            outReachable[iSimd + 1] = mask.GetElement(1) != 0;
            outReachable[iSimd + 2] = mask.GetElement(2) != 0;
            outReachable[iSimd + 3] = mask.GetElement(3) != 0;
        }

        // Scalar tail: identical formula shape (squared-distance), with per-target hitbox
        for (; iSimd < targets.Length; iSimd++)
        {
            var disp = targets[iSimd].Position - casterPos;
            double distSq = disp.DotProduct(disp);
            double limit = maxRange + targets[iSimd].HitboxRadius + Constants.PrefilterBuffer;
            outReachable[iSimd] = distSq <= limit * limit;
        }
    }
}
