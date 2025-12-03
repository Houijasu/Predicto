using Predicto.Models;

namespace Predicto;

/// <summary>
/// Interface for movement prediction engines.
/// Implementations compute where a skillshot should be aimed to intercept a moving target.
/// </summary>
public interface IPrediction
{
    /// <summary>
    /// Predicts the optimal interception point for a linear skillshot.
    /// </summary>
    /// <param name="input">Input parameters containing caster position, target state, and skillshot data</param>
    /// <returns>Prediction result indicating success (Hit), out of range, or unreachable</returns>
    PredictionResult Predict(PredictionInput input);
}
