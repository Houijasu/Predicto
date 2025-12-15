namespace Predicto.Models;

/// <summary>
/// Defines the dodge behavior parameters for a target.
/// Used by the physics-based prediction system to simulate how a target
/// will react to incoming skillshots.
/// </summary>
public readonly record struct DodgeProfile
{
    /// <summary>
    /// Reaction time in seconds before the target starts dodging.
    /// Human reaction times typically range from 0.15s (elite) to 0.5s (casual).
    /// Default: 0.25s (average human visual reaction time).
    /// </summary>
    public double ReactionTime { get; init; }

    /// <summary>
    /// Maximum acceleration the target can apply for dodging (units/second²).
    /// This represents how quickly the target can change direction.
    /// Typical values: 2000-4000 units/s² for champion-like movement.
    /// </summary>
    public double MaxAcceleration { get; init; }

    /// <summary>
    /// Maximum speed the target can reach (units/second).
    /// Used to clamp velocity during simulation.
    /// Typical champion speeds: 325-450 base, up to 600+ with buffs.
    /// </summary>
    public double MaxSpeed { get; init; }

    /// <summary>
    /// Skill level from 0.0 (doesn't dodge) to 1.0 (perfect dodge).
    /// Affects the responsiveness to danger and efficiency of dodge direction choice.
    /// - 0.0: No dodge attempt (bot-like or AFK)
    /// - 0.5: Average player (sometimes dodges, suboptimal direction)
    /// - 1.0: Professional (always dodges optimally)
    /// </summary>
    public double SkillLevel { get; init; }

    /// <summary>
    /// Friction coefficient that slows the target.
    /// Higher values = faster deceleration when not actively moving.
    /// Prevents unrealistic perpetual motion. Default: 5.0
    /// </summary>
    public double Friction { get; init; }

    /// <summary>
    /// How strongly the target tries to return to their original path/destination.
    /// Higher values = target prioritizes path over pure evasion.
    /// Default: 0.3 (moderate path adherence).
    /// </summary>
    public double PathAdherence { get; init; }

    /// <summary>
    /// Creates a dodge profile with specified parameters.
    /// </summary>
    public DodgeProfile(
        double reactionTime = 0.25,
        double maxAcceleration = 3000.0,
        double maxSpeed = 400.0,
        double skillLevel = 0.5,
        double friction = 5.0,
        double pathAdherence = 0.3)
    {
        ReactionTime = reactionTime;
        MaxAcceleration = maxAcceleration;
        MaxSpeed = maxSpeed;
        SkillLevel = skillLevel;
        Friction = friction;
        PathAdherence = pathAdherence;
    }

    #region Preset Profiles

    /// <summary>
    /// Profile for a target that doesn't dodge at all.
    /// Use for minions, stationary targets, or testing baseline predictions.
    /// </summary>
    public static DodgeProfile None => new(
        reactionTime: double.MaxValue,
        maxAcceleration: 0,
        maxSpeed: 400,
        skillLevel: 0,
        friction: 5.0,
        pathAdherence: 1.0);

    /// <summary>
    /// Profile for a beginner player with slow reactions and poor dodge execution.
    /// Reaction: 400ms, Skill: 30%
    /// </summary>
    public static DodgeProfile Beginner => new(
        reactionTime: 0.4,
        maxAcceleration: 2000,
        maxSpeed: 380,
        skillLevel: 0.3,
        friction: 6.0,
        pathAdherence: 0.5);

    /// <summary>
    /// Profile for an average player with typical reactions.
    /// Reaction: 250ms, Skill: 50%
    /// </summary>
    public static DodgeProfile Average => new(
        reactionTime: 0.25,
        maxAcceleration: 3000,
        maxSpeed: 400,
        skillLevel: 0.5,
        friction: 5.0,
        pathAdherence: 0.3);

    /// <summary>
    /// Profile for a skilled player with good reactions and dodge execution.
    /// Reaction: 200ms, Skill: 75%
    /// </summary>
    public static DodgeProfile Skilled => new(
        reactionTime: 0.2,
        maxAcceleration: 3500,
        maxSpeed: 420,
        skillLevel: 0.75,
        friction: 4.0,
        pathAdherence: 0.2);

    /// <summary>
    /// Profile for an expert/professional player with fast reactions.
    /// Reaction: 150ms, Skill: 95%
    /// </summary>
    public static DodgeProfile Expert => new(
        reactionTime: 0.15,
        maxAcceleration: 4000,
        maxSpeed: 450,
        skillLevel: 0.95,
        friction: 3.0,
        pathAdherence: 0.1);

    /// <summary>
    /// Creates a profile based on a target's movement speed.
    /// Useful when you know the champion's base stats.
    /// </summary>
    /// <param name="movementSpeed">Champion's movement speed</param>
    /// <param name="skillLevel">Skill level 0-1</param>
    public static DodgeProfile FromSpeed(double movementSpeed, double skillLevel = 0.5)
    {
        // Interpolate between Beginner and Expert based on skill level
        double reactionTime = 0.4 - skillLevel * 0.25; // 0.4 -> 0.15
        double friction = 6.0 - skillLevel * 3.0; // 6.0 -> 3.0
        double pathAdherence = 0.5 - skillLevel * 0.4; // 0.5 -> 0.1

        return new DodgeProfile(
            reactionTime: reactionTime,
            maxAcceleration: 2000 + skillLevel * 2000, // 2000 -> 4000
            maxSpeed: movementSpeed,
            skillLevel: skillLevel,
            friction: friction,
            pathAdherence: pathAdherence);
    }

    #endregion
}
