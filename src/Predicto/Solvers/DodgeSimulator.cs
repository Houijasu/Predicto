using System.Runtime.CompilerServices;
using MathNet.Spatial.Euclidean;
using Predicto.Models;
using Predicto.Physics;

namespace Predicto.Solvers;

/// <summary>
/// Simulates target dodge behavior using Newtonian physics.
/// The target is modeled as a particle subject to:
/// - Danger repulsion force (from skillshot danger field)
/// - Goal attraction force (toward original destination/path)
/// - Friction (energy dissipation)
/// 
/// This implements the "Least Action" principle through local force equilibrium:
/// the target naturally finds the path that minimizes "effort" (action integral).
/// </summary>
public class DodgeSimulator
{
    /// <summary>
    /// Default time step for simulation (in seconds).
    /// Smaller = more accurate but slower. 1/60 is typically sufficient.
    /// </summary>
    public const double DefaultTimeStep = 1.0 / 60.0;

    /// <summary>
    /// Maximum simulation time (seconds) to prevent infinite loops.
    /// </summary>
    public const double MaxSimulationTime = 2.0;

    /// <summary>
    /// Danger potential threshold below which target is considered "safe".
    /// </summary>
    public const double SafetyThreshold = 10.0;

    /// <summary>
    /// Simulates a target's dodge trajectory against a linear skillshot.
    /// </summary>
    /// <param name="targetPosition">Initial target position</param>
    /// <param name="targetVelocity">Initial target velocity</param>
    /// <param name="dangerField">The danger field from the skillshot</param>
    /// <param name="profile">Target's dodge behavior profile</param>
    /// <param name="goalPosition">Optional goal position the target wants to reach</param>
    /// <param name="maxTime">Maximum simulation time</param>
    /// <param name="dt">Time step</param>
    /// <returns>Simulation result with final state and collision info</returns>
    public DodgeSimulationResult Simulate(
        Point2D targetPosition,
        Vector2D targetVelocity,
        LinearDangerField dangerField,
        DodgeProfile profile,
        Point2D? goalPosition = null,
        double maxTime = MaxSimulationTime,
        double dt = DefaultTimeStep)
    {
        var state = new ParticleState(targetPosition, targetVelocity);
        Point2D startPos = targetPosition;
        Point2D goal = goalPosition ?? targetPosition + targetVelocity.Normalize() * 500;

        double time = 0;
        double distanceTraveled = 0;
        double minDangerDistance = double.MaxValue;
        Point2D prevPos = targetPosition;

        // Reaction phase: target moves at constant velocity
        double reactionEndTime = Math.Min(profile.ReactionTime, maxTime);
        while (time < reactionEndTime)
        {
            double stepDt = Math.Min(dt, reactionEndTime - time);

            // Check collision during reaction phase
            double collisionDist = GetCollisionDistance(state.Position, dangerField, time);
            minDangerDistance = Math.Min(minDangerDistance, collisionDist);

            if (collisionDist < Constants.DefaultHitboxRadius)
            {
                return DodgeSimulationResult.Collided(
                    state.Position,
                    state.Velocity,
                    time,
                    distanceTraveled,
                    minDangerDistance);
            }

            // Coast at constant velocity
            state.X += state.Vx * stepDt;
            state.Y += state.Vy * stepDt;

            distanceTraveled += Math.Sqrt(
                (state.X - prevPos.X) * (state.X - prevPos.X) +
                (state.Y - prevPos.Y) * (state.Y - prevPos.Y));
            prevPos = state.Position;

            time += stepDt;
        }

        // Dodge phase: target reacts to danger
        // Create acceleration function that captures all forces
        AccelerationFunction accel = (in ParticleState s, double t, out double ax, out double ay) =>
        {
            ComputeAcceleration(in s, t, dangerField, profile, goal, out ax, out ay);
        };

        while (time < maxTime)
        {
            // Check collision
            double collisionDist = GetCollisionDistance(state.Position, dangerField, time);
            minDangerDistance = Math.Min(minDangerDistance, collisionDist);

            if (collisionDist < Constants.DefaultHitboxRadius)
            {
                return DodgeSimulationResult.Collided(
                    state.Position,
                    state.Velocity,
                    time,
                    distanceTraveled,
                    minDangerDistance);
            }

            // Check if escaped danger (potential below threshold)
            double potential = dangerField.Potential(state.Position, time);
            if (potential < SafetyThreshold && time > profile.ReactionTime + 0.1)
            {
                return DodgeSimulationResult.Escaped(
                    state.Position,
                    state.Velocity,
                    time,
                    distanceTraveled,
                    minDangerDistance);
            }

            // Integrate one step
            double stepDt = Math.Min(dt, maxTime - time);
            StructOdeSolver.StepVerlet(ref state, time, stepDt, accel);

            // Clamp speed
            ClampSpeed(ref state, profile.MaxSpeed);

            distanceTraveled += Math.Sqrt(
                (state.X - prevPos.X) * (state.X - prevPos.X) +
                (state.Y - prevPos.Y) * (state.Y - prevPos.Y));
            prevPos = state.Position;

            time += stepDt;
        }

        // Reached max time without clear resolution
        return DodgeSimulationResult.Escaped(
            state.Position,
            state.Velocity,
            time,
            distanceTraveled,
            minDangerDistance);
    }

    /// <summary>
    /// Simulates dodge against a circular skillshot.
    /// </summary>
    public DodgeSimulationResult Simulate(
        Point2D targetPosition,
        Vector2D targetVelocity,
        CircularDangerField dangerField,
        DodgeProfile profile,
        Point2D? goalPosition = null,
        double maxTime = MaxSimulationTime,
        double dt = DefaultTimeStep)
    {
        var state = new ParticleState(targetPosition, targetVelocity);
        Point2D goal = goalPosition ?? targetPosition + targetVelocity.Normalize() * 500;

        double time = 0;
        double distanceTraveled = 0;
        double minDangerDistance = double.MaxValue;
        Point2D prevPos = targetPosition;

        // Reaction phase
        double reactionEndTime = Math.Min(profile.ReactionTime, maxTime);
        while (time < reactionEndTime)
        {
            double stepDt = Math.Min(dt, reactionEndTime - time);

            double dx = state.X - dangerField.Center.X;
            double dy = state.Y - dangerField.Center.Y;
            double dist = Math.Sqrt(dx * dx + dy * dy);
            minDangerDistance = Math.Min(minDangerDistance, dist);

            if (dist < dangerField.Radius && time >= dangerField.DetonationTime - 0.05)
            {
                return DodgeSimulationResult.Collided(
                    state.Position, state.Velocity, time, distanceTraveled, minDangerDistance);
            }

            state.X += state.Vx * stepDt;
            state.Y += state.Vy * stepDt;

            distanceTraveled += Math.Sqrt(
                (state.X - prevPos.X) * (state.X - prevPos.X) +
                (state.Y - prevPos.Y) * (state.Y - prevPos.Y));
            prevPos = state.Position;
            time += stepDt;
        }

        // Dodge phase
        AccelerationFunction accel = (in ParticleState s, double t, out double ax, out double ay) =>
        {
            ComputeAccelerationCircular(in s, t, dangerField, profile, goal, out ax, out ay);
        };

        while (time < maxTime)
        {
            double dx = state.X - dangerField.Center.X;
            double dy = state.Y - dangerField.Center.Y;
            double dist = Math.Sqrt(dx * dx + dy * dy);
            minDangerDistance = Math.Min(minDangerDistance, dist);

            // Check collision at detonation
            if (dist < dangerField.Radius && time >= dangerField.DetonationTime - 0.05)
            {
                return DodgeSimulationResult.Collided(
                    state.Position, state.Velocity, time, distanceTraveled, minDangerDistance);
            }

            // Check if past detonation and outside radius
            if (time > dangerField.DetonationTime + 0.1 && dist > dangerField.Radius)
            {
                return DodgeSimulationResult.Escaped(
                    state.Position, state.Velocity, time, distanceTraveled, minDangerDistance);
            }

            double stepDt = Math.Min(dt, maxTime - time);
            StructOdeSolver.StepVerlet(ref state, time, stepDt, accel);
            ClampSpeed(ref state, profile.MaxSpeed);

            distanceTraveled += Math.Sqrt(
                (state.X - prevPos.X) * (state.X - prevPos.X) +
                (state.Y - prevPos.Y) * (state.Y - prevPos.Y));
            prevPos = state.Position;
            time += stepDt;
        }

        return DodgeSimulationResult.Escaped(
            state.Position, state.Velocity, time, distanceTraveled, minDangerDistance);
    }

    /// <summary>
    /// Simulates dodge and returns the full trajectory for LAP-based interception.
    /// This method records positions at each time step for later intercept calculation.
    /// </summary>
    /// <param name="targetPosition">Initial target position</param>
    /// <param name="targetVelocity">Initial target velocity</param>
    /// <param name="dangerField">The danger field from the skillshot</param>
    /// <param name="profile">Target's dodge behavior profile</param>
    /// <param name="goalPosition">Optional goal position the target wants to reach</param>
    /// <param name="maxTime">Maximum simulation time</param>
    /// <param name="dt">Time step</param>
    /// <returns>Tuple of simulation result and sampled trajectory</returns>
    public (DodgeSimulationResult Result, DodgeTrajectory Trajectory) SimulateWithTrajectory(
        Point2D targetPosition,
        Vector2D targetVelocity,
        LinearDangerField dangerField,
        DodgeProfile profile,
        Point2D? goalPosition = null,
        double maxTime = MaxSimulationTime,
        double dt = DefaultTimeStep)
    {
        // Pre-allocate trajectory arrays (estimate max samples)
        int maxSamples = (int)(maxTime / dt) + 2;
        var positions = new List<Point2D>(maxSamples);
        var velocities = new List<Vector2D>(maxSamples);

        var state = new ParticleState(targetPosition, targetVelocity);
        Point2D startPos = targetPosition;
        Point2D goal = goalPosition ?? targetPosition + targetVelocity.Normalize() * 500;

        double time = 0;
        double distanceTraveled = 0;
        double minDangerDistance = double.MaxValue;
        Point2D prevPos = targetPosition;

        // Record initial state
        positions.Add(state.Position);
        velocities.Add(state.Velocity);

        // Reaction phase: target moves at constant velocity
        double reactionEndTime = Math.Min(profile.ReactionTime, maxTime);
        while (time < reactionEndTime)
        {
            double stepDt = Math.Min(dt, reactionEndTime - time);

            double collisionDist = GetCollisionDistance(state.Position, dangerField, time);
            minDangerDistance = Math.Min(minDangerDistance, collisionDist);

            if (collisionDist < Constants.DefaultHitboxRadius)
            {
                var trajectory = new DodgeTrajectory(positions.ToArray(), velocities.ToArray(), dt);
                return (DodgeSimulationResult.Collided(
                    state.Position, state.Velocity, time, distanceTraveled, minDangerDistance), trajectory);
            }

            state.X += state.Vx * stepDt;
            state.Y += state.Vy * stepDt;

            distanceTraveled += Math.Sqrt(
                (state.X - prevPos.X) * (state.X - prevPos.X) +
                (state.Y - prevPos.Y) * (state.Y - prevPos.Y));
            prevPos = state.Position;

            time += stepDt;

            // Record state at each step
            positions.Add(state.Position);
            velocities.Add(state.Velocity);
        }

        // Dodge phase: target reacts to danger
        AccelerationFunction accel = (in ParticleState s, double t, out double ax, out double ay) =>
        {
            ComputeAcceleration(in s, t, dangerField, profile, goal, out ax, out ay);
        };

        while (time < maxTime)
        {
            double collisionDist = GetCollisionDistance(state.Position, dangerField, time);
            minDangerDistance = Math.Min(minDangerDistance, collisionDist);

            if (collisionDist < Constants.DefaultHitboxRadius)
            {
                var trajectory = new DodgeTrajectory(positions.ToArray(), velocities.ToArray(), dt);
                return (DodgeSimulationResult.Collided(
                    state.Position, state.Velocity, time, distanceTraveled, minDangerDistance), trajectory);
            }

            double potential = dangerField.Potential(state.Position, time);
            if (potential < SafetyThreshold && time > profile.ReactionTime + 0.1)
            {
                var trajectory = new DodgeTrajectory(positions.ToArray(), velocities.ToArray(), dt);
                return (DodgeSimulationResult.Escaped(
                    state.Position, state.Velocity, time, distanceTraveled, minDangerDistance), trajectory);
            }

            double stepDt = Math.Min(dt, maxTime - time);
            StructOdeSolver.StepVerlet(ref state, time, stepDt, accel);
            ClampSpeed(ref state, profile.MaxSpeed);

            distanceTraveled += Math.Sqrt(
                (state.X - prevPos.X) * (state.X - prevPos.X) +
                (state.Y - prevPos.Y) * (state.Y - prevPos.Y));
            prevPos = state.Position;

            time += stepDt;

            // Record state at each step
            positions.Add(state.Position);
            velocities.Add(state.Velocity);
        }

        var finalTrajectory = new DodgeTrajectory(positions.ToArray(), velocities.ToArray(), dt);
        return (DodgeSimulationResult.Escaped(
            state.Position, state.Velocity, time, distanceTraveled, minDangerDistance), finalTrajectory);
    }

    /// <summary>
    /// Estimates the probability that a target will successfully dodge a linear skillshot.
    /// </summary>
    /// <param name="targetPosition">Current target position</param>
    /// <param name="targetVelocity">Current target velocity</param>
    /// <param name="casterPosition">Skillshot origin</param>
    /// <param name="skillshot">Skillshot parameters</param>
    /// <param name="profile">Target's dodge profile</param>
    /// <returns>Probability 0-1 that the target will dodge successfully</returns>
    public double EstimateDodgeProbability(
        Point2D targetPosition,
        Vector2D targetVelocity,
        Point2D casterPosition,
        LinearSkillshot skillshot,
        DodgeProfile profile)
    {
        // Quick reject: if skill level is 0, never dodges
        if (profile.SkillLevel < Constants.Epsilon)
            return 0;

        // Direction from caster to target
        var direction = (targetPosition - casterPosition).Normalize();

        // Create danger field
        var dangerField = new LinearDangerField(casterPosition, direction, skillshot);

        // Run simulation
        var result = Simulate(targetPosition, targetVelocity, dangerField, profile);

        // Calculate dodge probability based on simulation outcome
        if (result.WillCollide)
        {
            // Collision predicted - but skill level affects actual dodge rate
            // Even predicted collisions might be dodged by skilled players
            // who can make micro-adjustments we don't simulate
            return profile.SkillLevel * 0.2; // Max 20% dodge rate even with collision predicted
        }
        else
        {
            // Escape predicted - skill level affects execution
            // Higher skill = more likely to execute the predicted dodge
            double baseProb = 0.5 + profile.SkillLevel * 0.5; // 50% to 100%

            // Adjust based on how close the call was
            double safetyMargin = result.MinDangerDistance - Constants.DefaultHitboxRadius;
            if (safetyMargin < 50)
            {
                // Very close call - reduce probability
                baseProb *= 0.5 + safetyMargin / 100.0;
            }

            return Math.Clamp(baseProb, 0, 1);
        }
    }

    /// <summary>
    /// Gets the distance from target position to the skillshot's collision zone.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double GetCollisionDistance(Point2D position, LinearDangerField field, double time)
    {
        Point2D projectilePos = field.GetProjectilePosition(time);

        // Vector from projectile to target
        double dx = position.X - projectilePos.X;
        double dy = position.Y - projectilePos.Y;

        // Project onto skillshot direction
        double alongPath = dx * field.Direction.X + dy * field.Direction.Y;

        // Only collide with front of projectile
        if (alongPath < -50 || alongPath > 100)
            return double.MaxValue;

        // Perpendicular distance
        return Math.Abs(dx * field.Direction.Y - dy * field.Direction.X);
    }

    /// <summary>
    /// Computes the total acceleration on the target from all forces.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ComputeAcceleration(
        in ParticleState state,
        double time,
        LinearDangerField dangerField,
        DodgeProfile profile,
        Point2D goal,
        out double ax,
        out double ay)
    {
        // 1. Danger repulsion force (F = -∇U)
        dangerField.Force(state.Position, time, out double dangerFx, out double dangerFy);

        // Scale by skill level (better players react more strongly)
        dangerFx *= profile.SkillLevel;
        dangerFy *= profile.SkillLevel;

        // 2. Goal attraction force (toward destination)
        double goalDx = goal.X - state.X;
        double goalDy = goal.Y - state.Y;
        double goalDist = Math.Sqrt(goalDx * goalDx + goalDy * goalDy);

        double goalFx = 0, goalFy = 0;
        if (goalDist > Constants.Epsilon)
        {
            double goalStrength = profile.PathAdherence * profile.MaxAcceleration * 0.3;
            goalFx = goalDx / goalDist * goalStrength;
            goalFy = goalDy / goalDist * goalStrength;
        }

        // 3. Friction force (opposes velocity)
        double frictionFx = -profile.Friction * state.Vx;
        double frictionFy = -profile.Friction * state.Vy;

        // Total force
        double totalFx = dangerFx + goalFx + frictionFx;
        double totalFy = dangerFy + goalFy + frictionFy;

        // Clamp to max acceleration
        FastMath.ClampMagnitude(totalFx, totalFy, profile.MaxAcceleration, out ax, out ay);
    }

    /// <summary>
    /// Computes acceleration for circular danger field.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ComputeAccelerationCircular(
        in ParticleState state,
        double time,
        CircularDangerField dangerField,
        DodgeProfile profile,
        Point2D goal,
        out double ax,
        out double ay)
    {
        // Danger force
        dangerField.Force(state.Position, time, out double dangerFx, out double dangerFy);
        dangerFx *= profile.SkillLevel;
        dangerFy *= profile.SkillLevel;

        // Goal force
        double goalDx = goal.X - state.X;
        double goalDy = goal.Y - state.Y;
        double goalDist = Math.Sqrt(goalDx * goalDx + goalDy * goalDy);

        double goalFx = 0, goalFy = 0;
        if (goalDist > Constants.Epsilon)
        {
            double goalStrength = profile.PathAdherence * profile.MaxAcceleration * 0.3;
            goalFx = goalDx / goalDist * goalStrength;
            goalFy = goalDy / goalDist * goalStrength;
        }

        // Friction
        double frictionFx = -profile.Friction * state.Vx;
        double frictionFy = -profile.Friction * state.Vy;

        // Total
        double totalFx = dangerFx + goalFx + frictionFx;
        double totalFy = dangerFy + goalFy + frictionFy;

        FastMath.ClampMagnitude(totalFx, totalFy, profile.MaxAcceleration, out ax, out ay);
    }

    /// <summary>
    /// Clamps the particle's speed to the maximum allowed.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ClampSpeed(ref ParticleState state, double maxSpeed)
    {
        double speedSq = state.Vx * state.Vx + state.Vy * state.Vy;
        double maxSpeedSq = maxSpeed * maxSpeed;

        if (speedSq > maxSpeedSq)
        {
            double scale = maxSpeed / Math.Sqrt(speedSq);
            state.Vx *= scale;
            state.Vy *= scale;
        }
    }
}
