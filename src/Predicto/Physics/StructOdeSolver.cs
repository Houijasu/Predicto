using System.Runtime.CompilerServices;
using MathNet.Spatial.Euclidean;

namespace Predicto.Physics;

/// <summary>
/// Represents the state of a 2D particle for physics simulation.
/// Stack-allocated struct to avoid GC pressure.
/// </summary>
public struct ParticleState
{
    /// <summary>X position</summary>
    public double X;

    /// <summary>Y position</summary>
    public double Y;

    /// <summary>X velocity</summary>
    public double Vx;

    /// <summary>Y velocity</summary>
    public double Vy;

    /// <summary>
    /// Creates a new particle state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ParticleState(double x, double y, double vx, double vy)
    {
        X = x;
        Y = y;
        Vx = vx;
        Vy = vy;
    }

    /// <summary>
    /// Creates a particle state from MathNet types.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ParticleState(Point2D position, Vector2D velocity)
    {
        X = position.X;
        Y = position.Y;
        Vx = velocity.X;
        Vy = velocity.Y;
    }

    /// <summary>
    /// Gets the position as a Point2D.
    /// </summary>
    public readonly Point2D Position
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(X, Y);
    }

    /// <summary>
    /// Gets the velocity as a Vector2D.
    /// </summary>
    public readonly Vector2D Velocity
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(Vx, Vy);
    }

    /// <summary>
    /// Gets the speed (velocity magnitude).
    /// </summary>
    public readonly double Speed
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Math.Sqrt(Vx * Vx + Vy * Vy);
    }
}

/// <summary>
/// Represents the derivative of a particle state (acceleration).
/// </summary>
public struct StateDerivative
{
    /// <summary>X velocity (dx/dt)</summary>
    public double Dx;

    /// <summary>Y velocity (dy/dt)</summary>
    public double Dy;

    /// <summary>X acceleration (dvx/dt)</summary>
    public double Dvx;

    /// <summary>Y acceleration (dvy/dt)</summary>
    public double Dvy;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public StateDerivative(double dx, double dy, double dvx, double dvy)
    {
        Dx = dx;
        Dy = dy;
        Dvx = dvx;
        Dvy = dvy;
    }
}

/// <summary>
/// Delegate for computing acceleration given current state and time.
/// </summary>
/// <param name="state">Current particle state</param>
/// <param name="time">Current simulation time</param>
/// <param name="ax">Output X acceleration</param>
/// <param name="ay">Output Y acceleration</param>
public delegate void AccelerationFunction(
    in ParticleState state,
    double time,
    out double ax,
    out double ay);

/// <summary>
/// Zero-allocation 4th-order Runge-Kutta ODE solver for 2D particle dynamics.
/// Solves the system: dx/dt = v, dv/dt = a(x, v, t)
/// </summary>
public static class StructOdeSolver
{
    /// <summary>
    /// Performs a single RK4 integration step.
    /// </summary>
    /// <param name="state">Current state (modified in place)</param>
    /// <param name="time">Current time</param>
    /// <param name="dt">Time step</param>
    /// <param name="acceleration">Function to compute acceleration</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Step(
        ref ParticleState state,
        double time,
        double dt,
        AccelerationFunction acceleration)
    {
        // k1 = f(t, y)
        acceleration(in state, time, out double ax1, out double ay1);
        double k1x = state.Vx;
        double k1y = state.Vy;
        double k1vx = ax1;
        double k1vy = ay1;

        // k2 = f(t + dt/2, y + k1*dt/2)
        var state2 = new ParticleState(
            state.X + k1x * dt * 0.5,
            state.Y + k1y * dt * 0.5,
            state.Vx + k1vx * dt * 0.5,
            state.Vy + k1vy * dt * 0.5);
        acceleration(in state2, time + dt * 0.5, out double ax2, out double ay2);
        double k2x = state2.Vx;
        double k2y = state2.Vy;
        double k2vx = ax2;
        double k2vy = ay2;

        // k3 = f(t + dt/2, y + k2*dt/2)
        var state3 = new ParticleState(
            state.X + k2x * dt * 0.5,
            state.Y + k2y * dt * 0.5,
            state.Vx + k2vx * dt * 0.5,
            state.Vy + k2vy * dt * 0.5);
        acceleration(in state3, time + dt * 0.5, out double ax3, out double ay3);
        double k3x = state3.Vx;
        double k3y = state3.Vy;
        double k3vx = ax3;
        double k3vy = ay3;

        // k4 = f(t + dt, y + k3*dt)
        var state4 = new ParticleState(
            state.X + k3x * dt,
            state.Y + k3y * dt,
            state.Vx + k3vx * dt,
            state.Vy + k3vy * dt);
        acceleration(in state4, time + dt, out double ax4, out double ay4);
        double k4x = state4.Vx;
        double k4y = state4.Vy;
        double k4vx = ax4;
        double k4vy = ay4;

        // y(t+dt) = y(t) + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
        double sixth = 1.0 / 6.0;
        state.X += (k1x + 2.0 * k2x + 2.0 * k3x + k4x) * dt * sixth;
        state.Y += (k1y + 2.0 * k2y + 2.0 * k3y + k4y) * dt * sixth;
        state.Vx += (k1vx + 2.0 * k2vx + 2.0 * k3vx + k4vx) * dt * sixth;
        state.Vy += (k1vy + 2.0 * k2vy + 2.0 * k3vy + k4vy) * dt * sixth;
    }

    /// <summary>
    /// Performs a single Euler integration step (faster but less accurate).
    /// Use for quick approximations or when dt is very small.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void StepEuler(
        ref ParticleState state,
        double time,
        double dt,
        AccelerationFunction acceleration)
    {
        acceleration(in state, time, out double ax, out double ay);

        state.X += state.Vx * dt;
        state.Y += state.Vy * dt;
        state.Vx += ax * dt;
        state.Vy += ay * dt;
    }

    /// <summary>
    /// Performs a Velocity Verlet integration step.
    /// Second-order accurate and energy-conserving for conservative forces.
    /// Requires acceleration to be position-dependent only (not velocity-dependent).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void StepVerlet(
        ref ParticleState state,
        double time,
        double dt,
        AccelerationFunction acceleration)
    {
        // Get current acceleration
        acceleration(in state, time, out double ax1, out double ay1);

        // Update position: x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
        state.X += state.Vx * dt + 0.5 * ax1 * dt * dt;
        state.Y += state.Vy * dt + 0.5 * ay1 * dt * dt;

        // Get new acceleration at new position
        acceleration(in state, time + dt, out double ax2, out double ay2);

        // Update velocity: v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
        state.Vx += 0.5 * (ax1 + ax2) * dt;
        state.Vy += 0.5 * (ay1 + ay2) * dt;
    }

    /// <summary>
    /// Integrates the particle state over a time interval using fixed time steps.
    /// </summary>
    /// <param name="state">Initial state (modified to final state)</param>
    /// <param name="startTime">Start time of integration</param>
    /// <param name="endTime">End time of integration</param>
    /// <param name="dt">Time step size</param>
    /// <param name="acceleration">Acceleration function</param>
    /// <returns>Number of steps taken</returns>
    public static int Integrate(
        ref ParticleState state,
        double startTime,
        double endTime,
        double dt,
        AccelerationFunction acceleration)
    {
        double time = startTime;
        int steps = 0;

        while (time < endTime)
        {
            double stepDt = Math.Min(dt, endTime - time);
            Step(ref state, time, stepDt, acceleration);
            time += stepDt;
            steps++;
        }

        return steps;
    }

    /// <summary>
    /// Integrates with early termination when a condition is met.
    /// </summary>
    /// <param name="state">Initial state (modified to final state)</param>
    /// <param name="startTime">Start time</param>
    /// <param name="maxTime">Maximum time to simulate</param>
    /// <param name="dt">Time step</param>
    /// <param name="acceleration">Acceleration function</param>
    /// <param name="terminationCondition">Returns true to stop simulation</param>
    /// <param name="finalTime">Actual end time when simulation stopped</param>
    /// <returns>True if terminated early by condition, false if reached maxTime</returns>
    public static bool IntegrateUntil(
        ref ParticleState state,
        double startTime,
        double maxTime,
        double dt,
        AccelerationFunction acceleration,
        Func<ParticleState, double, bool> terminationCondition,
        out double finalTime)
    {
        double time = startTime;

        while (time < maxTime)
        {
            if (terminationCondition(state, time))
            {
                finalTime = time;
                return true;
            }

            double stepDt = Math.Min(dt, maxTime - time);
            Step(ref state, time, stepDt, acceleration);
            time += stepDt;
        }

        finalTime = time;
        return false;
    }
}
