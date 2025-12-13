using MathNet.Spatial.Euclidean;

namespace Predicto.Models;

/// <summary>
/// Represents a multi-waypoint movement path for a target.
/// Targets move along waypoints sequentially at a constant speed.
/// </summary>
public sealed class TargetPath
{
    /// <summary>
    /// The waypoints defining the path. Target moves from Waypoints[CurrentWaypointIndex] toward the end.
    /// Must contain at least 1 point (destination-only paths are allowed).
    /// </summary>
    public IReadOnlyList<Point2D> Waypoints { get; }

    /// <summary>
    /// Current position of the target (may be between waypoints).
    /// </summary>
    public Point2D CurrentPosition { get; }

    /// <summary>
    /// Index of the waypoint the target is currently moving toward.
    /// 0 means moving toward Waypoints[0], 1 means moving toward Waypoints[1], etc.
    /// </summary>
    public int CurrentWaypointIndex { get; }

    /// <summary>
    /// Target's movement speed in units per second.
    /// </summary>
    public double Speed { get; }

    /// <summary>
    /// Creates a new target path.
    /// </summary>
    /// <param name="waypoints">The waypoints defining the path (minimum 1)</param>
    /// <param name="currentPosition">Current position of the target</param>
    /// <param name="currentWaypointIndex">Index of waypoint target is moving toward</param>
    /// <param name="speed">Movement speed in units per second</param>
    /// <exception cref="ArgumentException">If waypoints has fewer than 1 point or index is invalid</exception>
    public TargetPath(IReadOnlyList<Point2D> waypoints, Point2D currentPosition, int currentWaypointIndex, double speed)
    {
        if (waypoints == null || waypoints.Count < 1)
            throw new ArgumentException("Path must contain at least 1 waypoint", nameof(waypoints));
        if (currentWaypointIndex < 0 || currentWaypointIndex >= waypoints.Count)
            throw new ArgumentException($"Waypoint index {currentWaypointIndex} is out of range [0, {waypoints.Count})", nameof(currentWaypointIndex));
        if (speed < 0)
            throw new ArgumentException("Speed cannot be negative", nameof(speed));

        Waypoints = waypoints;
        CurrentPosition = currentPosition;
        CurrentWaypointIndex = currentWaypointIndex;
        Speed = speed;
    }

    /// <summary>
    /// Creates a path from current position toward a destination.
    /// </summary>
    public static TargetPath FromDestination(Point2D currentPosition, Point2D destination, double speed)
    {
        return new TargetPath(
            new[] { destination },
            currentPosition,
            0,
            speed);
    }

    /// <summary>
    /// Gets the current velocity vector based on direction toward current waypoint.
    /// Returns zero vector if target has reached the final waypoint.
    /// </summary>
    public Vector2D GetCurrentVelocity()
    {
        if (CurrentWaypointIndex >= Waypoints.Count)
            return new Vector2D(0, 0);

        var direction = Waypoints[CurrentWaypointIndex] - CurrentPosition;
        var length = direction.Length;

        if (length < Constants.Epsilon)
            return new Vector2D(0, 0);

        return direction.Normalize() * Speed;
    }

    /// <summary>
    /// Gets the position at a given time in the future, following the path.
    /// After reaching the last waypoint, the target stops there.
    /// </summary>
    /// <param name="time">Time in seconds from now</param>
    /// <returns>Position after traveling along the path for the given time</returns>
    public Point2D GetPositionAtTime(double time)
    {
        if (time <= 0 || Speed < Constants.Epsilon)
            return CurrentPosition;

        double remainingDistance = Speed * time;
        var position = CurrentPosition;
        int waypointIndex = CurrentWaypointIndex;

        while (remainingDistance > 0 && waypointIndex < Waypoints.Count)
        {
            var target = Waypoints[waypointIndex];
            var toTarget = target - position;
            var distanceToTarget = toTarget.Length;

            if (distanceToTarget <= remainingDistance)
            {
                // Reach this waypoint and continue to next
                position = target;
                remainingDistance -= distanceToTarget;
                waypointIndex++;
            }
            else
            {
                // Stop partway to this waypoint
                position = position + toTarget.Normalize() * remainingDistance;
                remainingDistance = 0;
            }
        }

        // Target stops at last waypoint (no continuation)

        return position;
    }

    /// <summary>
    /// Gets the total remaining path length from current position through all waypoints.
    /// </summary>
    public double GetRemainingPathLength()
    {
        if (CurrentWaypointIndex >= Waypoints.Count)
            return 0;

        double length = (Waypoints[CurrentWaypointIndex] - CurrentPosition).Length;

        for (int i = CurrentWaypointIndex; i < Waypoints.Count - 1; i++)
        {
            length += (Waypoints[i + 1] - Waypoints[i]).Length;
        }

        return length;
    }

    /// <summary>
    /// Gets the time remaining to traverse the entire path at current speed.
    /// </summary>
    public double GetRemainingPathTime()
    {
        if (Speed < Constants.Epsilon)
            return double.PositiveInfinity;

        return GetRemainingPathLength() / Speed;
    }

    /// <summary>
    /// Gets the velocity vector at a given time in the future.
    /// Returns the direction toward the waypoint the target will be heading to at that time.
    /// Returns zero vector if the target has reached the final waypoint.
    /// </summary>
    /// <param name="time">Time in seconds from now</param>
    /// <returns>Velocity vector at the given future time</returns>
    public Vector2D GetVelocityAtTime(double time)
    {
        if (time <= 0 || Speed < Constants.Epsilon)
            return GetCurrentVelocity();

        // Find position and determine which segment we're on at that time
        double remainingDistance = Speed * time;
        var position = CurrentPosition;
        int waypointIndex = CurrentWaypointIndex;

        while (remainingDistance > 0 && waypointIndex < Waypoints.Count)
        {
            var target = Waypoints[waypointIndex];
            var toTarget = target - position;
            var distanceToTarget = toTarget.Length;

            if (distanceToTarget <= remainingDistance)
            {
                // Reach this waypoint and continue to next
                position = target;
                remainingDistance -= distanceToTarget;
                waypointIndex++;
            }
            else
            {
                // We're partway on this segment at time t
                // Return velocity toward current target waypoint
                if (distanceToTarget < Constants.Epsilon)
                    return new Vector2D(0, 0);
                return toTarget.Normalize() * Speed;
            }
        }

        // Reached or passed final waypoint - target is stationary
        return new Vector2D(0, 0);
    }

    /// <summary>
    /// Gets the total number of path segments from current position through all remaining waypoints.
    /// </summary>
    public int SegmentCount => CurrentWaypointIndex >= Waypoints.Count ? 0 : Waypoints.Count - CurrentWaypointIndex;

    /// <summary>
    /// Gets a path segment by index (0-based from current position).
    /// More efficient than EnumerateSegments for indexed access patterns.
    /// </summary>
    /// <param name="index">Segment index (0 = current position to first waypoint)</param>
    /// <returns>The path segment at the specified index</returns>
    /// <exception cref="ArgumentOutOfRangeException">If index is out of range</exception>
    public PathSegment GetSegment(int index)
    {
        if (index < 0 || index >= SegmentCount)
            throw new ArgumentOutOfRangeException(nameof(index), $"Segment index {index} is out of range [0, {SegmentCount})");

        if (index == 0)
        {
            return new PathSegment(CurrentPosition, Waypoints[CurrentWaypointIndex], CurrentWaypointIndex);
        }

        int waypointIndex = CurrentWaypointIndex + index;
        return new PathSegment(Waypoints[waypointIndex - 1], Waypoints[waypointIndex], waypointIndex);
    }

    /// <summary>
    /// Enumerates all path segments as (start, end, segmentIndex) tuples.
    /// First segment starts from CurrentPosition.
    /// For performance-critical code, consider using SegmentCount and GetSegment(int) instead.
    /// </summary>
    public IEnumerable<PathSegment> EnumerateSegments()
    {
        if (CurrentWaypointIndex >= Waypoints.Count)
            yield break;

        // First segment: current position to first waypoint
        yield return new PathSegment(CurrentPosition, Waypoints[CurrentWaypointIndex], CurrentWaypointIndex);

        // Remaining segments
        for (int i = CurrentWaypointIndex; i < Waypoints.Count - 1; i++)
        {
            yield return new PathSegment(Waypoints[i], Waypoints[i + 1], i + 1);
        }
    }
}

/// <summary>
/// Represents a single segment of a path.
/// </summary>
/// <param name="Start">Start point of the segment</param>
/// <param name="End">End point of the segment</param>
/// <param name="WaypointIndex">Index of the waypoint at the end of this segment</param>
public readonly record struct PathSegment(Point2D Start, Point2D End, int WaypointIndex)
{
    /// <summary>
    /// Length of this segment.
    /// </summary>
    public double Length => (End - Start).Length;

    /// <summary>
    /// Direction vector (normalized) of this segment.
    /// Returns zero vector for zero-length segments to avoid division by zero.
    /// </summary>
    public Vector2D Direction
    {
        get
        {
            var delta = End - Start;
            var length = delta.Length;
            if (length < Constants.Epsilon)
                return new Vector2D(0, 0);
            return delta.Normalize();
        }
    }
}
