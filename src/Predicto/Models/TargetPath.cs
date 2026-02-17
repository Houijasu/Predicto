using System.Diagnostics;
using MathNet.Spatial.Euclidean;

namespace Predicto.Models;

/// <summary>
/// Represents a multi-waypoint movement path for a target.
/// Targets move along waypoints sequentially at a constant speed.
/// </summary>
[DebuggerDisplay("TargetPath(waypoints={_waypoints.Length}, speed={Speed:F1}, idx={CurrentWaypointIndex})")]
public sealed class TargetPath
{
    /// <summary>
    /// The waypoints defining the path. Target moves from Waypoints[CurrentWaypointIndex] toward the end.
    /// Must contain at least 1 point (destination-only paths are allowed).
    /// </summary>
    public IReadOnlyList<Point2D> Waypoints => _waypoints;

    private readonly Point2D[] _waypoints;

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

    // Pre-computed segment data: _segDistances[i] = distance from segment i start to end
    // _segDirections[i] = normalized direction of segment i
    // Segment 0 = CurrentPosition → Waypoints[CurrentWaypointIndex]
    // Segment k = Waypoints[CurrentWaypointIndex + k - 1] → Waypoints[CurrentWaypointIndex + k]
    private readonly double[] _segDistances;
    private readonly Vector2D[] _segDirections;

    /// <summary>
    /// Creates a new target path.
    /// </summary>
    /// <param name="waypoints">The waypoints defining the path (minimum 1).
    /// If an array is passed, it is stored by reference without copying for performance.
    /// The caller must not mutate the array after construction.</param>
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

        // Store as concrete array for devirtualized indexed access
        if (waypoints is Point2D[] arr)
            _waypoints = arr;
        else
        {
            _waypoints = new Point2D[waypoints.Count];
            for (int i = 0; i < waypoints.Count; i++)
                _waypoints[i] = waypoints[i];
        }

        CurrentPosition = currentPosition;
        CurrentWaypointIndex = currentWaypointIndex;
        Speed = speed;

        int segCount = _waypoints.Length - currentWaypointIndex;
        _segDistances = new double[segCount];
        _segDirections = new Vector2D[segCount];

        var prev = currentPosition;
        for (int i = 0; i < segCount; i++)
        {
            var next = _waypoints[currentWaypointIndex + i];
            var delta = next - prev;
            double len = delta.Length;
            _segDistances[i] = len;
            _segDirections[i] = len < Constants.Epsilon
                ? new Vector2D(0, 0)
                : delta.Normalize();
            prev = next;
        }
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
        if (CurrentWaypointIndex >= _waypoints.Length)
            return new Vector2D(0, 0);

        if (_segDistances[0] < Constants.Epsilon)
            return new Vector2D(0, 0);

        return _segDirections[0] * Speed;
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

        for (int i = 0; i < _segDistances.Length; i++)
        {
            double distanceToTarget = _segDistances[i];

            if (i > 0)
            {
                var delta = _waypoints[CurrentWaypointIndex + i] - position;
                distanceToTarget = delta.Length;
            }

            if (distanceToTarget <= remainingDistance)
            {
                position = _waypoints[CurrentWaypointIndex + i];
                remainingDistance -= distanceToTarget;
            }
            else
            {
                Vector2D dir = (i == 0) ? _segDirections[0] : (_waypoints[CurrentWaypointIndex + i] - position).Normalize();
                position = position + (dir * remainingDistance);
                return position;
            }
        }

        return position;
    }

    /// <summary>
    /// Gets the total remaining path length from current position through all waypoints.
    /// </summary>
    public double GetRemainingPathLength()
    {
        if (_segDistances.Length == 0)
            return 0;

        double length = 0;
        for (int i = 0; i < _segDistances.Length; i++)
        {
            length += _segDistances[i];
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

        double remainingDistance = Speed * time;
        var position = CurrentPosition;

        for (int i = 0; i < _segDistances.Length; i++)
        {
            double distanceToTarget = _segDistances[i];

            if (i > 0)
            {
                var delta = _waypoints[CurrentWaypointIndex + i] - position;
                distanceToTarget = delta.Length;
            }

            if (distanceToTarget <= remainingDistance)
            {
                position = _waypoints[CurrentWaypointIndex + i];
                remainingDistance -= distanceToTarget;
            }
            else
            {
                if (distanceToTarget < Constants.Epsilon)
                    return new Vector2D(0, 0);
                Vector2D dir = (i == 0) ? _segDirections[0] : (_waypoints[CurrentWaypointIndex + i] - position).Normalize();
                return dir * Speed;
            }
        }

        return new Vector2D(0, 0);
    }

    /// <summary>
    /// Gets the total number of path segments from current position through all remaining waypoints.
    /// </summary>
    public int SegmentCount => CurrentWaypointIndex >= _waypoints.Length ? 0 : _waypoints.Length - CurrentWaypointIndex;

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
            return new PathSegment(CurrentPosition, _waypoints[CurrentWaypointIndex], CurrentWaypointIndex);
        }

        int waypointIndex = CurrentWaypointIndex + index;
        return new PathSegment(_waypoints[waypointIndex - 1], _waypoints[waypointIndex], waypointIndex);
    }

    /// <summary>
    /// Enumerates all path segments as (start, end, segmentIndex) tuples.
    /// First segment starts from CurrentPosition.
    /// For performance-critical code, consider using SegmentCount and GetSegment(int) instead.
    /// </summary>
    public IEnumerable<PathSegment> EnumerateSegments()
    {
        if (CurrentWaypointIndex >= _waypoints.Length)
            yield break;

        // First segment: current position to first waypoint
        yield return new PathSegment(CurrentPosition, _waypoints[CurrentWaypointIndex], CurrentWaypointIndex);

        // Remaining segments
        for (int i = CurrentWaypointIndex; i < _waypoints.Length - 1; i++)
        {
            yield return new PathSegment(_waypoints[i], _waypoints[i + 1], i + 1);
        }
    }
}

/// <summary>
/// Represents a single segment of a path with pre-computed length and direction.
/// </summary>
[DebuggerDisplay("Segment[{WaypointIndex}] len={Length:F1}")]
public readonly struct PathSegment
{
    public Point2D Start { get; }
    public Point2D End { get; }
    public int WaypointIndex { get; }

    /// <summary>
    /// Pre-computed segment length (avoids repeated sqrt).
    /// </summary>
    public double Length { get; }

    /// <summary>
    /// Pre-computed normalized direction. Zero vector for degenerate segments.
    /// </summary>
    public Vector2D Direction { get; }

    public PathSegment(Point2D start, Point2D end, int waypointIndex)
    {
        Start = start;
        End = end;
        WaypointIndex = waypointIndex;

        var delta = end - start;
        Length = delta.Length;
        Direction = Length < Constants.Epsilon
            ? new Vector2D(0, 0)
            : delta.Normalize();
    }
}
