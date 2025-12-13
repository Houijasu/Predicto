using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using MathNet.Spatial.Euclidean;
using Predicto;
using Predicto.Models;
using Predicto.Solvers;
using Raylib_cs;

static bool TryGetArg(string[] args, string key, out string value)
{
    for (int i = 0; i < args.Length - 1; i++)
    {
        if (string.Equals(args[i], key, StringComparison.OrdinalIgnoreCase))
        {
            value = args[i + 1];
            return true;
        }
    }

    value = string.Empty;
    return false;
}

static bool TryGetArgDouble(string[] args, string key, out double value)
{
    if (TryGetArg(args, key, out string raw)
        && double.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
    {
        return true;
    }

    value = default;
    return false;
}

static double GetBenchWidth(string[] args, double defaultWidth)
{
    if (TryGetArgDouble(args, "--bench-width", out double width))
        return width;

    if (TryGetArgDouble(args, "--bench-width-mult", out double mult))
        return defaultWidth * mult;

    return defaultWidth;
}

if (args.Contains("--bench", StringComparer.OrdinalIgnoreCase))
{
    const double defaultBenchWidth = 70.0;
    const double defaultTargetHitboxRadius = 65.0;
    const double defaultMargin = 1.0;

    double benchWidth = GetBenchWidth(args, defaultBenchWidth);
    double targetHitboxRadius = TryGetArgDouble(args, "--bench-hitbox", out double hitbox)
        ? hitbox
        : defaultTargetHitboxRadius;
    double margin = TryGetArgDouble(args, "--bench-margin", out double m)
        ? m
        : defaultMargin;

    RunBench(targetHitboxRadius, benchWidth, margin);
    return;
}

const int ScreenWidth = 1600;
const int ScreenHeight = 1000;

Raylib.InitWindow(ScreenWidth, ScreenHeight, "Predicto - Ultimate Behind-Edge Prediction");
Raylib.SetTargetFPS(60);

static void RunBench(double targetHitboxRadius, double skillshotWidth, double margin)
{
    CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

    const int scenarioCount = 2048;
    const int warmupIterations = 5_000;
    const int measureIterations = 200_000;

    var rng = new Random(12345);
    var scenarios = new (Point2D Caster, Point2D Target, Vector2D Vel, double Speed, double Delay, double Range)[scenarioCount];

    for (int i = 0; i < scenarios.Length; i++)
    {
        double casterX = rng.NextDouble() * 2000 - 1000;
        double casterY = rng.NextDouble() * 2000 - 1000;
        double targetX = rng.NextDouble() * 2000 - 1000;
        double targetY = rng.NextDouble() * 2000 - 1000;

        // Target speed in typical LoL-ish units.
        double targetSpeed = 50 + rng.NextDouble() * 650;
        double angle = rng.NextDouble() * Math.PI * 2;
        var vel = new Vector2D(Math.Cos(angle) * targetSpeed, Math.Sin(angle) * targetSpeed);

        // Skillshot parameters.
        double skillshotSpeed = 800 + rng.NextDouble() * 2600;
        double castDelay = rng.NextDouble() * 0.75;
        double skillshotRange = 800 + rng.NextDouble() * 2200;

        scenarios[i] = (new Point2D(casterX, casterY), new Point2D(targetX, targetY), vel, skillshotSpeed, castDelay, skillshotRange);
    }

    // Warmup (JIT + caches)
    double sink = 0;
    for (int i = 0; i < warmupIterations; i++)
    {
        var s = scenarios[i % scenarios.Length];
        sink += InterceptSolver.SolveInterceptTimeWithNewtonRefinement_ForBenchmark(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay, s.Range) ?? 0;
    }
    for (int i = 0; i < warmupIterations; i++)
    {
        var s = scenarios[i % scenarios.Length];
        sink += InterceptSolver.SolveInterceptTimeWithSecantRefinement_Legacy_ForBenchmark(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay, s.Range) ?? 0;
    }

    var (newtonMs, newtonFound, newtonResidual) = Measure(
        "Newton",
        scenarios,
        measureIterations,
        static s => InterceptSolver.SolveInterceptTimeWithNewtonRefinement_ForBenchmark(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay, s.Range));

    var (secantMs, secantFound, secantResidual) = Measure(
        "Secant (legacy)",
        scenarios,
        measureIterations,
        static s => InterceptSolver.SolveInterceptTimeWithSecantRefinement_Legacy_ForBenchmark(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay, s.Range));

    // Prevent dead-code elimination.
    if (sink == double.MaxValue)
        Console.WriteLine("impossible");

    Console.WriteLine();
    Console.WriteLine("=== InterceptSolver Benchmark (quadratic + refinement) ===");
    Console.WriteLine($"Scenarios: {scenarioCount}, iterations: {measureIterations}");

    Print("Newton", newtonMs, measureIterations, newtonFound, newtonResidual);
    Print("Secant (legacy)", secantMs, measureIterations, secantFound, secantResidual);

    Console.WriteLine();
    PrintDelta("Secant vs Newton", secantMs, newtonMs);
    PrintDelta("Newton vs Secant", newtonMs, secantMs);

    static void PrintDelta(string label, double aMs, double bMs)
    {
        if (bMs <= 0)
            return;

        // Example: (b - a)/b = how much faster a is than b
        double speedup = bMs / aMs;
        double fasterPct = (bMs - aMs) / bMs * 100.0;
        double slowerPct = (aMs - bMs) / bMs * 100.0;

        if (aMs <= bMs)
            Console.WriteLine($"{label,-18}: {speedup:F3}x ({fasterPct:F2}% faster)");
        else
            Console.WriteLine($"{label,-18}: {speedup:F3}x ({slowerPct:F2}% slower)");
    }

    static (double ElapsedMs, int Found, double AvgAbsResidual) Measure(
        string name,
        (Point2D Caster, Point2D Target, Vector2D Vel, double Speed, double Delay, double Range)[] scenarios,
        int iterations,
        Func<(Point2D Caster, Point2D Target, Vector2D Vel, double Speed, double Delay, double Range), double?> solver)
    {
        var sw = Stopwatch.StartNew();
        int found = 0;
        double residualSum = 0;

        for (int i = 0; i < iterations; i++)
        {
            var s = scenarios[i % scenarios.Length];
            double? t = solver(s);
            if (t.HasValue)
            {
                found++;
                // Residual for center-to-center: |D + Vt| - s*max(0,t-d)
                var displacement = s.Target - s.Caster;
                var D = new Vector2D(displacement.X, displacement.Y);
                var relative = D + s.Vel * t.Value;
                double dist = relative.Length;
                double flight = Math.Max(0, t.Value - s.Delay);
                double proj = s.Speed * flight;
                residualSum += Math.Abs(dist - proj);
            }
        }

        sw.Stop();
        double avgResidual = found > 0 ? residualSum / found : double.NaN;
        return (sw.Elapsed.TotalMilliseconds, found, avgResidual);
    }

    static void Print(string label, double ms, int iters, int found, double avgAbsResidual)
    {
        double nsPerOp = (ms * 1_000_000.0) / iters;
        Console.WriteLine($"{label,-16}  {ms,10:F2} ms  {nsPerOp,10:F1} ns/op  found={found,7}  avg|res|={avgAbsResidual:F6}");
    }
    
    // ========================================
    // BEHIND-TARGET METHOD COMPARISON
    // ========================================
    Console.WriteLine();
    Console.WriteLine("=== Behind-Target Method Comparison ===");
    
    // Extended scenarios with hitbox and skillshot width for behind-target methods
    // Controlled via CLI args: --bench-width, --bench-width-mult, --bench-hitbox, --bench-margin
    
    // Warmup for behind-target methods
    for (int i = 0; i < warmupIterations; i++)
    {
        var s = scenarios[i % scenarios.Length];
        var result1 = InterceptSolver.SolveBehindTargetWithFullRefinement(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay, 
            targetHitboxRadius, skillshotWidth, s.Range, margin);
        if (result1.HasValue) sink += result1.Value.InterceptTime;
        
        var result2 = InterceptSolver.SolveBehindTargetDirect(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay, 
            targetHitboxRadius, skillshotWidth, s.Range, margin);
        if (result2.HasValue) sink += result2.Value.InterceptTime;
    }
    
    // Measure FullRefinement method
    var swFullRef = Stopwatch.StartNew();
    int fullRefFound = 0;
    double fullRefResidualSum = 0;
    for (int i = 0; i < measureIterations; i++)
    {
        var s = scenarios[i % scenarios.Length];
        var result = InterceptSolver.SolveBehindTargetWithFullRefinement(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay,
            targetHitboxRadius, skillshotWidth, s.Range, margin);
        if (result.HasValue)
        {
            fullRefFound++;
            double effectiveRadius = targetHitboxRadius + skillshotWidth / 2 - margin;
            var aimToTarget = result.Value.PredictedTargetPosition - result.Value.AimPoint;
            fullRefResidualSum += Math.Abs(aimToTarget.Length - effectiveRadius);
        }
    }
    swFullRef.Stop();
    double fullRefMs = swFullRef.Elapsed.TotalMilliseconds;
    double fullRefAvgResidual = fullRefFound > 0 ? fullRefResidualSum / fullRefFound : double.NaN;
    
    // Measure Direct method
    var swDirect = Stopwatch.StartNew();
    int directFound = 0;
    double directResidualSum = 0;
    for (int i = 0; i < measureIterations; i++)
    {
        var s = scenarios[i % scenarios.Length];
        var result = InterceptSolver.SolveBehindTargetDirect(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay,
            targetHitboxRadius, skillshotWidth, s.Range, margin);
        if (result.HasValue)
        {
            directFound++;
            double effectiveRadius = targetHitboxRadius + skillshotWidth / 2 - margin;
            var aimToTarget = result.Value.PredictedTargetPosition - result.Value.AimPoint;
            directResidualSum += Math.Abs(aimToTarget.Length - effectiveRadius);
        }
    }
    swDirect.Stop();
    double directMs = swDirect.Elapsed.TotalMilliseconds;
    double directAvgResidual = directFound > 0 ? directResidualSum / directFound : double.NaN;
    
    Console.WriteLine($"Scenarios: {scenarioCount}, iterations: {measureIterations}");
    Console.WriteLine($"Target hitbox: {targetHitboxRadius}, Skillshot width: {skillshotWidth}, Margin: {margin}");
    Console.WriteLine($"Args: --bench-width {skillshotWidth} (or --bench-width-mult X)");
    Console.WriteLine();
    
    double fullRefNsPerOp = (fullRefMs * 1_000_000.0) / measureIterations;
    double directNsPerOp = (directMs * 1_000_000.0) / measureIterations;
    
    Console.WriteLine($"{"FullRefinement",-18}  {fullRefMs,10:F2} ms  {fullRefNsPerOp,10:F1} ns/op  found={fullRefFound,7}  avg|res|={fullRefAvgResidual:F9}");
    Console.WriteLine($"{"Direct",-18}  {directMs,10:F2} ms  {directNsPerOp,10:F1} ns/op  found={directFound,7}  avg|res|={directAvgResidual:F9}");
    
    Console.WriteLine();
    
    // Compare speed
    if (directMs < fullRefMs)
    {
        double speedup = fullRefMs / directMs;
        double fasterPct = (fullRefMs - directMs) / fullRefMs * 100.0;
        Console.WriteLine($"Direct vs FullRefinement: {speedup:F3}x ({fasterPct:F2}% faster)");
    }
    else
    {
        double slowdown = directMs / fullRefMs;
        double slowerPct = (directMs - fullRefMs) / fullRefMs * 100.0;
        Console.WriteLine($"Direct vs FullRefinement: {slowdown:F3}x ({slowerPct:F2}% slower)");
    }
    
    // Compare accuracy
    Console.WriteLine($"Accuracy comparison: FullRefinement avg residual = {fullRefAvgResidual:F12}, Direct avg residual = {directAvgResidual:F12}");
    if (directAvgResidual < fullRefAvgResidual)
        Console.WriteLine($"  -> Direct is more accurate");
    else if (fullRefAvgResidual < directAvgResidual)
        Console.WriteLine($"  -> FullRefinement is more accurate");
    else
        Console.WriteLine($"  -> Both methods have same accuracy");
    
    // ========================================
    // HIT SIMULATION TEST
    // ========================================
    Console.WriteLine();
    Console.WriteLine("=== Hit Simulation Test (validates actual collision) ===");
    
    int fullRefHits = 0, fullRefMisses = 0;
    int directHits = 0, directMisses = 0;
    double effectiveCollisionRadius = targetHitboxRadius + skillshotWidth / 2;
    
    // Track max difference between methods
    double maxTimeDiff = 0;
    double maxAimDiff = 0;
    
    for (int i = 0; i < scenarios.Length; i++)
    {
        var s = scenarios[i];
        
        // Test FullRefinement method
        var fullRefResult = InterceptSolver.SolveBehindTargetWithFullRefinement(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay,
            targetHitboxRadius, skillshotWidth, s.Range, margin);
        
        if (fullRefResult.HasValue)
        {
            double t = fullRefResult.Value.InterceptTime;
            var predictedTarget = s.Target + s.Vel * t;
            var aimDir = (fullRefResult.Value.AimPoint - s.Caster);
            var aimDirNorm = aimDir / aimDir.Length;
            double flightTime = Math.Max(0, t - s.Delay);
            var skillshotPos = s.Caster + aimDirNorm * s.Speed * flightTime;
            double dist = (skillshotPos - predictedTarget).Length;
            if (dist <= effectiveCollisionRadius)
                fullRefHits++;
            else
                fullRefMisses++;
        }
        
        // Test Direct method
        var directResult = InterceptSolver.SolveBehindTargetDirect(
            s.Caster, s.Target, s.Vel, s.Speed, s.Delay,
            targetHitboxRadius, skillshotWidth, s.Range, margin);
        
        if (directResult.HasValue)
        {
            double t = directResult.Value.InterceptTime;
            var predictedTarget = s.Target + s.Vel * t;
            var aimDir = (directResult.Value.AimPoint - s.Caster);
            var aimDirNorm = aimDir / aimDir.Length;
            double flightTime = Math.Max(0, t - s.Delay);
            var skillshotPos = s.Caster + aimDirNorm * s.Speed * flightTime;
            double dist = (skillshotPos - predictedTarget).Length;
            if (dist <= effectiveCollisionRadius)
                directHits++;
            else
            {
                directMisses++;
                if (directMisses <= 5)
                {
                    Console.WriteLine($"  Direct MISS #{directMisses}: dist={dist:F2}, needed<={effectiveCollisionRadius:F2}, gap={dist - effectiveCollisionRadius:F2}");
                    Console.WriteLine($"    Caster=({s.Caster.X:F1},{s.Caster.Y:F1}) Target=({s.Target.X:F1},{s.Target.Y:F1}) Vel=({s.Vel.X:F1},{s.Vel.Y:F1})");
                }
            }
        }
        
        // Compare results when both methods succeed
        if (fullRefResult.HasValue && directResult.HasValue)
        {
            double timeDiff = Math.Abs(fullRefResult.Value.InterceptTime - directResult.Value.InterceptTime);
            double aimDiff = (fullRefResult.Value.AimPoint - directResult.Value.AimPoint).Length;
            maxTimeDiff = Math.Max(maxTimeDiff, timeDiff);
            maxAimDiff = Math.Max(maxAimDiff, aimDiff);
        }
    }
    
    Console.WriteLine($"FullRefinement: {fullRefHits} hits, {fullRefMisses} misses out of {fullRefHits + fullRefMisses} attempts ({100.0 * fullRefHits / (fullRefHits + fullRefMisses):F2}% hit rate)");
    Console.WriteLine($"Direct:         {directHits} hits, {directMisses} misses out of {directHits + directMisses} attempts ({100.0 * directHits / (directHits + directMisses):F2}% hit rate)");
    Console.WriteLine();
    Console.WriteLine($"Max difference between methods: time={maxTimeDiff:E3}s, aim={maxAimDiff:E3} units");
    
    // Prevent dead-code elimination
    if (sink == double.MaxValue)
        Console.WriteLine("impossible sink");
}

// Load Liberation Sans font (clean, readable)
var font = Raylib.LoadFontEx("/usr/share/fonts/liberation/LiberationSans-Regular.ttf", 24, null, 0);
if (font.BaseSize == 0)
    font = Raylib.GetFontDefault();

var ultimate = new Ultimate();

// Camera/Zoom
var camera = new Camera2D
{
    Target = new Vector2(ScreenWidth / 2, ScreenHeight / 2),
    Offset = new Vector2(ScreenWidth / 2, ScreenHeight / 2),
    Rotation = 0f,
    Zoom = 1f
};
float targetZoom = 1f;

// Time control
float[] timeMultipliers = { 0.1f, 0.25f, 0.5f, 1f, 2f };
string[] timeLabels = { "0.1x", "0.25x", "0.5x", "1x", "2x" };
int currentTimeIndex = 3; // Start at 1x

// Simulation state
var casterPos = new Vector2(300, 400);
var targetPos = new Vector2(800, 400);
var targetVelocity = new Vector2(0, -1);
float targetSpeed = 350f;
// Skillshot parameters (defaults will be overwritten by preset)
float skillshotSpeed = 2000f;
float skillshotRange = 2000f;
float skillshotWidth = 40f;
float skillshotDelay = 0.25f;
float skillshotRadius = 200f;  // For circular spells
float targetHitbox = 65f;

// Animation state
bool isFiring = false;
float fireTime = 0f;
Vector2 skillshotPos = Vector2.Zero;
Vector2 skillshotDir = Vector2.Zero;
Vector2 fireTargetStartPos = Vector2.Zero;
Vector2 fireAimPos = Vector2.Zero;
Vector2 currentAnimTargetPos = Vector2.Zero;  // Current animated target position during firing
bool skillshotLaunched = false;
string hitResult = "";
float hitResultTimer = 0f;
int hitCount = 0;
int missCount = 0;

// Circular spell animation state
bool isCircularFiring = false;  // Whether current shot is circular
float circularAnimRadius = 0f;  // Current expanding radius during animation
float circularTargetRadius = 0f;  // Target radius for the spell

// Trail effect
List<Vector2> skillshotTrail = new();

// Path mode - multi-waypoint support
List<Vector2> waypoints = new();
bool pathModeActive = false;
TargetPath? activePath = null;  // Used during firing animation
Vector2 pathStartPos = Vector2.Zero;  // Starting position when path was created
float pathElapsedTime = 0f;  // Time elapsed since path started (unused now, kept for potential future use)
bool continuousFiring = false;  // Whether we're in continuous fire mode (path mode)
float pathTotalTime = 0f;  // Total time elapsed on path during continuous firing

// Prediction result
PredictionResult? ultimateResult = null;
PredictionResult? trailingEdgeResult = null;  // Pure trailing-edge prediction for comparison

// Collision info for display
Vector2 collisionPos = Vector2.Zero;
float collisionTime = 0f;
float collisionDisplayTimer = 0f;

// Debug info for miss logging
int currentShotNumber = 0;
float shotPathTotalTime = 0f;
Vector2 shotCasterPos = Vector2.Zero;
Vector2 shotTargetStartPos = Vector2.Zero;
Vector2 shotAimPos = Vector2.Zero;
Vector2 shotPredictedTargetPos = Vector2.Zero;
double shotInterceptTime = 0;
double shotConfidence = 0;
List<Vector2> shotWaypoints = new();
Vector2 shotCurrentPathPos = Vector2.Zero;  // Target position on path when shot started
Vector2 shotPrevWaypoint = Vector2.Zero;    // Previous waypoint (or start pos)
Vector2 shotNextWaypoint = Vector2.Zero;    // Next waypoint target is heading toward
int shotCurrentWaypointIndex = -1;          // Which waypoint we're heading toward

// Presets - Linear skillshots
var linearPresets = new (string Name, Vector2 Vel, float Speed, float Range, float Width, float Delay)[]
{
    ("Ezreal Q", new Vector2(0, -1), 2000, 1150, 60, 0.25f),
    ("Morgana Q", new Vector2(0, -1), 1200, 1175, 70, 0.25f),
    ("Blitzcrank Q", new Vector2(0, -1), 1800, 1150, 70, 0.25f),
    ("Nidalee Q", new Vector2(0, -1), 1300, 1500, 40, 0.25f),
    ("Lux Q", new Vector2(0, -1), 1200, 1175, 70, 0.25f),
    ("Thresh Q", new Vector2(0, -1), 1900, 1100, 70, 0.5f),
    ("Jinx W", new Vector2(0, -1), 3300, 1450, 60, 0.6f),
    ("Ashe R", new Vector2(0, -1), 1600, 2000, 130, 0.25f),
};

// Presets - Circular skillshots (ground-targeted, instant)
var circularPresets = new (string Name, float Radius, float Range, float Delay)[]
{
    ("Xerath W", 200, 1100, 0.5f),
    ("Xerath W Center", 50, 1100, 0.5f),
    ("Veigar W", 112, 950, 1.2f),
    ("Cho'Gath Q", 175, 950, 0.5f),
    ("Ziggs W", 150, 1000, 0.25f),
    ("Ziggs E", 250, 900, 0.25f),
};

int currentLinearPreset = 3; // Start with Nidalee Q
int currentCircularPreset = 0; // Start with Xerath W
bool isCircularMode = true; // Start with Xerath W (circular mode)

while (!Raylib.WindowShouldClose())
{
    float rawDt = Raylib.GetFrameTime();
    float timeMultiplier = timeMultipliers[currentTimeIndex];
    float dt = rawDt * timeMultiplier;

    // Zoom with mouse wheel (cursor-centered)
    float wheel = Raylib.GetMouseWheelMove();
    if (wheel != 0)
    {
        var mousePos = Raylib.GetMousePosition();
        var mouseWorldBefore = Raylib.GetScreenToWorld2D(mousePos, camera);
        targetZoom += wheel * 0.15f;
        targetZoom = Math.Clamp(targetZoom, 0.2f, 5f);
        camera.Zoom = targetZoom;
        var mouseWorldAfter = Raylib.GetScreenToWorld2D(mousePos, camera);
        camera.Target += mouseWorldBefore - mouseWorldAfter;
    }

    // Pan camera with middle mouse
    if (Raylib.IsMouseButtonDown(MouseButton.Middle))
    {
        var delta = Raylib.GetMouseDelta();
        camera.Target -= delta / camera.Zoom;
    }

    // Reset camera with Home
    if (Raylib.IsKeyPressed(KeyboardKey.Home))
    {
        camera.Target = new Vector2(ScreenWidth / 2, ScreenHeight / 2);
        targetZoom = 1f;
    }

    // Time multiplier buttons (1-5 keys)
    if (Raylib.IsKeyPressed(KeyboardKey.One)) currentTimeIndex = 0;
    if (Raylib.IsKeyPressed(KeyboardKey.Two)) currentTimeIndex = 1;
    if (Raylib.IsKeyPressed(KeyboardKey.Three)) currentTimeIndex = 2;
    if (Raylib.IsKeyPressed(KeyboardKey.Four)) currentTimeIndex = 3;
    if (Raylib.IsKeyPressed(KeyboardKey.Five)) currentTimeIndex = 4;

    // Time button click detection
    if (Raylib.IsMouseButtonPressed(MouseButton.Left))
    {
        int btnY = ScreenHeight - 55;
        int btnX = 20;
        int btnW = 50;
        int btnH = 28;
        var mousePos = Raylib.GetMousePosition();
        for (int i = 0; i < timeMultipliers.Length; i++)
        {
            var btnRect = new Rectangle(btnX + i * (btnW + 5), btnY, btnW, btnH);
            if (Raylib.CheckCollisionPointRec(mousePos, btnRect))
            {
                currentTimeIndex = i;
                break;
            }
        }
    }

    // Input handling
    Vector2 mouseWorld = Raylib.GetScreenToWorld2D(Raylib.GetMousePosition(), camera);
    HandleInput(ref casterPos, ref targetPos, ref targetVelocity, ref targetSpeed,
                ref skillshotSpeed, ref skillshotRange, ref skillshotWidth, ref skillshotDelay, ref skillshotRadius,
                linearPresets, circularPresets, ref currentLinearPreset, ref currentCircularPreset, ref isCircularMode,
                isFiring, mouseWorld, waypoints, ref pathModeActive,
                ref pathStartPos, ref pathElapsedTime);

    // Target stays stationary while drawing path - only moves when firing

    // Run prediction - use appropriate type based on mode
    if (isCircularMode)
    {
        // Circular prediction
        CircularPredictionInput circularInput;
        var circularSkillshot = new CircularSkillshot(skillshotRadius, skillshotRange, skillshotDelay);

        if (pathModeActive && waypoints.Count > 0)
        {
            var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
            var fullPath = new TargetPath(
                waypointsList,
                new Point2D(pathStartPos.X, pathStartPos.Y),
                0,
                targetSpeed);

            // Find which segment the target is currently on
            double remainingDistance = targetSpeed * pathElapsedTime;
            var position = new Point2D(pathStartPos.X, pathStartPos.Y);
            int currentWaypointIndex = 0;

            while (remainingDistance > 0 && currentWaypointIndex < waypointsList.Count)
            {
                var target = waypointsList[currentWaypointIndex];
                var distanceToTarget = (target - position).Length;

                if (distanceToTarget <= remainingDistance)
                {
                    position = target;
                    remainingDistance -= distanceToTarget;
                    currentWaypointIndex++;
                }
                else
                {
                    break;
                }
            }

            if (currentWaypointIndex >= waypointsList.Count)
            {
                // Stationary at last waypoint
                circularInput = new CircularPredictionInput(
                    new Point2D(casterPos.X, casterPos.Y),
                    new Point2D(targetPos.X, targetPos.Y),
                    new Vector2D(0, 0),
                    circularSkillshot,
                    targetHitbox);
            }
            else
            {
                var remainingWaypoints = waypointsList.Skip(currentWaypointIndex).ToList();
                var path = new TargetPath(
                    remainingWaypoints,
                    new Point2D(targetPos.X, targetPos.Y),
                    0,
                    targetSpeed);

                circularInput = CircularPredictionInput.WithPath(
                    new Point2D(casterPos.X, casterPos.Y),
                    path,
                    circularSkillshot,
                    targetHitbox);
            }
        }
        else
        {
            circularInput = new CircularPredictionInput(
                new Point2D(casterPos.X, casterPos.Y),
                new Point2D(targetPos.X, targetPos.Y),
                new Vector2D(targetVelocity.X * targetSpeed, targetVelocity.Y * targetSpeed),
                circularSkillshot,
                targetHitbox);
        }

        ultimateResult = ultimate.PredictCircular(circularInput);
        trailingEdgeResult = ultimateResult;  // Circular doesn't have blending yet, same result
    }
    else
    {
        // Linear prediction
        PredictionInput input;
        if (pathModeActive && waypoints.Count > 0)
        {
            var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
            var fullPath = new TargetPath(
                waypointsList,
                new Point2D(pathStartPos.X, pathStartPos.Y),
                0,
                targetSpeed);

            double remainingDistance = targetSpeed * pathElapsedTime;
            var position = new Point2D(pathStartPos.X, pathStartPos.Y);
            int currentWaypointIndex = 0;

            while (remainingDistance > 0 && currentWaypointIndex < waypointsList.Count)
            {
                var target = waypointsList[currentWaypointIndex];
                var distanceToTarget = (target - position).Length;

                if (distanceToTarget <= remainingDistance)
                {
                    position = target;
                    remainingDistance -= distanceToTarget;
                    currentWaypointIndex++;
                }
                else
                {
                    break;
                }
            }

            if (currentWaypointIndex >= waypointsList.Count)
            {
                input = new PredictionInput(
                    new Point2D(casterPos.X, casterPos.Y),
                    new Point2D(targetPos.X, targetPos.Y),
                    new Vector2D(0, 0),
                    new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                    targetHitbox);
            }
            else
            {
                var remainingWaypoints = waypointsList.Skip(currentWaypointIndex).ToList();
                var path = new TargetPath(
                    remainingWaypoints,
                    new Point2D(targetPos.X, targetPos.Y),
                    0,
                    targetSpeed);

                input = PredictionInput.WithPath(
                    new Point2D(casterPos.X, casterPos.Y),
                    path,
                    new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                    targetHitbox);
            }
        }
        else
        {
            input = new PredictionInput(
                new Point2D(casterPos.X, casterPos.Y),
                new Point2D(targetPos.X, targetPos.Y),
                new Vector2D(targetVelocity.X * targetSpeed, targetVelocity.Y * targetSpeed),
                new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                targetHitbox);
        }

        ultimateResult = ultimate.Predict(input);
        trailingEdgeResult = ultimate.PredictPureTrailingEdge(input);
    }

    // Stop continuous firing with Escape
    if (Raylib.IsKeyPressed(KeyboardKey.Escape) && continuousFiring)
    {
        continuousFiring = false;
        isFiring = false;
    }

    // Fire skillshot with Space
    if (Raylib.IsKeyPressed(KeyboardKey.Space) && !isFiring && ultimateResult is PredictionResult.Hit)
    {
        // Start continuous firing mode if in path mode
        continuousFiring = pathModeActive && waypoints.Count > 0;
        pathTotalTime = 0f;

        StartNewShot();
    }

    // Helper to start a new shot
    void StartNewShot()
    {
        PredictionResult? shotResult;

        // Store whether this shot is circular
        isCircularFiring = isCircularMode;

        if (isCircularMode)
        {
            // Circular prediction
            CircularPredictionInput shotInput;
            var circularSkillshot = new CircularSkillshot(skillshotRadius, skillshotRange, skillshotDelay);

            if (continuousFiring && waypoints.Count > 0)
            {
                var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
                var fullPath = new TargetPath(waypointsList, new Point2D(pathStartPos.X, pathStartPos.Y), 0, targetSpeed);
                var currentPathPos = fullPath.GetPositionAtTime(pathTotalTime);

                double remainingDistance = targetSpeed * pathTotalTime;
                var position = new Point2D(pathStartPos.X, pathStartPos.Y);
                int currentWaypointIndex = 0;

                while (remainingDistance > 0 && currentWaypointIndex < waypointsList.Count)
                {
                    var target = waypointsList[currentWaypointIndex];
                    var distanceToTarget = (target - position).Length;
                    if (distanceToTarget <= remainingDistance)
                    {
                        position = target;
                        remainingDistance -= distanceToTarget;
                        currentWaypointIndex++;
                    }
                    else break;
                }

                if (currentWaypointIndex >= waypointsList.Count)
                {
                    continuousFiring = false;
                    isFiring = false;
                    return;
                }

                var remainingWaypoints = waypointsList.Skip(currentWaypointIndex).ToList();
                var pathForPrediction = new TargetPath(remainingWaypoints, currentPathPos, 0, targetSpeed);

                shotInput = CircularPredictionInput.WithPath(
                    new Point2D(casterPos.X, casterPos.Y),
                    pathForPrediction,
                    circularSkillshot,
                    targetHitbox);

                activePath = fullPath;
            }
            else
            {
                shotInput = new CircularPredictionInput(
                    new Point2D(casterPos.X, casterPos.Y),
                    new Point2D(targetPos.X, targetPos.Y),
                    new Vector2D(targetVelocity.X * targetSpeed, targetVelocity.Y * targetSpeed),
                    circularSkillshot,
                    targetHitbox);
                activePath = null;
            }

            shotResult = ultimate.PredictCircular(shotInput);
            circularTargetRadius = skillshotRadius;
        }
        else
        {
            // Linear prediction
            PredictionInput shotInput;

            if (continuousFiring && waypoints.Count > 0)
            {
                var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
                var fullPath = new TargetPath(waypointsList, new Point2D(pathStartPos.X, pathStartPos.Y), 0, targetSpeed);
                var currentPathPos = fullPath.GetPositionAtTime(pathTotalTime);

                double remainingDistance = targetSpeed * pathTotalTime;
                var position = new Point2D(pathStartPos.X, pathStartPos.Y);
                int currentWaypointIndex = 0;

                while (remainingDistance > 0 && currentWaypointIndex < waypointsList.Count)
                {
                    var target = waypointsList[currentWaypointIndex];
                    var distanceToTarget = (target - position).Length;
                    if (distanceToTarget <= remainingDistance)
                    {
                        position = target;
                        remainingDistance -= distanceToTarget;
                        currentWaypointIndex++;
                    }
                    else break;
                }

                if (currentWaypointIndex >= waypointsList.Count)
                {
                    continuousFiring = false;
                    isFiring = false;
                    return;
                }

                var remainingWaypoints = waypointsList.Skip(currentWaypointIndex).ToList();
                var pathForPrediction = new TargetPath(remainingWaypoints, currentPathPos, 0, targetSpeed);

                shotInput = PredictionInput.WithPath(
                    new Point2D(casterPos.X, casterPos.Y),
                    pathForPrediction,
                    new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                    targetHitbox);

                activePath = fullPath;
            }
            else
            {
                shotInput = new PredictionInput(
                    new Point2D(casterPos.X, casterPos.Y),
                    new Point2D(targetPos.X, targetPos.Y),
                    new Vector2D(targetVelocity.X * targetSpeed, targetVelocity.Y * targetSpeed),
                    new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                    targetHitbox);
                activePath = null;
            }

            shotResult = ultimate.Predict(shotInput);
        }

        if (shotResult is PredictionResult.Hit hit)
        {
            // DEBUG: Behind-edge / trailing edge calculation verification
            float aimToTargetDist = Vector2.Distance(
                new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y),
                new Vector2((float)hit.PredictedTargetPosition.X, (float)hit.PredictedTargetPosition.Y));
            float effectiveRadius = isCircularFiring
                ? (float)(targetHitbox + skillshotRadius)
                : targetHitbox + skillshotWidth / 2;
            float trailingEdgeDist = effectiveRadius - aimToTargetDist;
            float casterToAimDist = Vector2.Distance(casterPos, new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y));

            // Store debug info
            currentShotNumber = hitCount + missCount + 1;
            shotPathTotalTime = pathTotalTime;
            shotCasterPos = casterPos;
            shotTargetStartPos = targetPos;
            shotAimPos = new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y);
            shotPredictedTargetPos = new Vector2((float)hit.PredictedTargetPosition.X, (float)hit.PredictedTargetPosition.Y);
            shotInterceptTime = hit.InterceptTime;
            shotConfidence = hit.Confidence;
            shotWaypoints = continuousFiring && waypoints.Count > 0 ? new List<Vector2>(waypoints) : new List<Vector2>();

            // Store path position info
            if (continuousFiring && waypoints.Count > 0)
            {
                var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
                var fullPath = new TargetPath(waypointsList, new Point2D(pathStartPos.X, pathStartPos.Y), 0, targetSpeed);
                var currentPos = fullPath.GetPositionAtTime(pathTotalTime);
                shotCurrentPathPos = new Vector2((float)currentPos.X, (float)currentPos.Y);

                double remainingDist = targetSpeed * pathTotalTime;
                var pos = new Point2D(pathStartPos.X, pathStartPos.Y);
                int wpIdx = 0;
                while (remainingDist > 0 && wpIdx < waypointsList.Count)
                {
                    var tgt = waypointsList[wpIdx];
                    var dist = (tgt - pos).Length;
                    if (dist <= remainingDist) { pos = tgt; remainingDist -= dist; wpIdx++; }
                    else break;
                }
                shotCurrentWaypointIndex = wpIdx;
                shotPrevWaypoint = wpIdx == 0 ? pathStartPos : waypoints[wpIdx - 1];
                shotNextWaypoint = wpIdx < waypoints.Count ? waypoints[wpIdx] : waypoints[^1];
            }
            else
            {
                shotCurrentPathPos = Vector2.Zero;
                shotPrevWaypoint = Vector2.Zero;
                shotNextWaypoint = Vector2.Zero;
                shotCurrentWaypointIndex = -1;
            }

            isFiring = true;
            fireTime = 0f;
            skillshotLaunched = false;
            fireTargetStartPos = targetPos;
            
            // Use the prediction's behind-target aim position directly
            fireAimPos = new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y);
            
            skillshotDir = Vector2.Normalize(fireAimPos - casterPos);
            skillshotPos = casterPos;
            skillshotTrail.Clear();
            circularAnimRadius = 0f;
            hitResult = "";
            collisionDisplayTimer = 0f;
        }
        else
        {
            // Can't hit - stop continuous firing
            continuousFiring = false;
            isFiring = false;
        }
    }

    // Update firing animation
    if (isFiring)
    {
        fireTime += dt;

        // Calculate target position - use path if available, otherwise linear velocity
        Vector2 animTargetPos;
        if (activePath != null)
        {
            float pathTime = continuousFiring ? pathTotalTime + fireTime : fireTime;
            var pathPos = activePath.GetPositionAtTime(pathTime);
            animTargetPos = new Vector2((float)pathPos.X, (float)pathPos.Y);
        }
        else
        {
            animTargetPos = fireTargetStartPos + targetVelocity * targetSpeed * fireTime;
        }

        currentAnimTargetPos = animTargetPos;

        if (isCircularFiring)
        {
            // Circular spell animation - detonates at aim position after delay
            if (fireTime >= skillshotDelay && !skillshotLaunched)
            {
                skillshotLaunched = true;
                circularAnimRadius = 0f;
            }

            if (skillshotLaunched)
            {
                // Expand circle quickly to target radius
                float expansionSpeed = circularTargetRadius * 4f; // Expand over ~0.25s
                circularAnimRadius = Math.Min(circularTargetRadius, circularAnimRadius + expansionSpeed * dt);

                // Check collision - target center within spell radius + hitbox
                float collisionDist = circularTargetRadius + targetHitbox;
                float distToAim = Vector2.Distance(animTargetPos, fireAimPos);

                if (distToAim <= collisionDist)
                {
                    hitResult = "HIT!";
                    hitResultTimer = 1f;
                    hitCount++;
                    targetPos = animTargetPos;
                    collisionPos = fireAimPos;
                    collisionTime = fireTime;
                    collisionDisplayTimer = 2f;

                    if (continuousFiring)
                    {
                        pathTotalTime += fireTime;
                        StartNewShot();
                    }
                    else
                    {
                        isFiring = false;
                    }
                }
                // Spell completes but missed (after full expansion + small buffer)
                else if (circularAnimRadius >= circularTargetRadius)
                {
                    hitResult = "MISS";
                    hitResultTimer = 1f;
                    missCount++;
                    targetPos = animTargetPos;

                    if (continuousFiring)
                    {
                        pathTotalTime += fireTime;
                        StartNewShot();
                    }
                    else
                    {
                        isFiring = false;
                    }
                }
            }
        }
        else
        {
            // Linear skillshot animation
            if (fireTime >= skillshotDelay && !skillshotLaunched)
            {
                skillshotLaunched = true;
                skillshotPos = casterPos;
            }

            if (skillshotLaunched)
            {
                skillshotPos += skillshotDir * skillshotSpeed * dt;

                if (skillshotTrail.Count == 0 || Vector2.Distance(skillshotTrail[^1], skillshotPos) > 10)
                    skillshotTrail.Add(skillshotPos);
                if (skillshotTrail.Count > 30) skillshotTrail.RemoveAt(0);

                float travelDist = Vector2.Distance(casterPos, skillshotPos);

                // Check collision
                float collisionDist = (skillshotWidth / 2) + targetHitbox;
                if (Vector2.Distance(skillshotPos, animTargetPos) <= collisionDist)
                {
                    hitResult = "HIT!";
                    hitResultTimer = 1f;
                    hitCount++;
                    targetPos = animTargetPos;
                    collisionPos = (skillshotPos + animTargetPos) / 2;
                    collisionTime = fireTime;
                    collisionDisplayTimer = 2f;

                    if (continuousFiring)
                    {
                        pathTotalTime += fireTime;
                        StartNewShot();
                    }
                    else
                    {
                        isFiring = false;
                    }
                }
                else if (travelDist > skillshotRange)
                {
                    hitResult = "MISS";
                    hitResultTimer = 1f;
                    missCount++;
                    targetPos = animTargetPos;


                    if (continuousFiring)
                    {
                        pathTotalTime += fireTime;
                        StartNewShot();
                    }
                    else
                    {
                        isFiring = false;
                    }
                }
            }
        }

        if (fireTime > 5f)
        {
            hitResult = "TIMEOUT";
            hitResultTimer = 1f;
            missCount++;

            if (continuousFiring)
            {
                pathTotalTime += fireTime;
                StartNewShot();
            }
            else
            {
                isFiring = false;
            }
        }
    }

    if (hitResultTimer > 0) hitResultTimer -= rawDt;
    if (collisionDisplayTimer > 0) collisionDisplayTimer -= rawDt;

    // Drawing
    Raylib.BeginDrawing();
    Raylib.ClearBackground(new Color(15, 15, 25, 255));

    Raylib.BeginMode2D(camera);

    DrawGrid(camera);
    DrawRangeCircle(casterPos, skillshotRange);

    // Draw prediction visualization
    if (!isFiring)
    {
        // First draw trailing edge prediction (blue) - so it appears behind
        // Only show in linear mode with path (where blending is active)
        if (trailingEdgeResult is PredictionResult.Hit trailingHit && !isCircularMode && pathModeActive)
        {
            var trailingAim = new Vector2((float)trailingHit.CastPosition.X, (float)trailingHit.CastPosition.Y);
            
            // Blue trailing edge aim point - smaller radius to not overlap with green
            Raylib.DrawCircleV(trailingAim, 6, new Color(80, 140, 255, 200));
            Raylib.DrawCircleLinesV(trailingAim, 8, new Color(100, 180, 255, 255));
            // Line from target to trailing aim
            Raylib.DrawLineEx(targetPos, trailingAim, 1.5f, new Color(80, 140, 255, 150));
        }

        // Then draw current prediction (green) - on top
        if (ultimateResult is PredictionResult.Hit ultimateHit2)
        {
            var ultimateAim = new Vector2((float)ultimateHit2.CastPosition.X, (float)ultimateHit2.CastPosition.Y);
            var ultimatePredicted = new Vector2((float)ultimateHit2.PredictedTargetPosition.X, (float)ultimateHit2.PredictedTargetPosition.Y);

            if (isCircularMode)
            {
                // Circular spell preview - show radius at aim position
                Raylib.DrawCircleV(ultimateAim, skillshotRadius, new Color(100, 200, 255, 40));
                Raylib.DrawCircleLinesV(ultimateAim, skillshotRadius, new Color(100, 200, 255, 150));
                // Show predicted target position
                Raylib.DrawCircleV(ultimatePredicted, targetHitbox * 0.3f, new Color(255, 215, 0, 100));
                Raylib.DrawCircleLinesV(ultimatePredicted, targetHitbox * 0.5f, new Color(255, 215, 0, 200));
                // Aim point
                Raylib.DrawCircleV(ultimateAim, 12, new Color(100, 230, 255, 220));
                Raylib.DrawCircleLinesV(ultimateAim, 14, Color.White);
                // Line from target to aim
                Raylib.DrawLineEx(targetPos, ultimateAim, 2, new Color(100, 200, 255, 150));
            }
            else
            {
                // Linear skillshot preview - green for blended prediction
                DrawSkillshotPreview(casterPos, ultimateAim, skillshotWidth, new Color(80, 200, 80, 40));
                Raylib.DrawCircleV(ultimatePredicted, targetHitbox * 0.3f, new Color(80, 255, 80, 100));
                Raylib.DrawCircleLinesV(ultimatePredicted, targetHitbox * 0.5f, new Color(80, 255, 80, 200));
                // Green aim point
                Raylib.DrawCircleV(ultimateAim, 12, new Color(80, 255, 120, 220));
                Raylib.DrawCircleLinesV(ultimateAim, 14, Color.White);
                Raylib.DrawLineEx(targetPos, ultimateAim, 2, new Color(80, 255, 80, 150));
            }
        }
    }

    // Draw collision circle and info
    if (collisionDisplayTimer > 0)
    {
        float alpha = Math.Min(1f, collisionDisplayTimer / 2f);
        Raylib.DrawCircleLinesV(collisionPos, 80, new Color((byte)255, (byte)255, (byte)0, (byte)(200 * alpha)));
        Raylib.DrawCircleLinesV(collisionPos, 60, new Color((byte)255, (byte)200, (byte)0, (byte)(150 * alpha)));
        Raylib.DrawCircleV(collisionPos, 8, new Color((byte)255, (byte)255, (byte)100, (byte)(255 * alpha)));
    }

    // Draw firing animation
    if (isFiring)
    {
        // Animated target (use the stored position from update loop)
        Raylib.DrawCircleV(currentAnimTargetPos, targetHitbox, new Color(255, 80, 80, 200));
        Raylib.DrawCircleLinesV(currentAnimTargetPos, targetHitbox, Color.White);

        if (isCircularFiring)
        {
            // Circular spell animation
            if (skillshotLaunched)
            {
                // Draw expanding circle at aim position
                Raylib.DrawCircleV(fireAimPos, circularAnimRadius, new Color(100, 200, 255, 150));
                Raylib.DrawCircleLinesV(fireAimPos, circularAnimRadius, new Color(150, 230, 255, 255));
                // Draw target radius outline
                Raylib.DrawCircleLinesV(fireAimPos, circularTargetRadius, new Color(100, 200, 255, 80));
            }
            else
            {
                // Charging animation at aim position
                float delayProgress = fireTime / skillshotDelay;
                float chargeRadius = circularTargetRadius * delayProgress * 0.5f;
                Raylib.DrawCircleV(fireAimPos, chargeRadius, new Color((byte)100, (byte)200, (byte)255, (byte)(100 * delayProgress)));
                Raylib.DrawCircleLinesV(fireAimPos, circularTargetRadius, new Color(100, 200, 255, 100));
            }
            // Aim point marker
            Raylib.DrawCircleV(fireAimPos, 8, new Color(100, 230, 255, 200));
        }
        else
        {
            // Linear skillshot animation
            // Skillshot trail
            for (int i = 0; i < skillshotTrail.Count - 1; i++)
            {
                float alpha = (float)i / skillshotTrail.Count;
                float trailLength = 20 * alpha;
                DrawLinearSkillshot(skillshotTrail[i], skillshotDir, skillshotWidth * alpha, trailLength,
                    new Color((byte)255, (byte)215, (byte)0, (byte)(alpha * 100)));
            }

            // Skillshot
            if (skillshotLaunched)
            {
                float spearLength = 80f;
                DrawLinearSkillshot(skillshotPos, skillshotDir, skillshotWidth, spearLength,
                    new Color(255, 215, 0, 220));
                DrawLinearSkillshotOutline(skillshotPos, skillshotDir, skillshotWidth, spearLength, Color.White);
            }
            else
            {
                float delayProgress = fireTime / skillshotDelay;
                Raylib.DrawCircleV(casterPos, 35, new Color((byte)255, (byte)255, (byte)100, (byte)(150 * delayProgress)));
            }

            Raylib.DrawCircleLinesV(fireAimPos, 10, new Color(0, 255, 100, 150));
        }
    }

    // Draw caster
    if (!isFiring)
    {
        Raylib.DrawCircleV(casterPos, 30, new Color(255, 215, 0, 255));
        Raylib.DrawCircleLinesV(casterPos, 32, Color.White);
    }
    else
    {
        Raylib.DrawCircleV(casterPos, 30, new Color(200, 170, 0, 150));
        Raylib.DrawCircleLinesV(casterPos, 32, new Color(255, 255, 255, 100));
    }

    // Draw target
    if (!isFiring)
    {
        // Draw path and waypoints if in path mode
        if (pathModeActive && waypoints.Count > 0)
        {
            // Draw path lines
            var prevPos = targetPos;
            foreach (var wp in waypoints)
            {
                Raylib.DrawLineEx(prevPos, wp, 3, new Color(100, 200, 255, 150));
                prevPos = wp;
            }

            // Draw waypoint markers
            for (int i = 0; i < waypoints.Count; i++)
            {
                var wp = waypoints[i];
                Raylib.DrawCircleV(wp, 12, new Color(100, 200, 255, 200));
                Raylib.DrawCircleLinesV(wp, 14, Color.White);
                // Draw waypoint number
                Raylib.DrawText($"{i + 1}", (int)wp.X - 4, (int)wp.Y - 6, 14, Color.White);
            }
        }

        Raylib.DrawCircleV(targetPos, targetHitbox, new Color(255, 80, 80, 200));
        Raylib.DrawCircleLinesV(targetPos, targetHitbox, Color.White);

        // Draw velocity arrow only in non-path mode
        if (!pathModeActive && targetVelocity.LengthSquared() > 0)
        {
            var velEnd = targetPos + Vector2.Normalize(targetVelocity) * 100;
            DrawArrow(targetPos, velEnd, new Color(255, 200, 100, 255));
        }
        // In path mode, draw arrow toward first waypoint
        else if (pathModeActive && waypoints.Count > 0)
        {
            var dirToWp = Vector2.Normalize(waypoints[0] - targetPos);
            var arrowEnd = targetPos + dirToWp * 80;
            DrawArrow(targetPos, arrowEnd, new Color(100, 200, 255, 255));
        }
    }

    Raylib.EndMode2D();

    // Hit result (screen space)
    if (hitResultTimer > 0)
    {
        var resultColor = hitResult.Contains("HIT") ? Color.Green : Color.Red;
        int hitFontSize = 48;
        int textWidth = Raylib.MeasureText(hitResult, hitFontSize);
        Raylib.DrawText(hitResult, ScreenWidth / 2 - textWidth / 2, ScreenHeight / 2 - 24, hitFontSize, resultColor);
    }

    // UI (screen space)
    string currentPresetName = isCircularMode
        ? circularPresets[currentCircularPreset].Name
        : linearPresets[currentLinearPreset].Name;
    DrawUI(font, ultimateResult, targetSpeed, skillshotSpeed, skillshotRange,
           skillshotRadius, skillshotDelay, isCircularMode,
           currentPresetName, hitCount, missCount, camera.Zoom,
           timeLabels, currentTimeIndex, pathModeActive, waypoints.Count);

    // Collision info overlay
    if (collisionDisplayTimer > 0)
    {
        int infoY = ScreenHeight / 2 + 40;
        Raylib.DrawRectangle(ScreenWidth / 2 - 140, infoY, 280, 60, new Color(0, 0, 0, 200));
        Raylib.DrawRectangleLines(ScreenWidth / 2 - 140, infoY, 280, 60, Color.Yellow);
        Raylib.DrawText($"Time: {collisionTime:F3}s", ScreenWidth / 2 - 120, infoY + 10, 18, Color.Yellow);
        Raylib.DrawText($"Pos: ({collisionPos.X:F0}, {collisionPos.Y:F0})", ScreenWidth / 2 - 120, infoY + 32, 18, Color.LightGray);
    }

    Raylib.EndDrawing();
}

Raylib.CloseWindow();

static void HandleInput(ref Vector2 casterPos, ref Vector2 targetPos, ref Vector2 targetVelocity,
                       ref float targetSpeed, ref float skillshotSpeed, ref float skillshotRange,
                       ref float skillshotWidth, ref float skillshotDelay, ref float skillshotRadius,
                       (string Name, Vector2 Vel, float Speed, float Range, float Width, float Delay)[] linearPresets,
                       (string Name, float Radius, float Range, float Delay)[] circularPresets,
                       ref int currentLinearPreset, ref int currentCircularPreset, ref bool isCircularMode,
                       bool isFiring, Vector2 mouseWorld, List<Vector2> waypoints, ref bool pathModeActive,
                       ref Vector2 pathStartPos, ref float pathElapsedTime)
{
    if (isFiring) return;

    bool shiftHeld = Raylib.IsKeyDown(KeyboardKey.LeftShift) || Raylib.IsKeyDown(KeyboardKey.RightShift);

    // Shift+RMB adds waypoint (path mode)
    if (Raylib.IsMouseButtonPressed(MouseButton.Right) && shiftHeld)
    {
        // First waypoint - record starting position
        if (waypoints.Count == 0)
        {
            pathStartPos = targetPos;
            pathElapsedTime = 0f;
        }
        waypoints.Add(mouseWorld);
        pathModeActive = true;
    }
    // Regular RMB (without shift) moves target and clears path mode
    else if (Raylib.IsMouseButtonDown(MouseButton.Right) && !shiftHeld)
    {
        targetPos = mouseWorld;
        // Clear waypoints when moving target directly
        if (waypoints.Count > 0)
        {
            waypoints.Clear();
            pathModeActive = false;
            pathElapsedTime = 0f;
        }
    }

    if (Raylib.IsMouseButtonDown(MouseButton.Left))
        casterPos = mouseWorld;

    // Delete clears all waypoints
    if (Raylib.IsKeyPressed(KeyboardKey.Delete) || Raylib.IsKeyPressed(KeyboardKey.Backspace))
    {
        waypoints.Clear();
        pathModeActive = false;
        pathElapsedTime = 0f;
    }

    // Arrow keys set velocity direction (only used in non-path mode)
    if (Raylib.IsKeyPressed(KeyboardKey.Up)) targetVelocity = new Vector2(0, -1);
    if (Raylib.IsKeyPressed(KeyboardKey.Down)) targetVelocity = new Vector2(0, 1);
    if (Raylib.IsKeyPressed(KeyboardKey.Left)) targetVelocity = new Vector2(-1, 0);
    if (Raylib.IsKeyPressed(KeyboardKey.Right)) targetVelocity = new Vector2(1, 0);

    if (Raylib.IsKeyDown(KeyboardKey.Up) && Raylib.IsKeyDown(KeyboardKey.Right))
        targetVelocity = Vector2.Normalize(new Vector2(1, -1));
    if (Raylib.IsKeyDown(KeyboardKey.Up) && Raylib.IsKeyDown(KeyboardKey.Left))
        targetVelocity = Vector2.Normalize(new Vector2(-1, -1));
    if (Raylib.IsKeyDown(KeyboardKey.Down) && Raylib.IsKeyDown(KeyboardKey.Right))
        targetVelocity = Vector2.Normalize(new Vector2(1, 1));
    if (Raylib.IsKeyDown(KeyboardKey.Down) && Raylib.IsKeyDown(KeyboardKey.Left))
        targetVelocity = Vector2.Normalize(new Vector2(-1, 1));

    if (Raylib.IsKeyPressed(KeyboardKey.Q)) targetSpeed = Math.Max(50, targetSpeed - 50);
    if (Raylib.IsKeyPressed(KeyboardKey.W)) targetSpeed = Math.Min(800, targetSpeed + 50);
    if (Raylib.IsKeyPressed(KeyboardKey.E)) skillshotSpeed = Math.Max(500, skillshotSpeed - 100);
    if (Raylib.IsKeyPressed(KeyboardKey.R)) skillshotSpeed = Math.Min(3500, skillshotSpeed + 100);

    // TAB cycles through presets within current mode
    if (Raylib.IsKeyPressed(KeyboardKey.Tab))
    {
        if (isCircularMode)
        {
            currentCircularPreset = (currentCircularPreset + 1) % circularPresets.Length;
            var preset = circularPresets[currentCircularPreset];
            skillshotRadius = preset.Radius;
            skillshotRange = preset.Range;
            skillshotDelay = preset.Delay;
        }
        else
        {
            currentLinearPreset = (currentLinearPreset + 1) % linearPresets.Length;
            var preset = linearPresets[currentLinearPreset];
            targetVelocity = preset.Vel;
            skillshotSpeed = preset.Speed;
            skillshotRange = preset.Range;
            skillshotWidth = preset.Width;
            skillshotDelay = preset.Delay;
        }
    }

    // Grave/tilde toggles between circular and linear mode
    if (Raylib.IsKeyPressed(KeyboardKey.Grave))
    {
        isCircularMode = !isCircularMode;
        // Apply current preset for the new mode
        if (isCircularMode)
        {
            var preset = circularPresets[currentCircularPreset];
            skillshotRadius = preset.Radius;
            skillshotRange = preset.Range;
            skillshotDelay = preset.Delay;
        }
        else
        {
            var preset = linearPresets[currentLinearPreset];
            skillshotSpeed = preset.Speed;
            skillshotRange = preset.Range;
            skillshotWidth = preset.Width;
            skillshotDelay = preset.Delay;
        }
    }
}

static void DrawGrid(Camera2D camera)
{
    int gridSize = 100;
    int extent = 3000;
    for (int x = -extent; x <= extent; x += gridSize)
        Raylib.DrawLine(x, -extent, x, extent, new Color(35, 35, 45, 255));
    for (int y = -extent; y <= extent; y += gridSize)
        Raylib.DrawLine(-extent, y, extent, y, new Color(35, 35, 45, 255));
}

static void DrawRangeCircle(Vector2 center, float radius)
{
    Raylib.DrawCircleLinesV(center, radius, new Color(70, 70, 90, 255));
    Raylib.DrawCircleV(center, radius, new Color(40, 40, 60, 20));
}

static void DrawSkillshotPreview(Vector2 start, Vector2 end, float width, Color fillColor)
{
    var dir = Vector2.Normalize(end - start);
    var perp = new Vector2(-dir.Y, dir.X);
    var halfWidth = width / 2;

    var p1 = start + perp * halfWidth;
    var p2 = start - perp * halfWidth;
    var p3 = end - perp * halfWidth;
    var p4 = end + perp * halfWidth;

    Raylib.DrawTriangle(p1, p2, p3, fillColor);
    Raylib.DrawTriangle(p1, p3, p4, fillColor);
    var lineColor = new Color(fillColor.R, fillColor.G, fillColor.B, (byte)Math.Min(255, fillColor.A * 4));
    Raylib.DrawLineEx(p1, p4, 2, lineColor);
    Raylib.DrawLineEx(p2, p3, 2, lineColor);
}

static void DrawArrow(Vector2 start, Vector2 end, Color color)
{
    Raylib.DrawLineEx(start, end, 4, color);
    var dir = Vector2.Normalize(end - start);
    var perp = new Vector2(-dir.Y, dir.X);
    var p1 = end - dir * 15 + perp * 8;
    var p2 = end - dir * 15 - perp * 8;
    Raylib.DrawTriangle(end, p1, p2, color);
}

static void DrawLinearSkillshot(Vector2 center, Vector2 direction, float width, float length, Color color)
{
    var perp = new Vector2(-direction.Y, direction.X);
    var halfWidth = width / 2;
    var halfLength = length / 2;

    var front = center + direction * halfLength;
    var back = center - direction * halfLength;

    var p1 = front + perp * halfWidth;
    var p2 = front - perp * halfWidth;
    var p3 = back - perp * halfWidth;
    var p4 = back + perp * halfWidth;

    Raylib.DrawTriangle(p1, p3, p2, color);
    Raylib.DrawTriangle(p1, p4, p3, color);

    var tipLength = width * 0.8f;
    var tip = front + direction * tipLength;
    var tipLeft = front + perp * halfWidth * 0.3f;
    var tipRight = front - perp * halfWidth * 0.3f;
    Raylib.DrawTriangle(tip, tipRight, tipLeft, color);
}

static void DrawLinearSkillshotOutline(Vector2 center, Vector2 direction, float width, float length, Color color)
{
    var perp = new Vector2(-direction.Y, direction.X);
    var halfWidth = width / 2;
    var halfLength = length / 2;

    var front = center + direction * halfLength;
    var back = center - direction * halfLength;

    var p1 = front + perp * halfWidth;
    var p2 = front - perp * halfWidth;
    var p3 = back - perp * halfWidth;
    var p4 = back + perp * halfWidth;

    Raylib.DrawLineEx(p1, p2, 2, color);
    Raylib.DrawLineEx(p2, p3, 2, color);
    Raylib.DrawLineEx(p3, p4, 2, color);
    Raylib.DrawLineEx(p4, p1, 2, color);

    var tipLength = width * 0.8f;
    var tip = front + direction * tipLength;
    Raylib.DrawLineEx(p1, tip, 2, color);
    Raylib.DrawLineEx(p2, tip, 2, color);
}

static void DrawUI(Font font, PredictionResult? ultimateResult,
                  float targetSpeed, float skillshotSpeed, float skillshotRange,
                  float skillshotRadius, float skillshotDelay, bool isCircularMode,
                  string presetName, int hits, int misses, float zoom,
                  string[] timeLabels, int currentTimeIndex, bool pathModeActive, int waypointCount)
{
    int y = 20;
    int lineHeight = 26;
    int titleSize = 24;
    int fontSize = 18;
    int smallFont = 16;
    float spacing = 1f;

    // Main panel
    Raylib.DrawRectangle(10, 10, 420, 200, new Color(0, 0, 0, 200));
    Raylib.DrawRectangleLines(10, 10, 420, 200, new Color(255, 215, 0, 255));

    Raylib.DrawTextEx(font, "ULTIMATE", new Vector2(20, y), titleSize, spacing, new Color(255, 215, 0, 255));
    Raylib.DrawTextEx(font, "Behind-Edge Prediction", new Vector2(130, y + 4), fontSize, spacing, Color.White);
    y += 30;

    // Mode indicator
    var modeColor = isCircularMode ? new Color(100, 200, 255, 255) : new Color(255, 200, 100, 255);
    var modeText = isCircularMode ? "[CIRCULAR]" : "[LINEAR]";
    Raylib.DrawTextEx(font, $"{modeText} {presetName}", new Vector2(20, y), smallFont, spacing, modeColor);
    // Path mode indicator
    if (pathModeActive)
    {
        Raylib.DrawTextEx(font, $"  PATH ({waypointCount} pts)", new Vector2(280, y), smallFont, spacing, new Color(100, 200, 255, 255));
    }
    y += lineHeight;

    // Show appropriate stats based on mode
    if (isCircularMode)
    {
        Raylib.DrawTextEx(font, $"Target: {targetSpeed:F0}  Radius: {skillshotRadius:F0}  Delay: {skillshotDelay:F2}s", new Vector2(20, y), smallFont, spacing, Color.LightGray);
    }
    else
    {
        Raylib.DrawTextEx(font, $"Target: {targetSpeed:F0}  Projectile: {skillshotSpeed:F0}  Range: {skillshotRange:F0}", new Vector2(20, y), smallFont, spacing, Color.LightGray);
    }
    y += lineHeight;

    // Stats
    int total = hits + misses;
    float accuracy = total > 0 ? (float)hits / total * 100 : 0;
    var accColor = accuracy >= 80 ? Color.Green : accuracy >= 50 ? Color.Yellow : Color.Orange;
    Raylib.DrawTextEx(font, $"Hits: {hits}  Miss: {misses}  ({accuracy:F0}%)", new Vector2(20, y), smallFont, spacing, accColor);
    y += lineHeight + 6;

    // Prediction result
    if (ultimateResult is PredictionResult.Hit hit)
    {
        Raylib.DrawTextEx(font, $"Intercept: {hit.InterceptTime:F3}s", new Vector2(20, y), fontSize, spacing, new Color(255, 215, 0, 255));
        y += lineHeight;
        Raylib.DrawTextEx(font, $"Aim: ({hit.CastPosition.X:F0}, {hit.CastPosition.Y:F0})  Conf: {hit.Confidence:P0}", new Vector2(20, y), smallFont, spacing, Color.LightGray);
    }
    else if (ultimateResult is PredictionResult.OutOfRange oor)
    {
        Raylib.DrawTextEx(font, $"OUT OF RANGE ({oor.Distance:F0} > {oor.MaxRange:F0})", new Vector2(20, y), fontSize, spacing, Color.Red);
    }
    else
    {
        Raylib.DrawTextEx(font, "UNREACHABLE", new Vector2(20, y), fontSize, spacing, Color.Red);
    }

    // Fire panel (right side)
    int rx = ScreenWidth - 200;
    Raylib.DrawRectangle(rx - 10, 10, 200, 120, new Color(0, 0, 0, 200));
    Raylib.DrawRectangleLines(rx - 10, 10, 200, 120, new Color(255, 215, 0, 255));
    Raylib.DrawTextEx(font, "FIRE", new Vector2(rx, 18), titleSize, spacing, new Color(255, 215, 0, 255));
    Raylib.DrawTextEx(font, "Press SPACE to fire", new Vector2(rx, 50), smallFont, spacing, Color.White);
    Raylib.DrawTextEx(font, $"Zoom: {zoom:F1}x", new Vector2(rx, 80), smallFont, spacing, Color.LightGray);

    // Time multiplier buttons
    int btnY = ScreenHeight - 70;
    int btnX = 20;
    int btnW = 60;
    int btnH = 36;

    Raylib.DrawRectangle(10, btnY - 28, 350, 85, new Color(0, 0, 0, 200));
    Raylib.DrawRectangleLines(10, btnY - 28, 350, 85, new Color(80, 80, 100, 255));
    Raylib.DrawTextEx(font, "TIME (1-5)", new Vector2(20, btnY - 22), smallFont, spacing, Color.White);

    for (int i = 0; i < timeLabels.Length; i++)
    {
        var btnRect = new Rectangle(btnX + i * (btnW + 6), btnY, btnW, btnH);
        bool isSelected = i == currentTimeIndex;
        bool isHovered = Raylib.CheckCollisionPointRec(Raylib.GetMousePosition(), btnRect);

        Color btnColor = isSelected ? new Color(255, 215, 0, 255) :
                        isHovered ? new Color(80, 80, 100, 255) : new Color(50, 50, 60, 255);
        Raylib.DrawRectangleRec(btnRect, btnColor);
        Raylib.DrawRectangleLinesEx(btnRect, 2, isSelected ? Color.White : new Color(100, 100, 120, 255));

        var textSize = Raylib.MeasureTextEx(font, timeLabels[i], fontSize, spacing);
        Raylib.DrawTextEx(font, timeLabels[i], new Vector2(btnX + i * (btnW + 6) + btnW / 2 - textSize.X / 2, btnY + btnH / 2 - textSize.Y / 2), fontSize, spacing,
            isSelected ? Color.Black : Color.LightGray);
    }

    // Controls panel
    int ctrlY = 220;
    Raylib.DrawRectangle(10, ctrlY, 420, 155, new Color(0, 0, 0, 200));
    Raylib.DrawRectangleLines(10, ctrlY, 420, 155, new Color(80, 80, 100, 255));

    Raylib.DrawTextEx(font, "CONTROLS", new Vector2(20, ctrlY + 8), fontSize, spacing, Color.White);
    ctrlY += 28;
    Raylib.DrawTextEx(font, "LMB/RMB : Move caster/target", new Vector2(20, ctrlY), smallFont, spacing, Color.LightGray);
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "Shift+RMB : Add waypoint (path mode)", new Vector2(20, ctrlY), smallFont, spacing, new Color(100, 200, 255, 255));
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "DEL/Backspace : Clear waypoints", new Vector2(20, ctrlY), smallFont, spacing, new Color(100, 200, 255, 255));
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "TAB : Cycle presets  |  ` : Toggle mode", new Vector2(20, ctrlY), smallFont, spacing, Color.LightGray);
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "Q/W : Target speed  |  E/R : Projectile speed", new Vector2(20, ctrlY), smallFont, spacing, Color.LightGray);

    // Prediction legend (only show for linear mode with path)
    if (!isCircularMode && pathModeActive)
    {
        int legendY = ctrlY + 40;
        Raylib.DrawRectangle(10, legendY, 260, 60, new Color(0, 0, 0, 200));
        Raylib.DrawRectangleLines(10, legendY, 260, 60, new Color(80, 80, 100, 255));
        
        Raylib.DrawTextEx(font, "PREDICTION LEGEND", new Vector2(20, legendY + 8), fontSize, spacing, Color.White);
        legendY += 26;
        // Green circle and text
        Raylib.DrawCircleV(new Vector2(30, legendY + 8), 6, new Color(80, 255, 120, 220));
        Raylib.DrawTextEx(font, "Blended (path-aware)", new Vector2(45, legendY), smallFont, spacing, new Color(80, 255, 120, 255));
        // Blue circle and text
        Raylib.DrawCircleV(new Vector2(160, legendY + 8), 5, new Color(80, 140, 255, 220));
        Raylib.DrawTextEx(font, "Trailing", new Vector2(175, legendY), smallFont, spacing, new Color(100, 160, 255, 255));
    }
}
