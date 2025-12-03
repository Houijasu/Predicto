using System.Numerics;
using MathNet.Spatial.Euclidean;
using Predicto;
using Predicto.Models;
using Raylib_cs;

const int ScreenWidth = 1600;
const int ScreenHeight = 1000;

Raylib.InitWindow(ScreenWidth, ScreenHeight, "Predicto - Ultimate Behind-Edge Prediction");
Raylib.SetTargetFPS(60);

// Load Arial font (fallback to default if not found)
Font arialFont;
try
{
    arialFont = Raylib.LoadFontEx("/usr/share/fonts/TTF/Arial.TTF", 32, null, 0);
    if (arialFont.BaseSize == 0)
        arialFont = Raylib.LoadFontEx("/usr/share/fonts/truetype/msttcorefonts/Arial.ttf", 32, null, 0);
    if (arialFont.BaseSize == 0)
        arialFont = Raylib.LoadFontEx("/usr/share/fonts/TTF/DejaVuSans.ttf", 32, null, 0);
}
catch
{
    arialFont = Raylib.GetFontDefault();
}
if (arialFont.BaseSize == 0)
    arialFont = Raylib.GetFontDefault();

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
// Nidalee Q defaults
float skillshotSpeed = 2000f;
float skillshotRange = 2000f;
float skillshotWidth = 40f;
float skillshotDelay = 0.25f;
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

// Presets
var presets = new (string Name, Vector2 Vel, float Speed, float Range, float Width, float Delay)[]
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
int currentPreset = 3; // Start with Nidalee Q

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
                ref skillshotSpeed, ref skillshotRange, ref skillshotWidth, ref skillshotDelay,
                presets, ref currentPreset, isFiring, mouseWorld, waypoints, ref pathModeActive,
                ref pathStartPos, ref pathElapsedTime);

    // Target stays stationary while drawing path - only moves when firing

    // Run prediction
    PredictionInput input;
    if (pathModeActive && waypoints.Count > 0)
    {
        // Build the path from the original start position
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
        
        // If we've passed all waypoints, target is stationary at last waypoint
        if (currentWaypointIndex >= waypointsList.Count)
        {
            // Target stopped at last waypoint - use zero velocity (stationary target)
            input = new PredictionInput(
                new Point2D(casterPos.X, casterPos.Y),
                new Point2D(targetPos.X, targetPos.Y),
                new Vector2D(0, 0),  // Stationary
                new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                targetHitbox);
        }
        else
        {
            // Still on path - create path from current position with remaining waypoints
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
        // Re-run prediction from current state
        PredictionInput shotInput;
        if (continuousFiring && waypoints.Count > 0)
        {
            // Build path from current path position
            var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
            var fullPath = new TargetPath(
                waypointsList,
                new Point2D(pathStartPos.X, pathStartPos.Y),
                0,
                targetSpeed);
            
            // Get current position on path
            var currentPathPos = fullPath.GetPositionAtTime(pathTotalTime);
            
            // Find remaining waypoints
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
                else
                {
                    break;
                }
            }
            
            // Check if path is complete
            if (currentWaypointIndex >= waypointsList.Count)
            {
                // Path complete - stop continuous firing
                continuousFiring = false;
                isFiring = false;
                return;
            }
            
            // Create path from current position with remaining waypoints
            var remainingWaypoints = waypointsList.Skip(currentWaypointIndex).ToList();
            var pathForPrediction = new TargetPath(
                remainingWaypoints,
                currentPathPos,
                0,
                targetSpeed);
            
            shotInput = PredictionInput.WithPath(
                new Point2D(casterPos.X, casterPos.Y),
                pathForPrediction,
                new LinearSkillshot(skillshotSpeed, skillshotRange, skillshotWidth, skillshotDelay),
                targetHitbox);
            
            // Store full path for animation (from original start)
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
        
        var shotResult = ultimate.Predict(shotInput);
        
        if (shotResult is PredictionResult.Hit hit)
        {
            // DEBUG: Behind-edge / trailing edge calculation verification
            float aimToTargetDist = Vector2.Distance(
                new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y),
                new Vector2((float)hit.PredictedTargetPosition.X, (float)hit.PredictedTargetPosition.Y));
            float effectiveRadius = targetHitbox + skillshotWidth / 2;
            float trailingEdgeDist = effectiveRadius - aimToTargetDist;  // How far inside the hitbox we're aiming
            float casterToAimDist = Vector2.Distance(casterPos, new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y));
            
            Console.WriteLine($"\n=== SHOT #{hitCount + missCount + 1} ===");
            Console.WriteLine($"Trailing edge distance: {trailingEdgeDist:F1} px (aim {aimToTargetDist:F1} from center, effective radius {effectiveRadius:F1})");
            Console.WriteLine($"Intercept: {hit.InterceptTime:F4}s | Aim dist: {casterToAimDist:F1} | Range: {skillshotRange:F1}");
            
            // Store debug info for potential miss logging
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
                
                // Find current segment
                double remainingDist = targetSpeed * pathTotalTime;
                var pos = new Point2D(pathStartPos.X, pathStartPos.Y);
                int wpIdx = 0;
                while (remainingDist > 0 && wpIdx < waypointsList.Count)
                {
                    var tgt = waypointsList[wpIdx];
                    var dist = (tgt - pos).Length;
                    if (dist <= remainingDist)
                    {
                        pos = tgt;
                        remainingDist -= dist;
                        wpIdx++;
                    }
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
            fireAimPos = new Vector2((float)hit.CastPosition.X, (float)hit.CastPosition.Y);
            skillshotDir = Vector2.Normalize(fireAimPos - casterPos);
            skillshotPos = casterPos;
            skillshotTrail.Clear();
            hitResult = "";
            collisionDisplayTimer = 0f;
        }
        else
        {
            // Can't hit - stop continuous firing
            Console.WriteLine($"\n=== PREDICTION FAILED - Stopping continuous fire ===");
            Console.WriteLine($"pathTotalTime: {pathTotalTime:F4}s");
            Console.WriteLine($"pathStartPos: ({pathStartPos.X:F1}, {pathStartPos.Y:F1})");
            Console.WriteLine($"Caster: ({casterPos.X:F1}, {casterPos.Y:F1})");
            if (shotResult is PredictionResult.OutOfRange oor)
            {
                Console.WriteLine($"Result: OutOfRange - Distance: {oor.Distance:F1}, MaxRange: {oor.MaxRange:F1}");
            }
            else if (shotResult is PredictionResult.Unreachable)
            {
                Console.WriteLine($"Result: Unreachable");
            }
            else
            {
                Console.WriteLine($"Result: {shotResult?.GetType().Name ?? "null"}");
            }
            if (continuousFiring && waypoints.Count > 0)
            {
                var waypointsList = waypoints.Select(wp => new Point2D(wp.X, wp.Y)).ToList();
                var fullPath = new TargetPath(waypointsList, new Point2D(pathStartPos.X, pathStartPos.Y), 0, targetSpeed);
                var currentPos = fullPath.GetPositionAtTime(pathTotalTime);
                Console.WriteLine($"Current path pos: ({currentPos.X:F1}, {currentPos.Y:F1})");
                Console.WriteLine($"Distance from caster to path pos: {Vector2.Distance(casterPos, new Vector2((float)currentPos.X, (float)currentPos.Y)):F1}");
                
                // Show remaining waypoints that were passed to prediction
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
                var remainingWaypoints = waypointsList.Skip(currentWaypointIndex).ToList();
                Console.WriteLine($"currentWaypointIndex: {currentWaypointIndex}");
                Console.WriteLine($"Remaining waypoints passed to prediction: {string.Join(" -> ", remainingWaypoints.Select(w => $"({w.X:F0},{w.Y:F0})"))}");
                Console.WriteLine($"Full waypoints: {string.Join(" -> ", waypoints.Select(w => $"({w.X:F0},{w.Y:F0})"))}");
            }
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
            // In continuous mode, use total path time; otherwise just fireTime
            float pathTime = continuousFiring ? pathTotalTime + fireTime : fireTime;
            var pathPos = activePath.GetPositionAtTime(pathTime);
            animTargetPos = new Vector2((float)pathPos.X, (float)pathPos.Y);
        }
        else
        {
            animTargetPos = fireTargetStartPos + targetVelocity * targetSpeed * fireTime;
        }
        
        // Store for drawing
        currentAnimTargetPos = animTargetPos;

        // Launch skillshot after delay
        if (fireTime >= skillshotDelay && !skillshotLaunched)
        {
            skillshotLaunched = true;
            skillshotPos = casterPos;
        }

        // Move skillshot
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
                
                // Update path time and continue if in continuous mode
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
            // Check range (only if no collision)
            else if (travelDist > skillshotRange)
            {
                hitResult = "MISS";
                hitResultTimer = 1f;
                missCount++;
                targetPos = animTargetPos;
                
                // Log debug info on miss
                Console.WriteLine($"\n=== MISS - SHOT #{currentShotNumber} ===");
                Console.WriteLine($"pathTotalTime: {shotPathTotalTime:F4}s");
                Console.WriteLine($"Caster: ({shotCasterPos.X:F1}, {shotCasterPos.Y:F1})");
                Console.WriteLine($"Target start (UI): ({shotTargetStartPos.X:F1}, {shotTargetStartPos.Y:F1})");
                Console.WriteLine($"Current path pos: ({shotCurrentPathPos.X:F1}, {shotCurrentPathPos.Y:F1})");
                Console.WriteLine($"Prev waypoint: ({shotPrevWaypoint.X:F1}, {shotPrevWaypoint.Y:F1})");
                Console.WriteLine($"Next waypoint [{shotCurrentWaypointIndex}]: ({shotNextWaypoint.X:F1}, {shotNextWaypoint.Y:F1})");
                Console.WriteLine($"CastPosition (aim): ({shotAimPos.X:F1}, {shotAimPos.Y:F1})");
                Console.WriteLine($"PredictedTargetPos: ({shotPredictedTargetPos.X:F1}, {shotPredictedTargetPos.Y:F1})");
                Console.WriteLine($"InterceptTime: {shotInterceptTime:F4}s");
                Console.WriteLine($"Confidence: {shotConfidence:F2}");
                Console.WriteLine($"Actual target pos at miss: ({animTargetPos.X:F1}, {animTargetPos.Y:F1})");
                Console.WriteLine($"Skillshot pos at miss: ({skillshotPos.X:F1}, {skillshotPos.Y:F1})");
                Console.WriteLine($"Travel distance: {travelDist:F1} (range: {skillshotRange:F1})");
                Console.WriteLine($"Distance to target: {Vector2.Distance(skillshotPos, animTargetPos):F1}");
                if (shotWaypoints.Count > 0)
                {
                    Console.WriteLine($"Waypoints: {string.Join(" -> ", shotWaypoints.Select(w => $"({w.X:F0},{w.Y:F0})"))}");
                }
                
                // Update path time and continue if in continuous mode
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

        if (fireTime > 5f)
        {
            hitResult = "TIMEOUT";
            hitResultTimer = 1f;
            missCount++;
            
            // Log debug info on timeout (also a miss)
            Console.WriteLine($"\n=== TIMEOUT - SHOT #{currentShotNumber} ===");
            Console.WriteLine($"pathTotalTime: {shotPathTotalTime:F4}s");
            Console.WriteLine($"Caster: ({shotCasterPos.X:F1}, {shotCasterPos.Y:F1})");
            Console.WriteLine($"Target start (UI): ({shotTargetStartPos.X:F1}, {shotTargetStartPos.Y:F1})");
            Console.WriteLine($"Current path pos: ({shotCurrentPathPos.X:F1}, {shotCurrentPathPos.Y:F1})");
            Console.WriteLine($"Prev waypoint: ({shotPrevWaypoint.X:F1}, {shotPrevWaypoint.Y:F1})");
            Console.WriteLine($"Next waypoint [{shotCurrentWaypointIndex}]: ({shotNextWaypoint.X:F1}, {shotNextWaypoint.Y:F1})");
            Console.WriteLine($"CastPosition (aim): ({shotAimPos.X:F1}, {shotAimPos.Y:F1})");
            Console.WriteLine($"PredictedTargetPos: ({shotPredictedTargetPos.X:F1}, {shotPredictedTargetPos.Y:F1})");
            Console.WriteLine($"InterceptTime: {shotInterceptTime:F4}s");
            Console.WriteLine($"Confidence: {shotConfidence:F2}");
            if (shotWaypoints.Count > 0)
            {
                Console.WriteLine($"Waypoints: {string.Join(" -> ", shotWaypoints.Select(w => $"({w.X:F0},{w.Y:F0})"))}");
            }
            
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
        // Ultimate prediction (gold)
        if (ultimateResult is PredictionResult.Hit ultimateHit2)
        {
            var ultimateAim = new Vector2((float)ultimateHit2.CastPosition.X, (float)ultimateHit2.CastPosition.Y);
            var ultimatePredicted = new Vector2((float)ultimateHit2.PredictedTargetPosition.X, (float)ultimateHit2.PredictedTargetPosition.Y);

            DrawSkillshotPreview(casterPos, ultimateAim, skillshotWidth, new Color(255, 215, 0, 40));
            Raylib.DrawCircleV(ultimatePredicted, targetHitbox * 0.3f, new Color(255, 215, 0, 100));
            Raylib.DrawCircleLinesV(ultimatePredicted, targetHitbox * 0.5f, new Color(255, 215, 0, 200));
            Raylib.DrawCircleV(ultimateAim, 12, new Color(255, 230, 100, 220));
            Raylib.DrawCircleLinesV(ultimateAim, 14, Color.White);
            Raylib.DrawLineEx(targetPos, ultimateAim, 2, new Color(255, 215, 0, 150));
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
    DrawUI(arialFont, ultimateResult, targetSpeed, skillshotSpeed, skillshotRange,
           presets[currentPreset].Name, hitCount, missCount, camera.Zoom,
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
                       ref float skillshotWidth, ref float skillshotDelay,
                       (string, Vector2, float, float, float, float)[] presets, ref int currentPreset,
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

    if (Raylib.IsKeyPressed(KeyboardKey.Tab))
    {
        currentPreset = (currentPreset + 1) % presets.Length;
        var preset = presets[currentPreset];
        targetVelocity = preset.Item2;
        skillshotSpeed = preset.Item3;
        skillshotRange = preset.Item4;
        skillshotWidth = preset.Item5;
        skillshotDelay = preset.Item6;
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

    Raylib.DrawTextEx(font, $"Preset: {presetName}", new Vector2(20, y), smallFont, spacing, Color.Yellow);
    // Path mode indicator
    if (pathModeActive)
    {
        Raylib.DrawTextEx(font, $"  PATH MODE ({waypointCount} pts)", new Vector2(200, y), smallFont, spacing, new Color(100, 200, 255, 255));
    }
    y += lineHeight;

    Raylib.DrawTextEx(font, $"Target: {targetSpeed:F0}  Projectile: {skillshotSpeed:F0}  Range: {skillshotRange:F0}", new Vector2(20, y), smallFont, spacing, Color.LightGray);
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
    Raylib.DrawRectangle(10, ctrlY, 420, 135, new Color(0, 0, 0, 200));
    Raylib.DrawRectangleLines(10, ctrlY, 420, 135, new Color(80, 80, 100, 255));

    Raylib.DrawTextEx(font, "CONTROLS", new Vector2(20, ctrlY + 8), fontSize, spacing, Color.White);
    ctrlY += 28;
    Raylib.DrawTextEx(font, "LMB/RMB : Move caster/target", new Vector2(20, ctrlY), smallFont, spacing, Color.LightGray);
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "Shift+RMB : Add waypoint (path mode)", new Vector2(20, ctrlY), smallFont, spacing, new Color(100, 200, 255, 255));
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "DEL/Backspace : Clear waypoints", new Vector2(20, ctrlY), smallFont, spacing, new Color(100, 200, 255, 255));
    ctrlY += lineHeight;
    Raylib.DrawTextEx(font, "TAB : Skills  |  Q/W E/R : Speed", new Vector2(20, ctrlY), smallFont, spacing, Color.LightGray);
}
