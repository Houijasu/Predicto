using BenchmarkDotNet.Attributes;
using MathNet.Spatial.Euclidean;
using Predicto;
using Predicto.Models;

namespace Predicto.Benchmarks;

/// <summary>
/// Comprehensive benchmarks for the Predicto prediction engine.
/// </summary>
[MemoryDiagnoser]
[CategoriesColumn]
public class PredictionBenchmarks
{
    private Ultimate _prediction = null!;

    // Linear inputs
    private PredictionInput _stationaryInput;
    private PredictionInput _movingInput;
    private PredictionInput _movingFastInput;
    private PredictionInput _pathInput;
    private PredictionInput _minimizeTimeInput;
    private PredictionInput _hitscanInput;

    // Circular inputs
    private CircularPredictionInput _circularStationaryInput;
    private CircularPredictionInput _circularMovingInput;

    // Multi-target inputs
    private TargetCandidate[] _targets5 = null!;
    private TargetCandidate[] _targets10 = null!;
    private Point2D _casterPosition;
    private LinearSkillshot _rankSkillshot;

    [GlobalSetup]
    public void Setup()
    {
        _prediction = new Ultimate();
        _casterPosition = new Point2D(500, 500);

        // Linear: stationary target
        _stationaryInput = new PredictionInput(
            CasterPosition: new Point2D(200, 200),
            TargetPosition: new Point2D(700, 400),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: LinearSkillshot.BlitzcrankQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        // Linear: moving target
        _movingInput = new PredictionInput(
            CasterPosition: new Point2D(300, 400),
            TargetPosition: new Point2D(800, 400),
            TargetVelocity: new Vector2D(0, -350),
            Skillshot: LinearSkillshot.EzrealQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        // Linear: fast moving target
        _movingFastInput = new PredictionInput(
            CasterPosition: new Point2D(100, 100),
            TargetPosition: new Point2D(600, 500),
            TargetVelocity: new Vector2D(500, 300),
            Skillshot: LinearSkillshot.JinxW,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        // Linear: path-based
        var path = new TargetPath(
            waypoints: [new Point2D(600, 200), new Point2D(900, 600)],
            currentPosition: new Point2D(400, 400),
            currentWaypointIndex: 0,
            speed: 350);
        _pathInput = PredictionInput.WithPath(
            casterPosition: new Point2D(100, 300),
            path: path,
            skillshot: LinearSkillshot.MorganaQ);

        // Linear: minimize time
        _minimizeTimeInput = new PredictionInput(
            CasterPosition: new Point2D(200, 300),
            TargetPosition: new Point2D(800, 600),
            TargetVelocity: new Vector2D(-200, 250),
            Skillshot: LinearSkillshot.NidaleeQ,
            TargetHitboxRadius: Constants.DefaultHitboxRadius,
            MinimizeTime: true);

        // Linear: hitscan
        _hitscanInput = new PredictionInput(
            CasterPosition: new Point2D(300, 300),
            TargetPosition: new Point2D(700, 500),
            TargetVelocity: new Vector2D(250, -200),
            Skillshot: LinearSkillshot.LuxR,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        // Circular: stationary
        _circularStationaryInput = new CircularPredictionInput(
            CasterPosition: new Point2D(200, 200),
            TargetPosition: new Point2D(600, 400),
            TargetVelocity: new Vector2D(0, 0),
            Skillshot: CircularSkillshot.XerathW,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        // Circular: moving
        _circularMovingInput = new CircularPredictionInput(
            CasterPosition: new Point2D(300, 300),
            TargetPosition: new Point2D(700, 500),
            TargetVelocity: new Vector2D(-300, 200),
            Skillshot: CircularSkillshot.VeigarW,
            TargetHitboxRadius: Constants.DefaultHitboxRadius);

        // Multi-target: 5 targets
        _rankSkillshot = LinearSkillshot.BlitzcrankQ;
        _targets5 =
        [
            new TargetCandidate(new Point2D(700, 500), new Vector2D(0, -300), PriorityWeight: 2.0, Tag: "ADC"),
            new TargetCandidate(new Point2D(600, 400), new Vector2D(200, 0), PriorityWeight: 0.5, Tag: "Tank"),
            new TargetCandidate(new Point2D(800, 300), new Vector2D(-100, 200), PriorityWeight: 1.5, Tag: "Mage"),
            new TargetCandidate(new Point2D(650, 600), new Vector2D(150, -100), PriorityWeight: 1.0, Tag: "Fighter"),
            new TargetCandidate(new Point2D(750, 450), new Vector2D(-200, -250), PriorityWeight: 1.5, Tag: "Assassin"),
        ];

        // Multi-target: 10 targets
        _targets10 =
        [
            new TargetCandidate(new Point2D(700, 500), new Vector2D(0, -300), PriorityWeight: 2.0, Tag: "ADC1"),
            new TargetCandidate(new Point2D(600, 400), new Vector2D(200, 0), PriorityWeight: 0.5, Tag: "Tank1"),
            new TargetCandidate(new Point2D(800, 300), new Vector2D(-100, 200), PriorityWeight: 1.5, Tag: "Mage1"),
            new TargetCandidate(new Point2D(650, 600), new Vector2D(150, -100), PriorityWeight: 1.0, Tag: "Fighter1"),
            new TargetCandidate(new Point2D(750, 450), new Vector2D(-200, -250), PriorityWeight: 1.5, Tag: "Assassin1"),
            new TargetCandidate(new Point2D(550, 350), new Vector2D(300, 100), PriorityWeight: 2.0, Tag: "ADC2"),
            new TargetCandidate(new Point2D(850, 550), new Vector2D(-150, -200), PriorityWeight: 0.5, Tag: "Tank2"),
            new TargetCandidate(new Point2D(500, 500), new Vector2D(0, 350), PriorityWeight: 1.5, Tag: "Mage2"),
            new TargetCandidate(new Point2D(900, 400), new Vector2D(-250, 100), PriorityWeight: 1.0, Tag: "Fighter2"),
            new TargetCandidate(new Point2D(680, 350), new Vector2D(200, -300), PriorityWeight: 1.5, Tag: "Assassin2"),
        ];
    }

    // === Linear Prediction Benchmarks ===

    [Benchmark]
    [BenchmarkCategory("Linear")]
    public PredictionResult LinearPredict_Stationary() => _prediction.Predict(_stationaryInput);

    [Benchmark]
    [BenchmarkCategory("Linear")]
    public PredictionResult LinearPredict_Moving() => _prediction.Predict(_movingInput);

    [Benchmark]
    [BenchmarkCategory("Linear")]
    public PredictionResult LinearPredict_MovingFast() => _prediction.Predict(_movingFastInput);

    [Benchmark]
    [BenchmarkCategory("Linear")]
    public PredictionResult LinearPredict_WithPath() => _prediction.Predict(_pathInput);

    [Benchmark]
    [BenchmarkCategory("Linear")]
    public PredictionResult LinearPredict_MinimizeTime() => _prediction.Predict(_minimizeTimeInput);

    [Benchmark]
    [BenchmarkCategory("Linear")]
    public PredictionResult LinearPredict_Hitscan() => _prediction.Predict(_hitscanInput);

    // === Circular Prediction Benchmarks ===

    [Benchmark]
    [BenchmarkCategory("Circular")]
    public PredictionResult CircularPredict_Stationary() => _prediction.PredictCircular(_circularStationaryInput);

    [Benchmark]
    [BenchmarkCategory("Circular")]
    public PredictionResult CircularPredict_Moving() => _prediction.PredictCircular(_circularMovingInput);

    // === Multi-Target Benchmarks ===

    [Benchmark]
    [BenchmarkCategory("MultiTarget")]
    public IReadOnlyList<RankedTarget> RankTargets_5Targets() =>
        _prediction.RankTargets(_casterPosition, _rankSkillshot, _targets5);

    [Benchmark]
    [BenchmarkCategory("MultiTarget")]
    public IReadOnlyList<RankedTarget> RankTargets_10Targets() =>
        _prediction.RankTargets(_casterPosition, _rankSkillshot, _targets10);
}
