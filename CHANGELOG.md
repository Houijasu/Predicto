# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- BenchmarkDotNet performance benchmarks (`tests/Predicto.Benchmarks/`)
- GitHub Actions CI/CD pipeline with build, test, and format checks
- `CHANGELOG.md` and `CONTRIBUTING.md`

### Changed
- Split `InterceptSolver.cs` into partial class files for maintainability
- Split `Ultimate.cs` into partial class files for maintainability
- Applied `dotnet format` for consistent code style

### Removed
- Experimental `PredictReactive` feature (unused)
