# Sensor Fusion Engine

`sensor-fusion-engine` is a dependency-free Node.js toolkit for deterministic sensor-data engineering. It connects calibrated three-axis measurements and timestamps to filtering, quaternion orientation, statistical fusion, inertial integration, stability metrics, health evidence, and uncertainty bounds.

It is designed for algorithm development and review, not as a certified navigation or safety controller. Production systems require sensor-specific calibration, frame conventions, timestamp integrity, observability analysis, numerical validation, fault containment, and representative motion/environment testing.

## Capabilities

- Apply three-axis bias and full 3×3 scale/misalignment calibration.
- Estimate stationary mean, sample noise, and bias against an expected vector.
- Run first-order low-pass and complementary angle filters.
- Run scalar recursive Kalman estimation with innovation evidence.
- Gate measurements by normalized innovation variance.
- Normalize/multiply quaternions and integrate three-axis gyroscope rotation.
- Convert quaternions to roll/pitch/yaw and derive tilt from acceleration.
- Calculate tilt-compensated magnetic heading with declination.
- Resample irregular scalar streams and estimate integer sample lag.
- Reject outliers with median absolute deviation.
- Fuse independent measurements by inverse variance.
- Calculate Allan deviation at selected cluster times.
- Integrate three-axis acceleration to velocity and position with bias removal.
- Detect timestamp gaps, out-of-range values, stuck data, and excessive noise.
- Run seeded heading uncertainty analysis.
- Generate concise Markdown evidence reports.

## Requirements

- Node.js 18 or newer

## Quick start

```bash
npm install
npm test
node src/cli.js stationary examples/stationary.json
node src/cli.js integrate-accel examples/acceleration.json
node src/cli.js health examples/health.json
node src/cli.js monte-carlo examples/magnetic.json --iterations 1000 --seed 42 --noise 0.05
node src/cli.js report examples/bundle.json --output report.md
```

## CLI

```text
sensor-fusion-engine calibrate raw.json bias.json matrix.json
sensor-fusion-engine stationary samples.json [expected.json]
sensor-fusion-engine complementary previousDeg gyroDegPerS accelDeg dt --tau seconds
sensor-fusion-engine kalman measurements.json --process variance --measurement variance
sensor-fusion-engine gate prediction predVariance measurement measurementVariance --sigma N
sensor-fusion-engine integrate-gyro quaternion.json rates.json dt
sensor-fusion-engine euler quaternion.json
sensor-fusion-engine tilt acceleration.json
sensor-fusion-engine heading magnetic.json rollDeg pitchDeg --declination deg
sensor-fusion-engine resample samples.json intervalSeconds
sensor-fusion-engine lag reference.json target.json maxLagSamples
sensor-fusion-engine outliers values.json --threshold N
sensor-fusion-engine weighted measurements.json
sensor-fusion-engine allan samples.json sampleIntervalSeconds clusterSizes.json
sensor-fusion-engine integrate-accel samples.json
sensor-fusion-engine health samples.json [options.json]
sensor-fusion-engine monte-carlo magnetic.json --iterations N --seed N --noise units
sensor-fusion-engine report bundle.json --output report.md
```

## Library API

```js
import {
  calibrateVector, stationaryStatistics, lowPass, complementaryAngle,
  scalarKalman, innovationGate, integrateGyroscope, quaternionToEuler,
  accelerometerTilt, tiltCompensatedHeading, resampleLinear,
  estimateSampleLag, madOutlierGate, varianceWeightedFusion,
  allanDeviation, integrateAcceleration, sensorHealth,
  monteCarloHeading, reportMarkdown
} from '@opencircuitlabs/sensor-fusion-engine';

const rotation = integrateGyroscope([1, 0, 0, 0], [0, 0, Math.PI / 2], 1);
console.log(quaternionToEuler(rotation.quaternion));
```

## Engineering assumptions

- Vectors must use one documented right-handed coordinate frame and consistent units.
- Gyroscope integration assumes piecewise-constant angular rate and does not estimate bias internally.
- Complementary and scalar Kalman filters are transparent primitives, not a complete attitude/navigation EKF.
- Magnetic heading is unreliable near ferromagnetic material, high current, or local field distortion.
- Double integration of acceleration drifts rapidly; practical inertial navigation needs gravity compensation, attitude coupling, constraints, and external observations.
- Allan deviation uses non-overlapping clusters and is intended for reproducible screening rather than replacing a complete IEEE stability characterization.
- Monte Carlo heading sampling uses bounded independent axis noise and does not model correlated calibration or environmental errors.

## Development

```bash
npm test
npm run check
```

CI runs all tests on Node.js 18, 20, and 22.

## License

MIT © OpenCircuitLabs contributors. See [LICENSE](LICENSE).
