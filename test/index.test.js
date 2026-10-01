import test from 'node:test';
import assert from 'node:assert/strict';
import {
  accelerometerTilt,allanDeviation,calibrateVector,complementaryAngle,
  estimateSampleLag,innovationGate,integrateAcceleration,integrateGyroscope,
  madOutlierGate,monteCarloHeading,multiplyQuaternions,normalizeQuaternion,
  quaternionToEuler,reportMarkdown,resampleLinear,scalarKalman,sensorHealth,
  stationaryStatistics,tiltCompensatedHeading,varianceWeightedFusion,lowPass
} from '../src/index.js';

test('applies bias and a full 3x3 calibration matrix',()=>{
  const result=calibrateVector([2,3,4],[1,1,1],[[2,0,0],[0,3,0],[0,0,4]]);
  assert.deepEqual(result.centered,[1,2,3]);
  assert.deepEqual(result.calibrated,[2,6,12]);
  assert.equal(result.magnitude,13.56466);
});

test('validates calibration dimensions',()=>{
  assert.throws(()=>calibrateVector([1,2],[0,0,0]),/three-axis/);
});

test('calculates stationary mean, noise, and bias',()=>{
  const result=stationaryStatistics([[1,2,3],[3,4,5]],[2,3,4]);
  assert.deepEqual(result.mean,[2,3,4]);
  assert.deepEqual(result.bias,[0,0,0]);
  assert.deepEqual(result.standardDeviation,[1.414214,1.414214,1.414214]);
});

test('performs a first-order low-pass update',()=>{
  const result=lowPass(0,10,.1,1);
  assert.equal(result.alpha,.38587);
  assert.equal(result.value,3.858695);
});

test('fuses gyro prediction and accelerometer angle',()=>{
  const result=complementaryAngle(10,2,20,.1,.5);
  assert.equal(result.alpha,.833333);
  assert.equal(result.gyroPredictionDeg,10.2);
  assert.equal(result.angleDeg,11.833333);
});

test('filters scalar measurements with a recursive Kalman estimator',()=>{
  const result=scalarKalman([10,11,9,10],{processVariance:.01,measurementVariance:.1,initialEstimate:0,initialVariance:1});
  assert.equal(result.rows.length,4);
  assert.ok(result.rows[0].kalmanGain>result.rows.at(-1).kalmanGain);
  assert.ok(result.finalEstimate>9&&result.finalEstimate<11);
  assert.ok(result.finalVariance>0);
});

test('validates Kalman variances',()=>{
  assert.throws(()=>scalarKalman([1],{measurementVariance:0}),/valid process/);
});

test('accepts and rejects normalized innovations',()=>{
  assert.equal(innovationGate(10,1,11,1,3).accepted,true);
  const rejected=innovationGate(10,.1,20,.1,3);
  assert.equal(rejected.accepted,false);
  assert.ok(rejected.normalizedInnovation>20);
});

test('normalizes and multiplies quaternions',()=>{
  assert.deepEqual(normalizeQuaternion([2,0,0,0]),[1,0,0,0]);
  assert.deepEqual(multiplyQuaternions([1,0,0,0],[0,1,0,0]),[0,1,0,0]);
  assert.throws(()=>normalizeQuaternion([0,0,0,0]),/positive quaternion norm/);
});

test('integrates a 90-degree yaw rotation',()=>{
  const result=integrateGyroscope([1,0,0,0],[0,0,Math.PI/2],1);
  assert.equal(result.rotationAngleDeg,90);
  assert.ok(Math.abs(result.quaternion[0]-Math.SQRT1_2)<1e-10);
  assert.ok(Math.abs(result.quaternion[3]-Math.SQRT1_2)<1e-10);
  assert.equal(quaternionToEuler(result.quaternion).yawDeg,90);
});

test('preserves orientation for zero angular rate',()=>{
  assert.deepEqual(integrateGyroscope([2,0,0,0],[0,0,0],.1),{quaternion:[1,0,0,0],rotationAngleDeg:0});
});

test('derives roll and pitch from gravity',()=>{
  assert.deepEqual(accelerometerTilt([0,0,9.81]),{rollDeg:0,pitchDeg:0,magnitude:9.81});
  assert.equal(accelerometerTilt([0,9.81,0]).rollDeg,90);
});

test('calculates tilt-compensated magnetic heading and declination',()=>{
  assert.equal(tiltCompensatedHeading([1,-1,0],0,0).headingDeg,45);
  assert.equal(tiltCompensatedHeading([1,-1,0],0,0,5).headingDeg,50);
});

test('linearly resamples irregular timed data',()=>{
  const result=resampleLinear([{timeSeconds:0,value:0},{timeSeconds:1,value:10}],.25);
  assert.deepEqual(result.rows.map(row=>row.value),[0,2.5,5,7.5,10]);
  assert.deepEqual(result.rows.map(row=>row.timeSeconds),[0,.25,.5,.75,1]);
});

test('rejects duplicate resampling timestamps',()=>{
  assert.throws(()=>resampleLinear([{timeSeconds:0,value:0},{timeSeconds:0,value:1}],.1),/unique/);
});

test('estimates sample lag by overlap RMSE',()=>{
  const result=estimateSampleLag([0,1,2,3,4],[9,0,1,2,3,4],2);
  assert.equal(result.lagSamples,1);
  assert.equal(result.rmse,0);
  assert.equal(result.overlapSamples,5);
});

test('rejects robust outliers using median absolute deviation',()=>{
  const result=madOutlierGate([1,1,1,10]);
  assert.deepEqual(result.accepted,[1,1,1]);
  assert.deepEqual(result.rejected,[10]);
  assert.equal(result.rows[3].score,Infinity);
});

test('fuses measurements with inverse-variance weighting',()=>{
  const result=varianceWeightedFusion([{value:10,variance:1},{value:12,variance:3}]);
  assert.equal(result.value,10.5);
  assert.equal(result.variance,.75);
  assert.deepEqual(result.weights,[.75,.25]);
});

test('computes Allan deviation over non-overlapping clusters',()=>{
  const result=allanDeviation([0,0,1,1,2,2,3,3],1,[1,2]);
  assert.equal(result.rows[1].tauSeconds,2);
  assert.equal(result.rows[1].allanDeviation,.707107);
});

test('validates Allan cluster sizes',()=>{
  assert.throws(()=>allanDeviation([1,2,3,4],1,[3]),/valid integer cluster/);
});

test('integrates constant acceleration into velocity and position',()=>{
  const result=integrateAcceleration([{timeSeconds:0,accelerationMps2:[1,0,0]},{timeSeconds:1,accelerationMps2:[1,0,0]},{timeSeconds:2,accelerationMps2:[1,0,0]}]);
  assert.deepEqual(result.finalVelocityMps,[2,0,0]);
  assert.deepEqual(result.finalPositionM,[2,0,0]);
});

test('removes declared acceleration bias during integration',()=>{
  const result=integrateAcceleration([{timeSeconds:0,accelerationMps2:[1,0,0]},{timeSeconds:1,accelerationMps2:[1,0,0]}],{accelerationBiasMps2:[1,0,0]});
  assert.deepEqual(result.finalVelocityMps,[0,0,0]);
  assert.deepEqual(result.finalPositionM,[0,0,0]);
});

test('detects timestamp gaps, range failures, stuck signal, and noise',()=>{
  const samples=[{timeSeconds:0,value:1},{timeSeconds:.1,value:1},{timeSeconds:.2,value:1},{timeSeconds:.5,value:10}];
  const result=sensorHealth(samples,{expectedIntervalSeconds:.1,range:[0,5],stuckSamples:3,maximumNoiseStd:2});
  assert.equal(result.healthy,false);
  for(const issue of ['timestamp-gaps','out-of-range','stuck-signal','excessive-noise'])assert.ok(result.issues.includes(issue));
});

test('accepts a healthy sampled sensor',()=>{
  const samples=[{timeSeconds:0,value:1},{timeSeconds:.1,value:1.1},{timeSeconds:.2,value:.9},{timeSeconds:.3,value:1.05}];
  assert.equal(sensorHealth(samples,{expectedIntervalSeconds:.1,range:[0,2],maximumNoiseStd:.2}).healthy,true);
});

test('runs deterministic magnetic-heading uncertainty analysis',()=>{
  const first=monteCarloHeading([1,-1,0],{iterations:100,seed:4,uniformNoise:.05}),second=monteCarloHeading([1,-1,0],{iterations:100,seed:4,uniformNoise:.05});
  assert.deepEqual(first,second);
  assert.ok(first.p05HeadingDeg<=first.p50HeadingDeg);
  assert.ok(first.p50HeadingDeg<=first.p95HeadingDeg);
  assert.throws(()=>monteCarloHeading([1,-1,0],{iterations:0}),/positive integer/);
});

test('renders a multi-domain sensor-fusion report',()=>{
  const report=reportMarkdown({calibration:{bias:[.1,0,0],standardDeviation:[.01,.01,.01]},orientation:{rollDeg:1,pitchDeg:2,yawDeg:3},health:{healthy:true,gapCount:0,outOfRangeCount:0},uncertainty:{p05HeadingDeg:2,p50HeadingDeg:3,p95HeadingDeg:4}});
  assert.match(report,/# Sensor fusion analysis/);
  assert.match(report,/Calibration bias: \[0.1, 0, 0\]/);
  assert.match(report,/yaw 3 deg/);
  assert.match(report,/Sensor health: healthy/);
  assert.match(report,/2 \/ 3 \/ 4 deg/);
});
