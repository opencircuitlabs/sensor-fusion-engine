const round=(value,places=6)=>Number(value.toFixed(places));
const sum=values=>values.reduce((total,value)=>total+value,0);
const positive=(value,name)=>{if(!(Number.isFinite(value)&&value>0))throw new Error(`positive ${name} is required`)};
const vector3=value=>Array.isArray(value)&&value.length===3&&value.every(Number.isFinite);
const vectorRound=value=>value.map(item=>round(item));
const magnitude=value=>Math.hypot(...value);

export function calibrateVector(raw,bias=[0,0,0],matrix=[[1,0,0],[0,1,0],[0,0,1]]){
  if(!vector3(raw)||!vector3(bias)||!Array.isArray(matrix)||matrix.length!==3||matrix.some(row=>!vector3(row)))throw new Error('three-axis raw, bias, and matrix values are required');const centered=raw.map((value,index)=>value-bias[index]),calibrated=matrix.map(row=>sum(row.map((coefficient,index)=>coefficient*centered[index])));return{raw,centered:vectorRound(centered),calibrated:vectorRound(calibrated),magnitude:round(magnitude(calibrated))};
}

export function stationaryStatistics(samples,expectedVector=null){
  if(!Array.isArray(samples)||samples.length<2||samples.some(sample=>!vector3(sample)))throw new Error('at least two three-axis samples are required');const count=samples.length,mean=[0,1,2].map(axis=>sum(samples.map(sample=>sample[axis]))/count),std=[0,1,2].map(axis=>Math.sqrt(sum(samples.map(sample=>(sample[axis]-mean[axis])**2))/(count-1))),bias=expectedVector==null?mean:mean.map((value,index)=>value-expectedVector[index]);return{sampleCount:count,mean:vectorRound(mean),standardDeviation:vectorRound(std),bias:vectorRound(bias),meanMagnitude:round(magnitude(mean))};
}

export function lowPass(previous,current,deltaSeconds,cutoffHz){
  positive(deltaSeconds,'sample interval');positive(cutoffHz,'cutoff frequency');if(!Number.isFinite(previous)||!Number.isFinite(current))throw new Error('finite samples are required');const rc=1/(2*Math.PI*cutoffHz),alpha=deltaSeconds/(rc+deltaSeconds),value=previous+alpha*(current-previous);return{alpha:round(alpha),value:round(value)};
}

export function complementaryAngle(previousAngleDeg,gyroRateDegPerSecond,accelerometerAngleDeg,deltaSeconds,timeConstantSeconds=.5){
  positive(deltaSeconds,'sample interval');positive(timeConstantSeconds,'time constant');if(![previousAngleDeg,gyroRateDegPerSecond,accelerometerAngleDeg].every(Number.isFinite))throw new Error('finite angle inputs are required');const alpha=timeConstantSeconds/(timeConstantSeconds+deltaSeconds),gyroPrediction=previousAngleDeg+gyroRateDegPerSecond*deltaSeconds,value=alpha*gyroPrediction+(1-alpha)*accelerometerAngleDeg;return{alpha:round(alpha),gyroPredictionDeg:round(gyroPrediction),angleDeg:round(value)};
}

export function scalarKalman(measurements,options={}){
  if(!Array.isArray(measurements)||!measurements.length||measurements.some(value=>!Number.isFinite(value)))throw new Error('finite measurements are required');const processVariance=options.processVariance??.001,measurementVariance=options.measurementVariance??.1;if(!(processVariance>=0&&measurementVariance>0))throw new Error('valid process and measurement variance are required');let estimate=options.initialEstimate??measurements[0],variance=options.initialVariance??1;const rows=measurements.map((measurement,index)=>{variance+=processVariance;const gain=variance/(variance+measurementVariance),innovation=measurement-estimate;estimate+=gain*innovation;variance=(1-gain)*variance;return{index,measurement,innovation:round(innovation),kalmanGain:round(gain),estimate:round(estimate),variance:round(variance)}});return{rows,finalEstimate:round(estimate),finalVariance:round(variance)};
}

export function innovationGate(prediction,predictionVariance,measurement,measurementVariance,sigmaLimit=3){
  if(![prediction,measurement].every(Number.isFinite)||!(predictionVariance>=0&&measurementVariance>0&&sigmaLimit>0))throw new Error('valid innovation inputs are required');const innovation=measurement-prediction,variance=predictionVariance+measurementVariance,normalized=Math.abs(innovation)/Math.sqrt(variance);return{innovation:round(innovation),innovationVariance:round(variance),normalizedInnovation:round(normalized),sigmaLimit,accepted:normalized<=sigmaLimit};
}

export function normalizeQuaternion(quaternion){if(!Array.isArray(quaternion)||quaternion.length!==4||quaternion.some(value=>!Number.isFinite(value)))throw new Error('four finite quaternion elements are required');const norm=Math.hypot(...quaternion);positive(norm,'quaternion norm');return quaternion.map(value=>round(value/norm,12));}
export function multiplyQuaternions(a,b){if(!Array.isArray(a)||!Array.isArray(b)||a.length!==4||b.length!==4)throw new Error('two quaternions are required');const[w,x,y,z]=a,[v,i,j,k]=b;return[round(w*v-x*i-y*j-z*k,12),round(w*i+x*v+y*k-z*j,12),round(w*j-x*k+y*v+z*i,12),round(w*k+x*j-y*i+z*v,12)];}

export function integrateGyroscope(quaternion,angularRateRadPerSecond,deltaSeconds){
  positive(deltaSeconds,'sample interval');if(!vector3(angularRateRadPerSecond))throw new Error('three-axis angular rate is required');const rate=magnitude(angularRateRadPerSecond);if(rate===0)return{quaternion:normalizeQuaternion(quaternion),rotationAngleDeg:0};const angle=rate*deltaSeconds,scale=Math.sin(angle/2)/rate,delta=[Math.cos(angle/2),...angularRateRadPerSecond.map(value=>value*scale)],result=normalizeQuaternion(multiplyQuaternions(normalizeQuaternion(quaternion),delta));return{quaternion:result,rotationAngleDeg:round(angle*180/Math.PI)};
}

export function quaternionToEuler(quaternion){
  const[w,x,y,z]=normalizeQuaternion(quaternion),sinr=2*(w*x+y*z),cosr=1-2*(x*x+y*y),roll=Math.atan2(sinr,cosr),sinp=2*(w*y-z*x),pitch=Math.asin(Math.max(-1,Math.min(1,sinp))),siny=2*(w*z+x*y),cosy=1-2*(y*y+z*z),yaw=Math.atan2(siny,cosy);return{rollDeg:round(roll*180/Math.PI),pitchDeg:round(pitch*180/Math.PI),yawDeg:round(yaw*180/Math.PI)};
}

export function accelerometerTilt(acceleration){if(!vector3(acceleration)||magnitude(acceleration)===0)throw new Error('non-zero three-axis acceleration is required');const[x,y,z]=acceleration,roll=Math.atan2(y,z),pitch=Math.atan2(-x,Math.hypot(y,z));return{rollDeg:round(roll*180/Math.PI),pitchDeg:round(pitch*180/Math.PI),magnitude:round(magnitude(acceleration))};}

export function tiltCompensatedHeading(magnetic,rollDeg,pitchDeg,declinationDeg=0){
  if(!vector3(magnetic)||![rollDeg,pitchDeg,declinationDeg].every(Number.isFinite))throw new Error('valid magnetic vector and angles are required');const roll=rollDeg*Math.PI/180,pitch=pitchDeg*Math.PI/180,[x,y,z]=magnetic,xh=x*Math.cos(pitch)+z*Math.sin(pitch),yh=x*Math.sin(roll)*Math.sin(pitch)+y*Math.cos(roll)-z*Math.sin(roll)*Math.cos(pitch);let heading=Math.atan2(-yh,xh)*180/Math.PI+declinationDeg;heading=(heading%360+360)%360;return{horizontalX:round(xh),horizontalY:round(yh),headingDeg:round(heading)};
}

export function resampleLinear(samples,intervalSeconds){
  positive(intervalSeconds,'resample interval');if(!Array.isArray(samples)||samples.length<2||samples.some(sample=>!Number.isFinite(sample.timeSeconds)||!Number.isFinite(sample.value)))throw new Error('at least two timed scalar samples are required');const sorted=[...samples].sort((a,b)=>a.timeSeconds-b.timeSeconds);for(let i=1;i<sorted.length;i++)if(sorted[i].timeSeconds===sorted[i-1].timeSeconds)throw new Error('sample timestamps must be unique');const rows=[];let cursor=0;for(let time=sorted[0].timeSeconds;time<=sorted.at(-1).timeSeconds+1e-12;time+=intervalSeconds){while(cursor<sorted.length-2&&sorted[cursor+1].timeSeconds<time)cursor++;const left=sorted[cursor],right=sorted[Math.min(cursor+1,sorted.length-1)],fraction=right.timeSeconds===left.timeSeconds?0:(time-left.timeSeconds)/(right.timeSeconds-left.timeSeconds),value=left.value+fraction*(right.value-left.value);rows.push({timeSeconds:round(time),value:round(value)})}return{intervalSeconds,rows};
}

export function estimateSampleLag(reference,target,maximumLagSamples){
  if(!Array.isArray(reference)||!Array.isArray(target)||reference.length<3||target.length<3||reference.some(value=>!Number.isFinite(value))||target.some(value=>!Number.isFinite(value)))throw new Error('finite reference and target series are required');if(!Number.isInteger(maximumLagSamples)||maximumLagSamples<0)throw new Error('non-negative integer maximum lag is required');let best=null;for(let lag=-maximumLagSamples;lag<=maximumLagSamples;lag++){let squared=0,count=0;for(let i=0;i<reference.length;i++){const j=i+lag;if(j>=0&&j<target.length){squared+=(reference[i]-target[j])**2;count++}}const rmse=Math.sqrt(squared/count);if(best==null||rmse<best.rmse)best={lagSamples:lag,rmse,count}}return{lagSamples:best.lagSamples,rmse:round(best.rmse),overlapSamples:best.count};
}

const median=values=>{const sorted=[...values].sort((a,b)=>a-b),middle=Math.floor(sorted.length/2);return sorted.length%2?sorted[middle]:(sorted[middle-1]+sorted[middle])/2};
export function madOutlierGate(values,threshold=3.5){if(!Array.isArray(values)||values.length<3||values.some(value=>!Number.isFinite(value))||!(threshold>0))throw new Error('finite values and positive threshold are required');const center=median(values),mad=median(values.map(value=>Math.abs(value-center))),scale=mad*1.4826,rows=values.map((value,index)=>{const score=scale===0?(value===center?0:Infinity):Math.abs(value-center)/scale;return{index,value,score:Number.isFinite(score)?round(score):Infinity,accepted:score<=threshold}});return{median:center,mad:round(mad),robustSigma:round(scale),rows,accepted:rows.filter(row=>row.accepted).map(row=>row.value),rejected:rows.filter(row=>!row.accepted).map(row=>row.value)};}

export function varianceWeightedFusion(measurements){
  if(!Array.isArray(measurements)||!measurements.length||measurements.some(item=>!Number.isFinite(item.value)||!(item.variance>0)))throw new Error('finite measurements with positive variance are required');const totalWeight=sum(measurements.map(item=>1/item.variance)),value=sum(measurements.map(item=>item.value/item.variance))/totalWeight;return{value:round(value),variance:round(1/totalWeight),standardDeviation:round(Math.sqrt(1/totalWeight)),weights:measurements.map(item=>round((1/item.variance)/totalWeight))};
}

export function allanDeviation(samples,sampleIntervalSeconds,clusterSizes){
  if(!Array.isArray(samples)||samples.length<4||samples.some(value=>!Number.isFinite(value)))throw new Error('at least four finite samples are required');positive(sampleIntervalSeconds,'sample interval');if(!Array.isArray(clusterSizes)||!clusterSizes.length)throw new Error('cluster sizes are required');const rows=clusterSizes.map(size=>{if(!Number.isInteger(size)||size<1||size*2>samples.length)throw new Error('valid integer cluster sizes are required');const clusters=[];for(let start=0;start+size<=samples.length;start+=size)clusters.push(sum(samples.slice(start,start+size))/size);const differences=clusters.slice(1).map((value,index)=>value-clusters[index]),deviation=Math.sqrt(.5*sum(differences.map(value=>value*value))/differences.length);return{clusterSize:size,tauSeconds:round(size*sampleIntervalSeconds),allanDeviation:round(deviation)}});return{rows};
}

export function integrateAcceleration(samples,options={}){
  if(!Array.isArray(samples)||samples.length<2||samples.some(sample=>!Number.isFinite(sample.timeSeconds)||!vector3(sample.accelerationMps2)))throw new Error('at least two timed acceleration samples are required');let velocity=options.initialVelocityMps??[0,0,0],position=options.initialPositionM??[0,0,0];if(!vector3(velocity)||!vector3(position))throw new Error('three-axis initial state is required');const rows=[{timeSeconds:samples[0].timeSeconds,velocityMps:vectorRound(velocity),positionM:vectorRound(position)}];for(let index=1;index<samples.length;index++){const dt=samples[index].timeSeconds-samples[index-1].timeSeconds;if(!(dt>0))throw new Error('strictly increasing timestamps are required');const acceleration=[0,1,2].map(axis=>(samples[index-1].accelerationMps2[axis]+samples[index].accelerationMps2[axis])/2-(options.accelerationBiasMps2?.[axis]??0));position=[0,1,2].map(axis=>position[axis]+velocity[axis]*dt+.5*acceleration[axis]*dt*dt);velocity=[0,1,2].map(axis=>velocity[axis]+acceleration[axis]*dt);rows.push({timeSeconds:samples[index].timeSeconds,velocityMps:vectorRound(velocity),positionM:vectorRound(position)})}return{rows,finalVelocityMps:vectorRound(velocity),finalPositionM:vectorRound(position)};
}

export function sensorHealth(samples,options={}){
  if(!Array.isArray(samples)||samples.length<2||samples.some(sample=>!Number.isFinite(sample.timeSeconds)||!Number.isFinite(sample.value)))throw new Error('timed scalar samples are required');const expectedInterval=options.expectedIntervalSeconds??(samples.at(-1).timeSeconds-samples[0].timeSeconds)/(samples.length-1),gapFactor=options.gapFactor??1.5,range=options.range??[-Infinity,Infinity],stuckLimit=options.stuckSamples??3;let gaps=0,outOfRange=0,longestStuck=1,currentStuck=1;for(let index=0;index<samples.length;index++){if(samples[index].value<range[0]||samples[index].value>range[1])outOfRange++;if(index){if(samples[index].timeSeconds-samples[index-1].timeSeconds>expectedInterval*gapFactor)gaps++;if(samples[index].value===samples[index-1].value){currentStuck++;longestStuck=Math.max(longestStuck,currentStuck)}else currentStuck=1}}const mean=sum(samples.map(sample=>sample.value))/samples.length,std=Math.sqrt(sum(samples.map(sample=>(sample.value-mean)**2))/(samples.length-1));const issues=[];if(gaps)issues.push('timestamp-gaps');if(outOfRange)issues.push('out-of-range');if(longestStuck>=stuckLimit)issues.push('stuck-signal');if(options.maximumNoiseStd!=null&&std>options.maximumNoiseStd)issues.push('excessive-noise');return{sampleCount:samples.length,gapCount:gaps,outOfRangeCount:outOfRange,longestStuckRun:longestStuck,mean:round(mean),standardDeviation:round(std),issues,healthy:issues.length===0};
}

function rng(seed){let state=seed>>>0;return()=>{state=(1664525*state+1013904223)>>>0;return state/4294967296}}
export function monteCarloHeading(magnetic,options={}){
  if(!vector3(magnetic))throw new Error('three-axis magnetic vector is required');const iterations=options.iterations??500;if(!Number.isInteger(iterations)||iterations<1)throw new Error('positive integer iterations are required');const random=rng(options.seed??42),noise=options.uniformNoise??1,headings=[];for(let index=0;index<iterations;index++){const varied=magnetic.map(value=>value+(random()*2-1)*noise),result=tiltCompensatedHeading(varied,options.rollDeg??0,options.pitchDeg??0,options.declinationDeg??0);headings.push(result.headingDeg)}headings.sort((a,b)=>a-b);const q=p=>headings[Math.floor(p*(iterations-1))];return{iterations,seed:options.seed??42,p05HeadingDeg:round(q(.05)),p50HeadingDeg:round(q(.5)),p95HeadingDeg:round(q(.95)),span90Deg:round(q(.95)-q(.05))};
}

export function reportMarkdown(bundle){const lines=['# Sensor fusion analysis',''];if(bundle.calibration)lines.push(`Calibration bias: [${bundle.calibration.bias.join(', ')}] | Noise std: [${bundle.calibration.standardDeviation.join(', ')}]`,'');if(bundle.orientation)lines.push(`Orientation: roll ${bundle.orientation.rollDeg} deg, pitch ${bundle.orientation.pitchDeg} deg, yaw ${bundle.orientation.yawDeg} deg`,'');if(bundle.health)lines.push(`Sensor health: ${bundle.health.healthy?'healthy':'attention'} | Gaps: ${bundle.health.gapCount} | Out of range: ${bundle.health.outOfRangeCount}`,'');if(bundle.uncertainty)lines.push(`Heading p05/p50/p95: ${bundle.uncertainty.p05HeadingDeg} / ${bundle.uncertainty.p50HeadingDeg} / ${bundle.uncertainty.p95HeadingDeg} deg`,'');return`${lines.join('\n')}\n`}
