import { G, EARTH_MASS, EARTH_RADIUS, ROCKET_CONFIG, GUIDANCE_CONFIG } from './constants.js';
import { getTotalMass } from './state.js';
import { getAtmosphericDensity, getAirspeed, getGravity } from './physics.js';
import { predictOrbit, computeRemainingDeltaV } from './orbital.js';
import { calculateTimeToApoapsis } from './events.js';

// Guidance state
export let guidanceState = {
    phase: 'pre-launch',
    lastCommandedPitch: 90,
    throttle: 1.0,
    lastFlightPathAngle: 90,
    lastPeriapsis: 0,
    lastApoapsis: 0,
    isRetrograde: false,
    circularizationBurnStarted: false,
    retrogradeBurnStarted: false,
};

// Reset guidance state
export function resetGuidance() {
    guidanceState = {
        phase: 'pre-launch',
        lastCommandedPitch: 90,
        throttle: 1.0,
        lastFlightPathAngle: 90,
        lastPeriapsis: 0,
        lastApoapsis: 0,
        isRetrograde: false,
        circularizationBurnStarted: false,
        retrogradeBurnStarted: false,
    };
}

// ============================================================================
// MAIN GUIDANCE FUNCTION
// ============================================================================
export function computeGuidance(state, dt) {
    
    // ========================================================================
    // STEP 1: GATHER CURRENT STATE
    // ========================================================================
    
    const r = Math.sqrt(state.x * state.x + state.y * state.y);
    const altitude = r - EARTH_RADIUS;
    const velocity = Math.sqrt(state.vx * state.vx + state.vy * state.vy);
    
    // Local reference frame (changes as we move around Earth)
    const localUp = { x: state.x / r, y: state.y / r };
    const localEast = { x: localUp.y, y: -localUp.x };
    
    // Decompose velocity into vertical and horizontal components
    const vVertical = state.vx * localUp.x + state.vy * localUp.y;
    const vHorizontal = state.vx * localEast.x + state.vy * localEast.y;
    
    // Flight path angle (angle of velocity from horizontal)
    // 90° = straight up, 0° = horizontal, negative = descending
    const flightPathAngle = Math.atan2(vVertical, vHorizontal) * 180 / Math.PI;
    
    // Dynamic pressure (for max Q check)
    const airDensity = getAtmosphericDensity(altitude);
    const { airspeed } = getAirspeed();
    const dynamicPressure = 0.5 * airDensity * airspeed * airspeed;
    
    // ========================================================================
    // STEP 2: PREDICT ORBIT (vacuum assumption)
    // ========================================================================
    // "If I cut engines right now, what orbit am I on?"
    
    const orbit = predictOrbit(state);
    
    // ========================================================================
    // STEP 3: COMPUTE TARGET PARAMETERS
    // ========================================================================
    
    // Circular orbit velocity at target altitude
    const targetRadius = EARTH_RADIUS + GUIDANCE_CONFIG.targetAltitude;
    const mu = G * EARTH_MASS;
    const vCircular = Math.sqrt(mu / targetRadius);
    
    // How much more horizontal velocity do we need?
    const velocityDeficit = vCircular - vHorizontal;
    
    // Remaining delta-v from propellant
    const remainingDeltaV = computeRemainingDeltaV(state);
    
    // ========================================================================
    // STEP 4: PRIORITY-BASED GUIDANCE LOGIC
    // ========================================================================
    
    let commandedPitch;
    let commandedThrottle = 1.0;
    let phase;
    let debugInfo = {};
    
    // ------------------------------------------------------------------------
    // PRIORITY 1: HEIGHT — Get out of atmosphere
    // ------------------------------------------------------------------------
    
    if (altitude < GUIDANCE_CONFIG.atmosphereLimit) {
        
        // Sub-case: Very early flight (first few seconds)
        if (state.time < GUIDANCE_CONFIG.pitchKickStart) {
            phase = 'vertical-ascent';
            commandedPitch = 90;
            debugInfo.reason = 'Clearing pad — vertical';
        }
        
        // Sub-case: Pitch kick (gradual tilt to start gravity turn)
        else if (state.time < GUIDANCE_CONFIG.pitchKickEnd) {
            phase = 'pitch-kick';
            const progress = (state.time - GUIDANCE_CONFIG.pitchKickStart) / 
                           (GUIDANCE_CONFIG.pitchKickEnd - GUIDANCE_CONFIG.pitchKickStart);
            // Smooth cosine interpolation
            const smoothProgress = (1 - Math.cos(progress * Math.PI)) / 2;
            commandedPitch = 90 - smoothProgress * (90 - GUIDANCE_CONFIG.initialPitch);
            debugInfo.reason = 'Pitch kick — initiating gravity turn';
        }
        
        // Sub-case: In atmosphere, past pitch kick
        else {
            // ============================================================
            // ATMOSPHERIC PHASE — PRIORITY IS ESCAPE, NOT ORBIT SHAPING
            // ============================================================
            
            // PRIORITY 2: MAX Q — Protect structure
            if (dynamicPressure > GUIDANCE_CONFIG.maxQ * 0.8) {
                phase = 'max-q-protection';
                commandedPitch = flightPathAngle;
                debugInfo.reason = 'Max Q — following prograde exactly';
                debugInfo.q = dynamicPressure;
            }
            else {
                phase = 'atmospheric-ascent';
                
                // ============================================================
                // ALTITUDE-BASED MINIMUM PITCH CONSTRAINT
                // ============================================================
                
                const altitudeFraction = Math.min(1.0, altitude / GUIDANCE_CONFIG.atmosphereLimit);
                const minPitchForAltitude = 90 - altitudeFraction * altitudeFraction * 80; // Smooth quadratic curve
                
                // ============================================================
                // PROGRADE FOLLOWING WITH CONSTRAINTS
                // ============================================================
                
                // Start with prograde (minimizes angle of attack)
                let basePitch = flightPathAngle;
                
                // Calculate natural gravity turn rate
                const g = getGravity(r);
                const gamma = flightPathAngle * Math.PI / 180;
                const naturalTurnRate = (g * Math.cos(gamma) / velocity) * 180 / Math.PI;  // deg/sec
                
                // Measure actual turn rate
                const actualTurnRate = dt > 0 ? (guidanceState.lastFlightPathAngle - flightPathAngle) / dt : 0;
                
                // Store for next frame
                guidanceState.lastFlightPathAngle = flightPathAngle;
                
                // CONSTRAINT 1: Turn rate limiting
                let correction = 0;
                const turnRateExcess = actualTurnRate - naturalTurnRate;
                
                if (turnRateExcess > 0.5) {
                    correction = Math.min(5, turnRateExcess * 2);
                    debugInfo.reason = 'Turn rate excess — resisting';
                    debugInfo.turnRateExcess = turnRateExcess;
                }
                
                // CONSTRAINT 2: Minimum pitch for altitude (soft constraint)
                if (basePitch + correction < minPitchForAltitude) {
                    const deficit = minPitchForAltitude - (basePitch + correction);
                    correction += deficit * 0.3;
                    debugInfo.reason = 'Altitude minimum pitch — gentle correction';
                }
                
                // ============================================================
                // CONSTRAINT 3: MINIMUM VERTICAL VELOCITY AT ATMOSPHERE EXIT
                // ============================================================
                // 
                // Problem: For low orbits, we go very horizontal. But if we
                // exit the atmosphere with too little vertical velocity, we'll
                // fall back in before we can raise periapsis.
                //
                // Solution: Ensure minimum vVertical that scales with:
                // - Distance to atmosphere exit (more important as we get close)
                // - Target altitude (higher targets need more vVertical)
                //
                // Simple model: Need enough vVertical to coast to ~100km above 
                // atmosphere before gravity pulls us back down.
                // Time to coast: t ≈ 2 * vVertical / g (up and down)
                // So need vVertical that gives us margin above atmosphere
                // ============================================================
                
                const distanceToAtmoExit = GUIDANCE_CONFIG.atmosphereLimit - altitude;
                const targetAlt = GUIDANCE_CONFIG.targetAltitude;
                
                // Minimum vVertical needed at atmosphere exit
                // For 200km target: need ~300 m/s to have time to circularize
                // For 400km target: need ~500 m/s
                // Scale linearly with target altitude above atmosphere
                const targetAboveAtmo = targetAlt - GUIDANCE_CONFIG.atmosphereLimit;
                const minVVerticalAtExit = 200 + (targetAboveAtmo / 1000) * 0.5; // m/s
                
                // Scale requirement based on how close we are to exit
                // At 50km altitude: start caring about vVertical
                // At 70km: full requirement
                const proximityToExit = Math.max(0, (altitude - 50000) / 20000); // 0 at 50km, 1 at 70km
                const currentMinVVertical = minVVerticalAtExit * proximityToExit;
                
                debugInfo.minVVertical = currentMinVVertical;
                debugInfo.actualVVertical = vVertical;
                
                // If vVertical is below minimum, pitch up
                if (proximityToExit > 0.3 && vVertical < currentMinVVertical) {
                    const vVerticalDeficit = currentMinVVertical - vVertical;
                    
                    // Scale correction: bigger deficit = bigger pitch up
                    // Every 100 m/s deficit = 5° pitch up
                    const vVerticalCorrection = Math.min(15, vVerticalDeficit / 20);
                    
                    // Only apply if it's a significant correction
                    if (vVerticalCorrection > 2) {
                        correction += vVerticalCorrection;
                        debugInfo.reason = `Low vVertical (${vVertical.toFixed(0)} m/s < ${currentMinVVertical.toFixed(0)} m/s) — pitching up`;
                    }
                }
                
                // Final hard constraint
                commandedPitch = Math.max(minPitchForAltitude, basePitch + correction);
                
                debugInfo.basePitch = basePitch;
                debugInfo.correction = correction;
                debugInfo.minPitchForAltitude = minPitchForAltitude;
                debugInfo.naturalTurnRate = naturalTurnRate;
                debugInfo.actualTurnRate = actualTurnRate;
            }
        }
    }
    
    // ------------------------------------------------------------------------
    // ABOVE ATMOSPHERE: Active guidance
    // ------------------------------------------------------------------------
    
    else {
        // ================================================================
        // VACUUM GUIDANCE: Manage flight path angle during ascent
        // ================================================================
        
        phase = 'vacuum-guidance';
        
        const orbit = predictOrbit(state);
        const mu = G * EARTH_MASS;
        
        const isAscending = vVertical > 0;
        // ================================================================
        // CONSTANTS RELATIVE TO TARGET AND ATMOSPHERE
        // ================================================================
        
        const ATMOSPHERE = GUIDANCE_CONFIG.atmosphereLimit; // 70km
        const SAFE_PERIAPSIS = ATMOSPHERE + 30000; // 100km — safely above atmosphere
        const TARGET = GUIDANCE_CONFIG.targetAltitude;
        
        // For checking if we're in a stable orbit (not going to reenter)
        const isSuborbital = orbit.periapsis < SAFE_PERIAPSIS;
        
        const apoapsisError = orbit.apoapsis - GUIDANCE_CONFIG.targetAltitude;
        const periapsisError = orbit.periapsis - GUIDANCE_CONFIG.targetAltitude;
        
        debugInfo.apoapsisError = apoapsisError / 1000;
        debugInfo.periapsisError = periapsisError / 1000;
        debugInfo.flightPathAngle = flightPathAngle;
        
        commandedThrottle = 1.0;
        guidanceState.isRetrograde = false;
        
        const tolerance = 1000; // 10km
        
        // ================================================================
        // CALCULATE TARGET FLIGHT PATH ANGLE
        // ================================================================
        // 
        // Key insight: We want FPA = 0 when we reach TARGET altitude.
        // But the rate at which we pitch over depends on the target.
        //
        // For LOW orbits (150-300km): Need to go horizontal quickly
        //   - Start at ~15-20° and pitch down fast
        //   - Most delta-V goes to horizontal velocity
        //
        // For MEDIUM orbits (300-600km): Balanced approach
        //   - Start at ~20-30° and pitch down steadily
        //
        // For HIGH orbits (600km+): Stay steep longer
        //   - Start at ~30-45° to build altitude first
        //   - Then pitch down for horizontal velocity
        //   - Need more total delta-V, so trajectory matters more
        // ================================================================
        
        // Rocket can't pitch below -5°, so FPA can't go below -5° either
        const MIN_FPA = -5;
        
        // How far above atmosphere is the target?
        const targetAboveAtmo = TARGET - ATMOSPHERE;
        
        // How far above atmosphere are we currently?
        const altitudeAboveAtmo = Math.max(0, altitude - ATMOSPHERE);
        
        // Fraction of the way from atmosphere to target (0 at atmo exit, 1 at target)
        const progressToTarget = Math.min(1.0, altitudeAboveAtmo / targetAboveAtmo);
        
        // ================================================================
        // STARTING FPA — Continuous function of target altitude
        // ================================================================
        // 
        // Higher target = steeper starting angle (more time to pitch over)
        // Lower target = shallower starting angle (need horizontal velocity fast)
        //
        // Smooth scaling based on target altitude above atmosphere:
        // - Base: 10° (minimum for very low orbits)
        // - Scale: +1° per 15km of target above atmosphere
        // - Cap: 50° (maximum for very high orbits)
        //
        // Examples:
        //   150km target (80km above atmo): 10 + 80/15 = ~15°
        //   300km target (230km above atmo): 10 + 230/15 = ~25°
        //   600km target (530km above atmo): 10 + 530/15 = ~45°
        // ================================================================
        
        const startingFPA = Math.max(10, Math.min(50, 10 + targetAboveAtmo / 50000));
        
        debugInfo.startingFPA = startingFPA;
        
        // ================================================================
        // FPA PROFILE SHAPE — Continuous function of target altitude
        // ================================================================
        //
        // Use a power curve: FPA = startingFPA * (1 - progress)^exponent
        //
        // The exponent controls the shape:
        //   exponent > 1 = pitch down early (good for low orbits)
        //   exponent = 1 = linear decrease
        //   exponent < 1 = stay steep longer (good for high orbits)
        //
        // Smooth scaling:
        // - Base: 1.5 (pitch down early for lowest orbits)
        // - Scale: -0.1 per 100km of target above atmosphere
        // - Floor: 0.5 (stay steep for highest orbits)
        //
        // Examples:
        //   150km target (80km above atmo): 1.5 - 0.08 = ~1.4
        //   300km target (230km above atmo): 1.5 - 0.23 = ~1.3
        //   500km target (430km above atmo): 1.5 - 0.43 = ~1.1
        //   800km target (730km above atmo): 1.5 - 0.73 = ~0.8
        // ================================================================
        
        const profileExponent = Math.max(0.5, Math.min(1.5, 1.5 - targetAboveAtmo / 3000000));
        
        debugInfo.profileExponent = profileExponent;
        
        // Target FPA based on progress and profile shape
        // Clamp FPA to minimum (rocket can't pitch below -5°)
        let baseFPA = startingFPA * Math.pow(1 - progressToTarget, profileExponent);
        baseFPA = Math.max(MIN_FPA, baseFPA);
        
        debugInfo.progressToTarget = progressToTarget;
        debugInfo.baseFPA = baseFPA;
        
        // ================================================================
        // PERIAPSIS PREDICTION — Following pitch program
        // ================================================================
        //
        // Average FPA over remaining trajectory depends on profile shape.
        //
        // For FPA = startingFPA * (1 - progress)^exponent:
        // Average from currentProgress to 1 = integral / interval length
        //
        // For simplicity, approximate with midpoint value:
        // avgProgress = (currentProgress + 1) / 2
        // avgFPA ≈ startingFPA * (1 - avgProgress)^exponent
        
        const remainingProgress = 1 - progressToTarget;
        const avgProgress = (progressToTarget + 1) / 2;
        let averageFPA = startingFPA * Math.pow(1 - avgProgress, profileExponent);
        
        // Clamp average FPA to minimum (rocket can't pitch below -5°)
        averageFPA = Math.max(MIN_FPA, averageFPA);
        
        debugInfo.averageFPA = averageFPA;
        
        // Estimate gain ratio from average FPA
        // For negative FPA (descending), use absolute value for calculation
        const avgFPARad = Math.abs(averageFPA) * Math.PI / 180;
        const expectedGainRatio = avgFPARad > 0.01 ? Math.cos(avgFPARad) / Math.sin(avgFPARad) : 10;
        
        debugInfo.expectedGainRatio = expectedGainRatio;
        
        // Remaining apoapsis to gain
        const apoapsisToGain = Math.max(0, TARGET - orbit.apoapsis);
        
        // Expected periapsis gain based on average FPA
        const expectedPeriapsisGain = apoapsisToGain * expectedGainRatio;
        
        // Predicted final periapsis
        const predictedFinalPeriapsis = orbit.periapsis + expectedPeriapsisGain;
        
        debugInfo.apoapsisToGain = apoapsisToGain / 1000;
        debugInfo.expectedPeriapsisGain = expectedPeriapsisGain / 1000;
        debugInfo.predictedFinalPeriapsis = predictedFinalPeriapsis / 1000;
        
        // ================================================================
        // PERIAPSIS SAFETY CHECK (fallback for current danger)
        // ================================================================
        
        // Track periapsis change (need to store previous value)
        const periapsisChangeRate = (orbit.periapsis - (guidanceState.lastPeriapsis || orbit.periapsis)) / Math.max(0.01, dt);
        guidanceState.lastPeriapsis = orbit.periapsis;
        
        debugInfo.periapsisChangeRate = periapsisChangeRate; // m/s
        
        let periapsisSafetyBias = 0;
        
        if (orbit.periapsis < SAFE_PERIAPSIS) {  // Below 100km — in danger zone
            
            if (periapsisChangeRate > 500) {
                // Periapsis rising fast (>500 m/s) — we're OK, stay efficient
                periapsisSafetyBias = 0;
                debugInfo.periapsisStatus = 'rising-fast';
            }
            else if (periapsisChangeRate > 100) {
                // Periapsis rising slowly — small bias
                periapsisSafetyBias = 2;
                debugInfo.periapsisStatus = 'rising-slow';
            }
            else if (periapsisChangeRate > -100) {
                // Periapsis roughly stable — moderate bias
                periapsisSafetyBias = 5;
                debugInfo.periapsisStatus = 'stable';
            }
            else {
                // Periapsis FALLING — we're in trouble, pitch up
                periapsisSafetyBias = Math.min(15, 5 + (-periapsisChangeRate) / 200);
                debugInfo.periapsisStatus = 'falling';
            }
            
            // Extra bias if periapsis is deeply negative
            if (orbit.periapsis < -200000) {
                periapsisSafetyBias += 5;
            }
        }
        
        // ================================================================
        // ADJUST PITCH PROGRAM IF PREDICTION IS OFF
        // ================================================================
        
        let predictionBias = 0;
        
        // Always calculate prediction - it's useful even when apoapsis is at target
        if (predictedFinalPeriapsis < SAFE_PERIAPSIS && apoapsisToGain > 5000) {
            // Predicted to end up with low periapsis
            // Need to pitch DOWN (lower FPA) to increase periapsis gain ratio
            // Only adjust if we still have some apoapsis to gain (otherwise we're done)
            
            const predictedDeficit = SAFE_PERIAPSIS - predictedFinalPeriapsis;
            
            // Scale: Every 10km of predicted deficit = 1° pitch down (more sensitive)
            predictionBias = -Math.min(15, predictedDeficit / 10000);
            
            debugInfo.predictionStatus = `LOW: Pe predicted ${(predictedFinalPeriapsis/1000).toFixed(0)}km, need ${(SAFE_PERIAPSIS/1000).toFixed(0)}km, bias: ${predictionBias.toFixed(1)}°`;
        }
        else if (predictedFinalPeriapsis > TARGET * 1.1 && apoapsisToGain > 5000) {
            // Predicted to overshoot periapsis significantly
            // Can pitch UP slightly to be more efficient
            // Only adjust if we still have apoapsis to gain
            
            const predictedExcess = predictedFinalPeriapsis - TARGET;
            predictionBias = Math.min(5, predictedExcess / 50000);
            
            debugInfo.predictionStatus = `HIGH: Pe predicted ${(predictedFinalPeriapsis/1000).toFixed(0)}km, bias: ${predictionBias.toFixed(1)}°`;
        }
        else {
            // Prediction looks good, or we're too close to target
            predictionBias = 0;
            if (apoapsisToGain <= 5000) {
                debugInfo.predictionStatus = `Near target (${(apoapsisToGain/1000).toFixed(0)}km to go), no adjustment`;
            } else {
                debugInfo.predictionStatus = `OK: Pe predicted ${(predictedFinalPeriapsis/1000).toFixed(0)}km`;
            }
        }
        
        debugInfo.predictionBias = predictionBias;
        

        
        // ================================================================
        // FINAL TARGET FPA
        // ================================================================
        
        const targetFlightPathAngle = Math.max(0, baseFPA + periapsisSafetyBias + predictionBias);
        
        // How far off are we from the ideal flight path angle?
        const fpaError = flightPathAngle - targetFlightPathAngle;
        
        console.log('Final target FPA:', targetFlightPathAngle.toFixed(1), '°');
        console.log('Current FPA:', flightPathAngle.toFixed(1), '°');
        console.log('FPA error:', fpaError.toFixed(1), '°');
        console.log('==========================');
        
        debugInfo.baseFPA = baseFPA;
        debugInfo.periapsisSafetyBias = periapsisSafetyBias;
        debugInfo.targetFPA = targetFlightPathAngle;
        debugInfo.fpaError = fpaError;
        
        // ================================================================
        // REORDERED CASES — Periapsis safety comes first!
        // ================================================================
        
        // CASE 0: EMERGENCY — Periapsis critically low, regardless of apoapsis
        if (orbit.periapsis < 0 && apoapsisError >= -tolerance) {
            // Apoapsis is at or above target, but periapsis is BELOW GROUND
            // This is an emergency — we need to raise periapsis NOW
            
            phase = 'emergency-raise-periapsis';
            
            // Burn as horizontal as possible to raise periapsis
            // Yes, this will raise apoapsis too, but survival is priority
            commandedPitch = Math.max(0, flightPathAngle - 15);
            commandedThrottle = 1.0;
            
            debugInfo.reason = `EMERGENCY: Pe below ground (${(orbit.periapsis/1000).toFixed(0)}km) — burning horizontal`;
        }
        
        // CASE 1: Apoapsis too LOW — raise it
        else if (apoapsisError < -tolerance) {
            phase = 'raising-apoapsis';
            
            // We need more apoapsis. But we also need to manage our angle.
            // If we're too steep (FPA too high), pitch DOWN toward horizontal
            // If we're too shallow, pitch UP
            
            let correction = 0;
            
            // When prediction bias is large, prioritize periapsis correction
            if (Math.abs(predictionBias) > 5) {
                // Large prediction bias - directly correct to target FPA (100% correction)
                correction = -fpaError;
                debugInfo.reason = `Raising Apo — large prediction bias, direct correction (FPA: ${flightPathAngle.toFixed(1)}° → ${targetFlightPathAngle.toFixed(1)}°)`;
            } else if (fpaError > 5) {
                // Too steep — pitch down to go more horizontal
                // This adds more horizontal velocity, which raises apoapsis efficiently
                correction = -Math.min(10, fpaError * 0.5);
                debugInfo.reason = `Raising Apo — too steep, pitching down (FPA: ${flightPathAngle.toFixed(1)}° target: ${targetFlightPathAngle.toFixed(1)}°)`;
            } else if (fpaError < -5) {
                // Too shallow — pitch up slightly
                // We need some vertical component to keep climbing
                correction = Math.min(5, -fpaError * 0.3);
                debugInfo.reason = `Raising Apo — too shallow, pitching up (FPA: ${flightPathAngle.toFixed(1)}° target: ${targetFlightPathAngle.toFixed(1)}°)`;
                } else {
                // On target — follow prograde
                correction = 0;
                debugInfo.reason = `Raising Apo — on profile (FPA: ${flightPathAngle.toFixed(1)}° target: ${targetFlightPathAngle.toFixed(1)}°)`;
            }
            
            // Throttle based on deficit
            // But if periapsis is unsafe, use full throttle regardless
            if (orbit.periapsis < SAFE_PERIAPSIS) {
                commandedThrottle = 1.0;
            } else {
                const deficit = -apoapsisError;
                if (deficit < 50000) {
                    commandedThrottle = Math.max(0.2, deficit / 50000);
                } else {
                    commandedThrottle = 1.0;
                }
            }
            
            commandedPitch = flightPathAngle + correction;
            
            console.log('CASE 1 - Raising Apo: correction =', correction.toFixed(1), '°, commandedPitch =', commandedPitch.toFixed(1), '°');
        }
        
        // CASE 2: Apoapsis on target, periapsis low — build periapsis
        else if (orbit.periapsis < SAFE_PERIAPSIS) {
            phase = 'building-periapsis';
            
            // Apoapsis is good (within tolerance)
            // Follow target FPA which includes prediction bias for periapsis adjustment
            // Target FPA tells us how horizontal we should be to achieve safe periapsis
            
            // Correct toward target FPA
            // When prediction bias is large, directly target the FPA (more aggressive)
            let correction = 0;
            if (Math.abs(predictionBias) > 5) {
                // Large prediction bias means periapsis is in danger
                // Directly correct to target FPA (use 100% of error)
                correction = -fpaError;
            } else if (fpaError > 3) {
                // Normal correction
                correction = -Math.min(10, fpaError * 0.5);
            } else if (fpaError < -3) {
                // Too shallow — pitch up slightly
                correction = Math.min(5, -fpaError * 0.3);
            }
            
            commandedPitch = Math.max(0, flightPathAngle + correction);
            commandedThrottle = 1.0;
            
            console.log('CASE 2 - Building Pe: correction =', correction.toFixed(1), '°, commandedPitch =', commandedPitch.toFixed(1), '°');
            
            debugInfo.reason = `Building Pe (${(orbit.periapsis/1000).toFixed(0)}km → ${(SAFE_PERIAPSIS/1000).toFixed(0)}km, FPA: ${flightPathAngle.toFixed(1)}° target: ${targetFlightPathAngle.toFixed(1)}°)`;
        }
        
        // CASE 3: Apoapsis TOO HIGH — need to lower it
        else if (apoapsisError > tolerance) {
            phase = 'apoapsis-too-high';
            
            // Periapsis is safe (>100km), so we can coast
            commandedThrottle = 0;
            commandedPitch = flightPathAngle;
            
            debugInfo.reason = `Apo too high — coasting for retrograde`;
        }
        
        // CASE 4: Both good but periapsis below target — circularize
        else if (periapsisError < -tolerance) {
            phase = 'coasting-to-circ';
            
            // Calculate circularization burn timing
            // Symmetric burn: start burnTime/2 before apoapsis, end burnTime/2 after
            const r_apo = EARTH_RADIUS + orbit.apoapsis;
            const v_circular = Math.sqrt(mu / r_apo);
            const v_at_apo = Math.sqrt(mu * (2/r_apo - 1/orbit.semiMajorAxis));
            const circDeltaV = Math.max(0, v_circular - v_at_apo);
            
            let burnTime = 0;
            if (state.currentStage < ROCKET_CONFIG.stages.length && circDeltaV > 0) {
                const stage = ROCKET_CONFIG.stages[state.currentStage];
                const currentMass = getTotalMass();
                const thrust = stage.thrustVac;
                
                if (thrust > 0) {
                    // Estimate burn time using current mass
                    // For more accuracy, we could use average mass (mass - deltaV*mdot/2),
                    // but current mass gives reasonable approximation
                    burnTime = circDeltaV * currentMass / thrust;
                }
            }
            
            // Time to apoapsis using proper orbital mechanics (accounts for full orbit)
            let timeToApoapsis = calculateTimeToApoapsis(orbit, state, mu);
            
            // Fallback to simple calculation if orbital calculation fails
            if (!isFinite(timeToApoapsis) || timeToApoapsis <= 0) {
                if (isAscending && orbit.apoapsis > altitude) {
                    timeToApoapsis = (orbit.apoapsis - altitude) / Math.max(1, vVertical);
                } else {
                    timeToApoapsis = Infinity;
                }
            }
            
            // Symmetric burn: start half the burn time before apoapsis
            const burnStartOffset = burnTime / 2;
            
            // Start burning when we're within burnStartOffset of apoapsis
            const shouldBurn = isFinite(timeToApoapsis) && 
                              burnTime > 0 &&
                              timeToApoapsis <= burnStartOffset && 
                              orbit.periapsis >= SAFE_PERIAPSIS;
            
            if (shouldBurn) {
                phase = 'circularizing';
                commandedPitch = flightPathAngle; // Prograde
                
                const periDeficit = -periapsisError;
                
                // If periapsis is still unsafe, use full throttle
                // Otherwise, throttle down for precision when close
                if (orbit.periapsis < SAFE_PERIAPSIS) {
                    commandedThrottle = 1.0;
                } else {
                    if (periDeficit < 30000) {
                        commandedThrottle = Math.max(0.1, periDeficit / 30000);
                    } else {
                        commandedThrottle = 1.0;
                    }
                }
                
                debugInfo.reason = `Circularizing (${(periDeficit/1000).toFixed(0)}km Pe to go)`;
            } else {
                commandedThrottle = 0;
                commandedPitch = flightPathAngle;
                
                debugInfo.reason = `Coasting to Apo (${timeToApoapsis.toFixed(0)}s, burn in ${(timeToApoapsis - burnStartOffset).toFixed(0)}s)`;
            }
            
            debugInfo.circDeltaV = circDeltaV;
            debugInfo.burnTime = burnTime;
        }
        
        // CASE 5: Orbit achieved!
        else {
            phase = 'orbit-achieved';
            commandedThrottle = 0;
            commandedPitch = flightPathAngle;
            
            debugInfo.reason = `Orbit achieved! (e=${orbit.eccentricity.toFixed(4)})`;
        }
        
        // Apply pitch constraints
        commandedPitch = Math.max(-5, Math.min(90, commandedPitch));
        
        debugInfo.commandedPitch = commandedPitch;
    }
    
    // ========================================================================
    // STEP 5: APPLY CONSTRAINTS
    // ========================================================================
    
    // Clamp pitch to valid range
    commandedPitch = Math.max(-5, Math.min(90, commandedPitch));
    
    // Rate limiting — rocket can only rotate so fast
    if (dt > 0) {
        const maxChange = GUIDANCE_CONFIG.maxPitchRate * dt;
        const desiredChange = commandedPitch - guidanceState.lastCommandedPitch;
        
        if (Math.abs(desiredChange) > maxChange) {
            commandedPitch = guidanceState.lastCommandedPitch + 
                            Math.sign(desiredChange) * maxChange;
        }
    }
    
    // Store for next frame
    guidanceState.lastCommandedPitch = commandedPitch;
    guidanceState.phase = phase;
    guidanceState.throttle = commandedThrottle;
    
    // ========================================================================
    // STEP 6: CONVERT TO THRUST VECTOR
    // ========================================================================
    
    let thrustDir;
    if (guidanceState.isRetrograde) {
        // Retrograde burn: thrust opposite to velocity vector
        const velocityMag = Math.sqrt(state.vx * state.vx + state.vy * state.vy);
        if (velocityMag > 0) {
            thrustDir = {
                x: -state.vx / velocityMag,
                y: -state.vy / velocityMag
            };
        } else {
            thrustDir = { x: -localEast.x, y: -localEast.y };
        }
    } else {
        // Normal prograde/guided thrust
        const pitchRad = commandedPitch * Math.PI / 180;
        thrustDir = {
            x: Math.cos(pitchRad) * localEast.x + Math.sin(pitchRad) * localUp.x,
            y: Math.cos(pitchRad) * localEast.y + Math.sin(pitchRad) * localUp.y
        };
    }
    
    // Normalize
    const mag = Math.sqrt(thrustDir.x * thrustDir.x + thrustDir.y * thrustDir.y);
    if (mag > 0) {
        thrustDir.x /= mag;
        thrustDir.y /= mag;
    }
    
    return {
        pitch: commandedPitch,
        thrustDir: thrustDir,
        throttle: commandedThrottle,
        phase: phase,
        debug: debugInfo,
        orbit: orbit,
        velocityDeficit: velocityDeficit,
        remainingDeltaV: remainingDeltaV,
    };
}

