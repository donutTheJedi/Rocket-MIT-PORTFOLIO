import { G, EARTH_MASS, EARTH_RADIUS, KARMAN_LINE, ROCKET_CONFIG, GUIDANCE_CONFIG } from './constants.js';
import { state, getAltitude, getTotalMass } from './state.js';
import { getMassFlowRate, getGravity, getCurrentThrust, getDrag, getAirspeed } from './physics.js';
import { predictOrbit } from './orbital.js';

// Store absolute burn start times for accurate countdown
// These are calculated ONCE when entering each phase and NEVER recalculated
let absoluteBurnStartTime = null; // For circularization burn
let absoluteRetrogradeBurnStartTime = null; // For retrograde burn

// Format time as MM:SS.ms
export function formatTime(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    const ms = Math.floor((seconds % 1) * 100);
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}.${ms.toString().padStart(2, '0')}`;
}

// Format T-minus countdown
export function formatTMinus(seconds) {
    if (seconds <= 0) return '00:00';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

/**
 * Calculate time to apoapsis using orbital mechanics
 * 
 * @param {Object} orbit - Orbital elements from predictOrbit()
 * @param {Object} state - Current state object with x, y, vx, vy
 * @param {number} mu - Standard gravitational parameter (m³/s²)
 * @returns {number} Time to apoapsis (s), or Infinity if unreachable
 */
export function calculateTimeToApoapsis(orbit, state, mu) {
    if (orbit.isEscape || orbit.eccentricity >= 1 || orbit.semiMajorAxis <= 0) {
        return Infinity;
    }
    
    const e = orbit.eccentricity;
    const a = orbit.semiMajorAxis;
    
    // Handle near-circular orbits specially
    if (e < 1e-6) {
        // Circular orbit: apoapsis is π radians ahead
        // Time = T/2 regardless of position
        const T = 2 * Math.PI * Math.sqrt((a * a * a) / mu);
        return T / 2;
    }
    
    const r = Math.sqrt(state.x * state.x + state.y * state.y);
    
    // Calculate cos(θ) from orbital equation
    const p = a * (1 - e * e);  // semi-latus rectum
    let cosTheta = (p / r - 1) / e;
    
    // Check if r is valid for this orbit
    if (Math.abs(cosTheta) > 1.001) {
        // r doesn't lie on computed orbit — orbit prediction may be stale
        console.warn('Position inconsistent with orbit, cosTheta =', cosTheta);
        cosTheta = Math.max(-1, Math.min(1, cosTheta));
    }
    cosTheta = Math.max(-1, Math.min(1, cosTheta));
    
    // Determine quadrant using radial velocity (r·v)
    const rDotV = state.x * state.vx + state.y * state.vy;
    
    let theta;
    if (rDotV >= 0) {
        theta = Math.acos(cosTheta);           // 0 to π (moving away from periapsis)
    } else {
        theta = 2 * Math.PI - Math.acos(cosTheta);  // π to 2π (moving toward periapsis)
    }
    
    // True anomaly → Eccentric anomaly
    // tan(E/2) = sqrt((1-e)/(1+e)) * tan(θ/2)
    const tanHalfTheta = Math.tan(theta / 2);
    const tanHalfE = Math.sqrt((1 - e) / (1 + e)) * tanHalfTheta;
    let E = 2 * Math.atan(tanHalfE);
    
    // Ensure E is in [0, 2π]
    if (E < 0) E += 2 * Math.PI;
    
    // Eccentric anomaly → Mean anomaly (Kepler's equation)
    const M = E - e * Math.sin(E);
    
    // Orbital period
    const T = 2 * Math.PI * Math.sqrt((a * a * a) / mu);
    
    // Mean anomaly at apoapsis is π
    // Time to apoapsis = time for M to reach π
    let timeToApo;
    if (M <= Math.PI) {
        timeToApo = (Math.PI - M) / (2 * Math.PI) * T;
    } else {
        // Past apoapsis, time to next one
        timeToApo = (2 * Math.PI + Math.PI - M) / (2 * Math.PI) * T;
    }
    
    return timeToApo;
}

/**
 * Calculate time to periapsis using orbital mechanics
 * 
 * @param {Object} orbit - Orbital elements from predictOrbit()
 * @param {Object} state - Current state object with x, y, vx, vy
 * @param {number} mu - Standard gravitational parameter (m³/s²)
 * @returns {number} Time to periapsis (s), or Infinity if unreachable
 */
function calculateTimeToPeriapsis(orbit, state, mu) {
    if (orbit.isEscape || orbit.eccentricity >= 1 || orbit.semiMajorAxis <= 0) {
        return Infinity;
    }
    
    const e = orbit.eccentricity;
    const a = orbit.semiMajorAxis;
    
    // Handle near-circular orbits specially
    if (e < 1e-6) {
        // Circular orbit: periapsis is π radians ahead
        const T = 2 * Math.PI * Math.sqrt((a * a * a) / mu);
        return T / 2;
    }
    
    const r = Math.sqrt(state.x * state.x + state.y * state.y);
    
    // Calculate cos(θ) from orbital equation
    const p = a * (1 - e * e);  // semi-latus rectum
    let cosTheta = (p / r - 1) / e;
    cosTheta = Math.max(-1, Math.min(1, cosTheta));
    
    // Determine quadrant using radial velocity (r·v)
    const rDotV = state.x * state.vx + state.y * state.vy;
    
    let theta;
    if (rDotV >= 0) {
        theta = Math.acos(cosTheta);           // 0 to π (moving away from periapsis)
    } else {
        theta = 2 * Math.PI - Math.acos(cosTheta);  // π to 2π (moving toward periapsis)
    }
    
    // True anomaly → Eccentric anomaly
    const tanHalfTheta = Math.tan(theta / 2);
    const tanHalfE = Math.sqrt((1 - e) / (1 + e)) * tanHalfTheta;
    let E = 2 * Math.atan(tanHalfE);
    
    // Ensure E is in [0, 2π]
    if (E < 0) E += 2 * Math.PI;
    
    // Eccentric anomaly → Mean anomaly (Kepler's equation)
    const M = E - e * Math.sin(E);
    
    // Orbital period
    const T = 2 * Math.PI * Math.sqrt((a * a * a) / mu);
    
    // Mean anomaly at periapsis is 0 (or 2π)
    // Time to periapsis depends on whether we're before or after periapsis
    let timeToPeri;
    if (theta < Math.PI) {
        // Before apoapsis, going away from periapsis
        // Need to go to apoapsis then back to periapsis
        timeToPeri = (2 * Math.PI - M) / (2 * Math.PI) * T;
    } else {
        // After apoapsis, moving toward periapsis
        timeToPeri = (2 * Math.PI - M) / (2 * Math.PI) * T;
    }
    
    return timeToPeri;
}

/**
 * Calculate time to reach target altitude accounting for acceleration
 * Uses kinematic equation: s = v0*t + 0.5*a*t²
 * Solves for t: t = (-v0 + sqrt(v0² + 2*a*s)) / a
 * 
 * @param {number} currentAltitude - Current altitude (m)
 * @param {number} targetAltitude - Target altitude (m)
 * @param {number} vVert - Current vertical velocity (m/s)
 * @returns {number} Time to reach target altitude (s), or Infinity if unreachable
 */
function calculateTimeToAltitude(currentAltitude, targetAltitude, vVert) {
    const altitudeDiff = targetAltitude - currentAltitude;
    
    // If already past target or no vertical velocity toward target, return Infinity
    if (altitudeDiff <= 0) return Infinity;
    if (vVert <= 0) return Infinity;
    
    // Calculate vertical acceleration
    const r = Math.sqrt(state.x * state.x + state.y * state.y);
    const localUp = { x: state.x / r, y: state.y / r };
    
    // Gravity component (negative in vertical direction)
    const gravity = getGravity(r);
    const gVert = -gravity;
    
    // Thrust component in vertical direction
    let thrustVert = 0;
    if (state.engineOn && state.currentStage < ROCKET_CONFIG.stages.length) {
        const throttle = state.guidanceThrottle !== undefined ? state.guidanceThrottle : 1.0;
        const thrust = getCurrentThrust(currentAltitude, throttle);
        const mass = getTotalMass();
        const thrustAccel = thrust / mass;
        
        // Get thrust direction from guidance or burn mode
        let thrustDir;
        if (state.burnMode) {
            // In burn mode, thrust is in burn direction (prograde, retrograde, etc.)
            const velocity = Math.sqrt(state.vx * state.vx + state.vy * state.vy);
            if (velocity > 0) {
                const prograde = { x: state.vx / velocity, y: state.vy / velocity };
                if (state.burnMode === 'prograde') {
                    thrustDir = prograde;
                } else if (state.burnMode === 'retrograde') {
                    thrustDir = { x: -prograde.x, y: -prograde.y };
                } else {
                    // For other burn modes, assume prograde for simplicity
                    thrustDir = prograde;
                }
            } else {
                thrustDir = localUp;
            }
        } else {
            // Use guidance pitch
            const pitch = state.guidancePitch !== undefined ? state.guidancePitch : 90.0;
            const pitchRad = pitch * Math.PI / 180;
            const localEast = { x: localUp.y, y: -localUp.x };
            thrustDir = {
                x: Math.cos(pitchRad) * localEast.x + Math.sin(pitchRad) * localUp.x,
                y: Math.cos(pitchRad) * localEast.y + Math.sin(pitchRad) * localUp.y
            };
        }
        
        const thrustVertAccel = thrustAccel * (thrustDir.x * localUp.x + thrustDir.y * localUp.y);
        thrustVert = thrustVertAccel;
    }
    
    // Drag component (opposes velocity, so negative if ascending)
    let dragVert = 0;
    const { airspeed } = getAirspeed();
    if (airspeed > 0) {
        const drag = getDrag(currentAltitude, airspeed);
        const mass = getTotalMass();
        const dragAccel = drag / mass;
        // Drag opposes velocity direction
        const velocity = Math.sqrt(state.vx * state.vx + state.vy * state.vy);
        if (velocity > 0) {
            const velDir = { x: state.vx / velocity, y: state.vy / velocity };
            const dragDirVert = -(velDir.x * localUp.x + velDir.y * localUp.y);
            dragVert = dragAccel * dragDirVert;
        }
    }
    
    // Total vertical acceleration
    const aVert = gVert + thrustVert + dragVert;
    
    // Use kinematic equation: s = v0*t + 0.5*a*t²
    // Solve for t: t = (-v0 + sqrt(v0² + 2*a*s)) / a
    // For constant acceleration approximation
    if (Math.abs(aVert) < 0.01) {
        // Near-zero acceleration, use constant velocity
        return altitudeDiff / vVert;
    }
    
    const discriminant = vVert * vVert + 2 * aVert * altitudeDiff;
    if (discriminant < 0) {
        // No real solution (won't reach target with current acceleration)
        return Infinity;
    }
    
    const t = (-vVert + Math.sqrt(discriminant)) / aVert;
    return t > 0 ? t : Infinity;
}

// Add mission event
export function addEvent(text) {
    const timeStr = formatTime(state.time);
    state.events.push({ time: timeStr, text });
    const eventList = document.getElementById('event-list');
    const eventDiv = document.createElement('div');
    eventDiv.className = 'event';
    eventDiv.innerHTML = `<span class="event-time">T+${timeStr}</span> ${text}`;
    eventList.insertBefore(eventDiv, eventList.firstChild);
}

// ============================================================================
// Calculate upcoming burn events
// ============================================================================
export function calculateBurnEvents() {
    const altitude = getAltitude();
    const r = Math.sqrt(state.x * state.x + state.y * state.y);
    const velocity = Math.sqrt(state.vx * state.vx + state.vy * state.vy);
    const localUp = { x: state.x / r, y: state.y / r };
    const localEast = { x: localUp.y, y: -localUp.x };
    const vVertical = state.vx * localUp.x + state.vy * localUp.y;
    const vHorizontal = state.vx * localEast.x + state.vy * localEast.y;
    const isAscending = vVertical > 0;
    
    // Only calculate burn events if we're in vacuum (above atmosphere)
    if (altitude < GUIDANCE_CONFIG.atmosphereLimit) {
        return [];
    }
    
    const orbit = predictOrbit(state);
    const mu = G * EARTH_MASS;
    const tolerance = 10000; // 10km
    const apoError = orbit.apoapsis - GUIDANCE_CONFIG.targetAltitude;
    const periError = orbit.periapsis - GUIDANCE_CONFIG.targetAltitude;
    
    const events = [];
    
    // ========================================================================
    // STRATEGY SELECTION: Same logic as guidance.js
    // ========================================================================
    // Calculate circularization burn time to determine strategy
    const r_apo = EARTH_RADIUS + orbit.apoapsis;
    const v_circular = Math.sqrt(mu / r_apo);
    const v_at_apo = orbit.semiMajorAxis > 0 
        ? Math.sqrt(mu * (2 / r_apo - 1 / orbit.semiMajorAxis))
        : velocity;
    const circularizationDeltaV = Math.max(0, v_circular - v_at_apo);
    
    let circularizationBurnTime = 0;
    if (state.currentStage < ROCKET_CONFIG.stages.length && circularizationDeltaV > 0) {
        const stage = ROCKET_CONFIG.stages[state.currentStage];
        const currentMass = getTotalMass();
        const thrust = stage.thrustVac;
        if (thrust > 0) {
            circularizationBurnTime = circularizationDeltaV * currentMass / thrust;
        }
    }
    
    const useDirectAscent = GUIDANCE_CONFIG.targetAltitude < 250000 || 
                            (circularizationBurnTime > 0 && 
                             circularizationBurnTime > 60 && 
                             apoError < tolerance);
    
    // ========================================================================
    // TRADITIONAL STRATEGY: Circularization burn at apoapsis
    // ========================================================================
    // Only predict circularization if more than 25 minutes have passed
    if (!useDirectAscent && state.time >= 1500 && periError < -tolerance && apoError >= -tolerance) {
        const r = Math.sqrt(state.x * state.x + state.y * state.y);
        const altitudeToApoapsis = orbit.apoapsis - altitude;
        
        let timeUntilBurnStart = Infinity;
        
        if (!state.engineOn && isAscending && altitudeToApoapsis > 0) {
            // Coasting: calculate absolute burn start time ONCE, then just subtract current time
            if (absoluteBurnStartTime === null) {
                // Calculate time to apoapsis using orbital mechanics
                const calculatedTimeToApo = calculateTimeToApoapsis(orbit, state, mu);
                
                if (isFinite(calculatedTimeToApo) && calculatedTimeToApo > 0) {
                    // Use circularization burn time calculated above
                    const burnStartOffset = circularizationBurnTime / 2;
                    
                    // Store absolute burn start time (calculated ONCE, never recalculated)
                    absoluteBurnStartTime = state.time + calculatedTimeToApo - burnStartOffset;
                }
            }
            
            // Simple subtraction: stored absolute time minus current time
            if (absoluteBurnStartTime !== null) {
                timeUntilBurnStart = absoluteBurnStartTime - state.time;
                
                // If we've passed burn start, reset
                if (timeUntilBurnStart <= 0) {
                    absoluteBurnStartTime = null;
                }
            }
            
            // Add event using the stored time
            if (timeUntilBurnStart > 0 && timeUntilBurnStart < 10000) {
                events.push({ 
                    time: timeUntilBurnStart, 
                    name: 'Circularization burn start',
                    type: 'circularization',
                    burnTime: 0,
                    deltaV: 0
                });
            }
        } else if (state.engineOn && isAscending && altitudeToApoapsis > 0) {
            // Thrusting: Even with engine on, use orbital mechanics for time to apoapsis
            // (The rocket may be thrusting but still following an orbital trajectory)
            if (absoluteBurnStartTime === null) {
                // Calculate time to apoapsis using orbital mechanics (Kepler's equation)
                let timeToApoapsis = calculateTimeToApoapsis(orbit, state, mu);
                
                // Fallback to kinematic calculation if orbital calculation fails
                if (!isFinite(timeToApoapsis) || timeToApoapsis <= 0) {
                    const localUp = { x: state.x / r, y: state.y / r };
                    const vVert = state.vx * localUp.x + state.vy * localUp.y;
                    if (vVert > 0) {
                        timeToApoapsis = calculateTimeToAltitude(altitude, orbit.apoapsis, vVert);
                        if (!isFinite(timeToApoapsis) || timeToApoapsis <= 0) {
                            timeToApoapsis = altitudeToApoapsis / Math.max(1, vVertical);
                        }
                    }
                }
                
                // Use circularization burn time calculated above
                const burnStartTimeBeforeApo = circularizationBurnTime / 2;
                
                if (isFinite(timeToApoapsis) && timeToApoapsis > 0 && timeToApoapsis > burnStartTimeBeforeApo) {
                    absoluteBurnStartTime = state.time + timeToApoapsis - burnStartTimeBeforeApo;
                }
            }
            
            // Use stored absolute time
            if (absoluteBurnStartTime !== null) {
                timeUntilBurnStart = absoluteBurnStartTime - state.time;
                
                if (timeUntilBurnStart > 0 && timeUntilBurnStart < 10000) {
                    events.push({ 
                        time: timeUntilBurnStart, 
                        name: 'Circularization burn start',
                        type: 'circularization',
                        burnTime: 0,
                        deltaV: 0
                    });
                } else if (timeUntilBurnStart <= 0) {
                    // Passed burn start, reset
                    absoluteBurnStartTime = null;
                }
            }
        }
    } else {
        // Not in circularization phase, reset stored time
        absoluteBurnStartTime = null;
    }
    
    // ========================================================================
    // DIRECT ASCENT STRATEGY: Prograde burn to raise periapsis
    // ========================================================================
    if (useDirectAscent && periError < -tolerance && state.engineOn) {
        // Currently burning to raise periapsis - show countdown to target
        // Note: This is an active burn, not a future event, so we don't add it to events
        // The guidance system handles this phase
    }
    
    // ========================================================================
    // RETROGRADE BURN AT PERIAPSIS
    // Direct ascent: Expected after raising periapsis (apoapsis will be high)
    // Traditional: Edge case if apoapsis overshoots
    // ========================================================================
    if (periError >= -tolerance && apoError > tolerance) {
        const altitudeToPeriapsis = altitude - orbit.periapsis;
        let timeUntilBurnStart = Infinity;
        
        // Calculate absolute burn start time ONCE, then just subtract current time
        if (absoluteRetrogradeBurnStartTime === null) {
            // Calculate time to periapsis using orbital mechanics
            let timeToPeriapsis = calculateTimeToPeriapsis(orbit, state, mu);
            
            // Fallback to simple calculation if orbital calculation fails
            if (!isFinite(timeToPeriapsis) || timeToPeriapsis <= 0) {
                if (!isAscending && altitudeToPeriapsis > 0) {
                    timeToPeriapsis = altitudeToPeriapsis / Math.max(1, -vVertical);
                }
            }
            
            // Calculate retrograde delta-v and burn time
            const r_peri = EARTH_RADIUS + orbit.periapsis;
            const r_target = EARTH_RADIUS + GUIDANCE_CONFIG.targetAltitude;
            const a_target = (r_peri + r_target) / 2;
            const v_peri_target = Math.sqrt(mu * (2 / r_peri - 1 / a_target));
            const v_at_peri = orbit.semiMajorAxis > 0
                ? Math.sqrt(mu * (2 / r_peri - 1 / orbit.semiMajorAxis))
                : velocity;
            const retrogradeDeltaV = Math.max(0, v_at_peri - v_peri_target);
            
            let retrogradeBurnTime = 0;
            if (state.currentStage < ROCKET_CONFIG.stages.length && retrogradeDeltaV > 0) {
                const stage = ROCKET_CONFIG.stages[state.currentStage];
                const currentMass = getTotalMass();
                const thrust = stage.thrustVac;
                if (thrust > 0) {
                    retrogradeBurnTime = retrogradeDeltaV * currentMass / thrust;
                }
            }
            
            const burnStartTimeBeforePeri = retrogradeBurnTime / 2;
            
            // Store absolute burn start time
            if (isFinite(timeToPeriapsis) && timeToPeriapsis > 0 && timeToPeriapsis > burnStartTimeBeforePeri) {
                absoluteRetrogradeBurnStartTime = state.time + timeToPeriapsis - burnStartTimeBeforePeri;
            }
        }
        
        // Use stored absolute time
        if (absoluteRetrogradeBurnStartTime !== null) {
            timeUntilBurnStart = absoluteRetrogradeBurnStartTime - state.time;
            
            if (timeUntilBurnStart > 0 && timeUntilBurnStart < 10000) {
                events.push({ 
                    time: timeUntilBurnStart, 
                    name: 'Retrograde burn start',
                    type: 'retrograde',
                    burnTime: 0,
                    deltaV: 0
                });
            } else if (timeUntilBurnStart <= 0) {
                // Passed burn start, reset
                absoluteRetrogradeBurnStartTime = null;
            }
        }
    } else {
        // Not in retrograde phase, reset stored time
        absoluteRetrogradeBurnStartTime = null;
    }
    
    return events;
}

// Get next upcoming event
export function getNextEvent() {
    const altitude = getAltitude();
    const velocity = Math.sqrt(state.vx * state.vx + state.vy * state.vy);
    const events = [];
    
    // Pitch program start (first pitch change at 10s)
    if (state.time < 10) {
        events.push({ time: 10 - state.time, name: 'Pitch program start' });
    }
    
    // Kármán line (100km)
    if (altitude < KARMAN_LINE && !state.events.some(e => e.text.includes("Kármán"))) {
        const r = Math.sqrt(state.x * state.x + state.y * state.y);
        const localUp = { x: state.x / r, y: state.y / r };
        const vVert = state.vx * localUp.x + state.vy * localUp.y;
        if (vVert > 0) {
            const timeToKarman = calculateTimeToAltitude(altitude, KARMAN_LINE, vVert);
            if (timeToKarman > 0 && timeToKarman < 10000 && isFinite(timeToKarman)) {
                events.push({ time: timeToKarman, name: 'Kármán line' });
            }
        }
    }
    
    // Fairing jettison (110km)
    if (!state.fairingJettisoned && altitude < ROCKET_CONFIG.fairingJettisonAlt) {
        const r = Math.sqrt(state.x * state.x + state.y * state.y);
        const localUp = { x: state.x / r, y: state.y / r };
        const vVert = state.vx * localUp.x + state.vy * localUp.y;
        if (vVert > 0) {
            const timeToFairing = calculateTimeToAltitude(altitude, ROCKET_CONFIG.fairingJettisonAlt, vVert);
            if (timeToFairing > 0 && timeToFairing < 10000 && isFinite(timeToFairing)) {
                events.push({ time: timeToFairing, name: 'Fairing jettison' });
            }
        }
    }
    
    // Stage separation (when stage 0 propellant runs out)
    if (state.currentStage === 0 && state.propellantRemaining[0] > 0 && state.engineOn) {
        const throttle = state.guidanceThrottle !== undefined ? state.guidanceThrottle : 1.0;
        const massFlowRate = getMassFlowRate(altitude, throttle);
        if (massFlowRate > 0) {
            const timeToSeparation = state.propellantRemaining[0] / massFlowRate;
            if (timeToSeparation > 0 && timeToSeparation < 10000) {
                events.push({ time: timeToSeparation, name: 'Stage separation' });
            }
        }
    }
    
    // Orbit (150km altitude and engine off)
    const inOrbit = altitude >= 150000 && !state.engineOn;
    if (!inOrbit && altitude < 150000) {
        const r = Math.sqrt(state.x * state.x + state.y * state.y);
        const localUp = { x: state.x / r, y: state.y / r };
        const vVert = state.vx * localUp.x + state.vy * localUp.y;
        if (vVert > 0) {
            const timeToOrbit = calculateTimeToAltitude(altitude, 150000, vVert);
            if (timeToOrbit > 0 && timeToOrbit < 10000 && isFinite(timeToOrbit)) {
                let totalTime = timeToOrbit;
                if (state.engineOn && state.currentStage < ROCKET_CONFIG.stages.length) {
                    const throttle = state.guidanceThrottle !== undefined ? state.guidanceThrottle : 1.0;
                    const massFlowRate = getMassFlowRate(altitude, throttle);
                    if (massFlowRate > 0 && state.propellantRemaining[state.currentStage] > 0) {
                        const timeToBurnout = state.propellantRemaining[state.currentStage] / massFlowRate;
                        totalTime = Math.max(timeToOrbit, timeToBurnout);
                    }
                }
                events.push({ time: totalTime, name: 'Orbit' });
            }
        }
    }
    
    // SECO (when stage 1 propellant runs out)
    if (state.currentStage === 1 && state.propellantRemaining[1] > 0 && state.engineOn) {
        const throttle = state.guidanceThrottle !== undefined ? state.guidanceThrottle : 1.0;
        const massFlowRate = getMassFlowRate(altitude, throttle);
        if (massFlowRate > 0) {
            const timeToSECO = state.propellantRemaining[1] / massFlowRate;
            if (timeToSECO > 0 && timeToSECO < 10000) {
                events.push({ time: timeToSECO, name: 'SECO' });
            }
        }
    }
    
    // Add burn events (circularization and retrograde)
    const burnEvents = calculateBurnEvents();
    events.push(...burnEvents);
    
    // Return the event with the shortest time
    if (events.length > 0) {
        events.sort((a, b) => a.time - b.time);
        return events[0];
    }
    return null;
}

