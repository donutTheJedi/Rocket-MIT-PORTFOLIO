// Physical constants
export const G = 6.67430e-11;
export const EARTH_MASS = 5.972e24;
export const EARTH_RADIUS = 6.371e6;
export const EARTH_ROTATION = 7.2921159e-5;
export const KARMAN_LINE = 100000;
export const ATM_SCALE_HEIGHT = 8500;
export const SEA_LEVEL_PRESSURE = 101325;
export const SEA_LEVEL_DENSITY = 1.225;

// Rocket configuration (Falcon 9-like)
export const ROCKET_CONFIG = {
    stages: [
        {
            name: "Stage 1",
            dryMass: 22200,
            propellantMass: 395700,
            thrust: 7607000,
            thrustVac: 8227000,
            isp: 282,
            ispVac: 311,
            diameter: 3.7,
            length: 47,
            dragCoeff: 0.3
        },
        {
            name: "Stage 2",
            dryMass: 4000,
            propellantMass: 92670,
            thrust: 981000,
            thrustVac: 981000,
            isp: 348,
            ispVac: 348,
            diameter: 3.7,
            length: 14,
            dragCoeff: 0.25
        }
    ],
    payload: 15000,
    fairingMass: 1700,
    fairingJettisonAlt: 110000,
    totalLength: 70
};

// ============================================================================
// CLOSED-LOOP GUIDANCE SYSTEM v3
// ============================================================================
// 
// V3 PHILOSOPHY: Simple, priority-based guidance that focuses on what matters
// when it matters. In atmosphere, we DON'T optimize for orbit - we escape.
// Above atmosphere, we optimize for target orbit.
//
// PRIORITY SYSTEM:
//   1. HEIGHT      — Get above 70km (out of significant atmosphere)
//   2. MAX Q       — Don't exceed structural limits, follow prograde in atmo
//   3. ANGLE       — Correct orbit shape (apoapsis/periapsis) - ONLY above 70km
//   4. VELOCITY    — Throttle down near end to hit target precisely
//
// KEY INSIGHT: Don't try to optimize trajectory in atmosphere.
// Get high, get fast, THEN correct.
//
// ATMOSPHERIC PHASE (below 70km):
//   - Follow prograde (minimizes angle of attack and drag)
//   - Smooth altitude-based minimum pitch constraint (prevents premature pitchover)
//   - Turn rate limiting (prevents gravity turn from running away)
//   - Max Q protection (follow prograde exactly when near structural limits)
//
// VACUUM PHASE (above 70km):
//   - Active closed-loop guidance to reach target orbit
//   - Adjusts pitch based on apoapsis/periapsis errors
//   - Throttle control for precise orbit insertion
//
// ============================================================================
export const GUIDANCE_CONFIG = {
    // Target orbit
    targetAltitude: 700000,          // meters — target circular orbit
    
    // Atmosphere threshold
    atmosphereLimit: 70000,          // meters — above this, we're in "vacuum"
    
    // Max Q protection
    maxQ: 35000,                     // Pa — typical max Q for Falcon 9 ~32-35 kPa
    
    // Pitch constraints
    maxPitchCorrection: 10,          // degrees — max deviation from prograde in vacuum
    maxPitchRate: 2.0,               // degrees/second — physical rotation limit
    
    // Throttle control
    throttleDownMargin: 1.15,        // Start throttling when deltaV reserve > 115% needed
    minThrottle: 0.4,                // Don't throttle below 40%
    
    // Initial ascent
    initialPitch: 85,                // degrees — slight eastward tilt from start
    pitchKickStart: 3,               // seconds — when to start pitching from vertical
    pitchKickEnd: 15,                // seconds — when to reach initialPitch
};

// Launch latitude (Cape Canaveral)
export const LAUNCH_LATITUDE = 28.5 * Math.PI / 180;

