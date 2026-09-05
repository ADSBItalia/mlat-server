use crate::coordinates::{
    ecef2llh, ecef_distance, ecef_vel_to_track_speed, EcefPoint, GeodeticPoint,
};
use dashmap::DashMap;
use flate2::read::GzDecoder;
use log::info;
use parking_lot::RwLock;
use std::collections::HashSet;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct TrackFilter {
    pub pos_ecef: EcefPoint,
    pub vel_ecef: (f64, f64, f64), // m/s in ECEF
    pub geo: GeodeticPoint,
    pub track_deg: Option<f32>,
    pub speed_kts: Option<f32>,
    pub last_update: Instant,
    pub last_sbs_emission: Instant,
    pub hits: usize,
    pub consecutive_rejects: usize,
    pub anchor_pos: EcefPoint,
    pub anchor_time: Instant,
}

#[derive(Debug, Clone)]
pub struct TrackedAircraft {
    pub icao: u32,
    pub filter: Option<TrackFilter>,
    pub altitude_ft: Option<i32>,
    pub last_altitude_time: Option<Instant>,
    pub vertical_rate_fpm: Option<f32>,
    pub last_seen: Instant,
    pub last_adsb_time: Option<Instant>,
    pub tracking_receivers: HashSet<Arc<str>>,
}

impl TrackedAircraft {
    pub fn new(icao: u32) -> Self {
        let now = Instant::now();
        Self {
            icao,
            filter: None,
            altitude_ft: None,
            last_altitude_time: None,
            vertical_rate_fpm: None,
            last_seen: now,
            last_adsb_time: None,
            tracking_receivers: HashSet::new(),
        }
    }
}

pub struct AircraftTracker {
    aircraft: Arc<DashMap<u32, TrackedAircraft>>,
    receiver_positions: Arc<DashMap<String, (EcefPoint, GeodeticPoint)>>,
    receiver_tracking: Arc<DashMap<String, HashSet<u32>>>,
    fixed_ground_beacons: Arc<RwLock<HashSet<u32>>>,
}

impl Default for AircraftTracker {
    fn default() -> Self {
        let fixed = Self::load_fixed_beacons();
        Self {
            aircraft: Arc::new(DashMap::new()),
            receiver_positions: Arc::new(DashMap::new()),
            receiver_tracking: Arc::new(DashMap::new()),
            fixed_ground_beacons: Arc::new(RwLock::new(fixed)),
        }
    }
}

impl AircraftTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Load known fixed ground stations, airport towers (TWR), ground test transponders (GND),
    /// radar Site Status Monitors (SSM), and masts to permanently exclude them from MLAT.
    pub fn load_fixed_beacons() -> HashSet<u32> {
        let mut set = HashSet::new();
        let paths = [
            "/usr/local/share/tar1090/aircraft.csv.gz",
            "/var/lib/adsbitalia-tar1090-db/aircraft.csv.gz",
        ];
        for path in &paths {
            if let Ok(file) = File::open(path) {
                let gz = GzDecoder::new(file);
                let reader = BufReader::new(gz);
                for line in reader.lines().filter_map(|l| l.ok()) {
                    let mut parts = line.split(';');
                    if let (Some(hex_str), Some(reg), Some(type_code)) = (parts.next(), parts.next(), parts.next()) {
                        let t_up = type_code.to_ascii_uppercase();
                        let reg_up = reg.to_ascii_uppercase();
                        if matches!(t_up.as_str(), "TWR" | "GND" | "OBST" | "MAST" | "RADAR")
                            || matches!(reg_up.as_str(), "TWR" | "GND")
                            || reg_up.starts_with("SSM")
                        {
                            if let Ok(icao) = u32::from_str_radix(hex_str.trim(), 16) {
                                set.insert(icao);
                            }
                        }
                    }
                }
                break;
            }
        }
        // Custom blacklist file support
        if let Ok(file) = File::open("/var/lib/mlat-server/blacklist.txt") {
            let reader = BufReader::new(file);
            for line in reader.lines().filter_map(|l| l.ok()) {
                let s = line.trim();
                if !s.is_empty() && !s.starts_with('#') {
                    if let Ok(icao) = u32::from_str_radix(s, 16) {
                        set.insert(icao);
                    }
                }
            }
        }
        // Ensure known Sydney SSM2 radar calibration monitor is blocked
        set.insert(0x7CF7CB);
        info!("[MLAT-TRACKER] Loaded {} fixed ground beacons/towers to permanently exclude from MLAT.", set.len());
        set
    }

    #[inline]
    pub fn is_fixed_beacon(&self, icao: u32) -> bool {
        self.fixed_ground_beacons.read().contains(&icao)
    }

    pub fn set_receiver_position(&self, user: String, ecef: EcefPoint, geo: GeodeticPoint) {
        self.receiver_positions.insert(user.clone(), (ecef, geo));
        self.receiver_tracking.entry(user).or_insert_with(HashSet::new);
    }

    pub fn remove_receiver(&self, user: &str) {
        self.receiver_positions.remove(user);
        self.receiver_tracking.remove(user);
        for mut ac in self.aircraft.iter_mut() {
            ac.tracking_receivers.retain(|u| &**u != user);
        }
    }

    pub fn record_add(&self, icao: u32, user: &Arc<str>) {
        if self.is_fixed_beacon(icao) {
            return;
        }
        let now = Instant::now();
        {
            let mut entry = self.aircraft.entry(icao).or_insert_with(|| TrackedAircraft::new(icao));
            entry.last_seen = now;
            entry.tracking_receivers.insert(Arc::clone(user));
        }

        if let Some(mut set) = self.receiver_tracking.get_mut(&**user) {
            set.insert(icao);
        }
    }

    pub fn mark_adsb_seen(&self, icao: u32) {
        let now = Instant::now();
        let mut entry = self.aircraft.entry(icao).or_insert_with(|| TrackedAircraft::new(icao));
        entry.last_seen = now;
        entry.last_adsb_time = Some(now);
    }

    pub fn mark_mlat_candidate(&self, icao: u32) {
        if let Some(mut entry) = self.aircraft.get_mut(&icao) {
            entry.last_adsb_time = None;
        }
    }

    pub fn record_seen(&self, icao: u32, user: &Arc<str>) {
        if self.is_fixed_beacon(icao) {
            return;
        }
        let now = Instant::now();
        {
            let mut entry = self.aircraft.entry(icao).or_insert_with(|| TrackedAircraft::new(icao));
            entry.last_seen = now;
            entry.tracking_receivers.insert(Arc::clone(user));
        }
        if let Some(mut set) = self.receiver_tracking.get_mut(&**user) {
            set.insert(icao);
        }
    }

    pub fn is_mlat_candidate(&self, icao: u32) -> bool {
        if self.is_fixed_beacon(icao) {
            return false;
        }
        if let Some(ac) = self.aircraft.get(&icao) {
            if let Some(t) = ac.last_adsb_time {
                if t.elapsed() < Duration::from_secs(60) {
                    return false;
                }
            }
        }
        true
    }

    pub fn get_receiver_candidate_icaos(&self, user: &str) -> Vec<String> {
        let icaos: Vec<u32> = if let Some(set) = self.receiver_tracking.get(user) {
            set.iter().copied().collect()
        } else {
            Vec::new()
        };

        let mut mlat_candidates = Vec::new();
        let mut sync_candidates = Vec::new();

        for icao in icaos {
            if self.is_fixed_beacon(icao) {
                continue;
            }
            let seen_count = self.aircraft.get(&icao).map_or(1, |ac| ac.tracking_receivers.len());
            if self.is_mlat_candidate(icao) {
                // Request MLAT if seen by 2 or more receivers or already tracked
                if seen_count >= 2 || self.aircraft.get(&icao).map_or(false, |ac| ac.filter.is_some()) {
                    mlat_candidates.push(format!("{:06x}", icao));
                }
            } else if sync_candidates.len() < 16 {
                sync_candidates.push(format!("{:06x}", icao));
            }
        }
        mlat_candidates.extend(sync_candidates);
        mlat_candidates
    }

    pub fn has_recent_position(&self, icao: u32, max_age: Duration) -> bool {
        self.aircraft.get(&icao).map_or(false, |ac| {
            ac.filter.as_ref().map_or(false, |f| f.last_update.elapsed() <= max_age)
        })
    }

    pub fn get_last_ecef(&self, icao: u32) -> Option<EcefPoint> {
        self.aircraft.get(&icao).and_then(|ac| ac.filter.as_ref().map(|f| f.pos_ecef))
    }

    /// Speed sanity check, hyperbolic mirror rejection & GDOP-adaptive alpha-beta trajectory smoothing.
    /// Returns Some((smoothed_geo, track_deg, speed_kts)) on success, or None if rejected.
    pub fn update_mlat_position(
        &self,
        icao: u32,
        sol_ecef: EcefPoint,
        sol_geo: GeodeticPoint,
        receiver_count: usize,
        gdop: f64,
        residual_rms: f64,
    ) -> Option<(GeodeticPoint, Option<f32>, Option<f32>, Option<i32>)> {
        let mut entry = self.aircraft.entry(icao).or_insert_with(|| TrackedAircraft::new(icao));
        let now = Instant::now();

        // 1. Check if we have an active, recent track filter
        let is_active = entry.filter.as_ref().map_or(false, |f| {
            f.last_update.elapsed() <= Duration::from_secs(45)
        });

        if !is_active {
            // New track acquisition or re-acquisition after signal drop:
            if receiver_count < 3 {
                return None;
            }
            // For 3-station fixes on initial acquisition, require tight GDOP (<= 6.5)
            // to avoid seeding on a false hyperbolic mirror branch!
            if receiver_count == 3 && gdop > 6.5 {
                return None;
            }

            let filter = TrackFilter {
                pos_ecef: sol_ecef,
                vel_ecef: (0.0, 0.0, 0.0),
                geo: sol_geo,
                track_deg: None,
                speed_kts: None,
                last_update: now,
                last_sbs_emission: now,
                hits: 1, // Pending correlation
                consecutive_rejects: 0,
                anchor_pos: sol_ecef,
                anchor_time: now,
            };
            entry.filter = Some(filter);
            // Require at least 2 correlated observations before painting on map.
            // Eliminates single-ping glitches and stray phantom targets!
            return None;
        }

        let cached_baro_vrate = entry.vertical_rate_fpm;
        let filter = entry.filter.as_mut().unwrap();
        let dt = filter.last_update.elapsed().as_secs_f64();

        // Avoid duplicate frames
        if dt < 0.08 {
            return None;
        }

        // Track correlation confirmation (hits == 1)
        if filter.hits < 2 {
            let total_dt = filter.anchor_time.elapsed().as_secs_f64();
            let dist_from_anchor = ecef_distance(&filter.anchor_pos, &sol_ecef);
            let max_phys = (340.0 * total_dt + 600.0).max(1_000.0);
            if dist_from_anchor > max_phys {
                // Reject impossible teleportation / hyperbolic mirror jump
                return None;
            }

            let dist_first = ecef_distance(&filter.pos_ecef, &sol_ecef);
            let max_first = (350.0 * dt + 400.0).max(500.0);
            if dist_first > max_first {
                // First point was an outlier, re-seed candidate fix ONLY within physical range of anchor
                filter.pos_ecef = sol_ecef;
                filter.geo = sol_geo;
                filter.last_update = now;
                return None;
            }
            // Correlated 2nd point! Initialize velocity vector
            let init_vx = (sol_ecef.x - filter.pos_ecef.x) / dt;
            let init_vy = (sol_ecef.y - filter.pos_ecef.y) / dt;
            let init_vz = (sol_ecef.z - filter.pos_ecef.z) / dt;
            filter.pos_ecef = sol_ecef;
            filter.vel_ecef = (init_vx, init_vy, init_vz);
            filter.geo = sol_geo;
            filter.hits = 2;
            filter.last_update = now;
            filter.last_sbs_emission = now;
            filter.anchor_pos = sol_ecef;
            filter.anchor_time = now;
            return Some((sol_geo, None, None, None));
        }

        // 2. Predict position using current velocity
        let pred_x = filter.pos_ecef.x + filter.vel_ecef.0 * dt;
        let pred_y = filter.pos_ecef.y + filter.vel_ecef.1 * dt;
        let pred_z = filter.pos_ecef.z + filter.vel_ecef.2 * dt;
        let pred_ecef = EcefPoint::new(pred_x, pred_y, pred_z);

        // Absolute physical speed barrier from last confirmed position
        // Enforces that an aircraft cannot exceed Mach 1.0 (340 m/s) relative to last confirmed fix.
        // Completely suppresses 15-20km hyperbolic mirror jumps when receivers are collinear!
        let dist_from_last = ecef_distance(&filter.pos_ecef, &sol_ecef);
        let max_phys_jump = (340.0 * dt + 600.0).max(1_000.0);
        if dist_from_last > max_phys_jump {
            filter.consecutive_rejects += 1;
            return None;
        }

        // 3-station gate: for established cruise, reject loose GDOP to prevent lateral wiggles
        if receiver_count == 3 && filter.hits >= 4 && gdop > 3.8 {
            return None;
        }

        // 3. Innovation distance check
        let gdop_scale = (gdop / 3.0).clamp(1.0, 1.6);
        let dist = ecef_distance(&pred_ecef, &sol_ecef);
        
        // 3-station fixes have zero mathematical redundancy: require tight innovation
        // to prevent single-receiver clock drift from pulling the track sideways!
        let max_allowed = if receiver_count == 3 {
            (220.0 * dt + 150.0).max(250.0)
        } else {
            (320.0 * dt + 220.0 * gdop_scale).max(350.0)
        };

        if dist > max_allowed {
            filter.consecutive_rejects += 1;
            
            // Maneuver recovery: ONLY allowed with 4+ stations, within 1,200 meters,
            // and persisting for 6 consecutive frames. 3-station fixes NEVER trigger maneuver recovery!
            if receiver_count >= 4 && dist < 1_200.0 && filter.consecutive_rejects >= 6 {
                filter.pos_ecef = sol_ecef;
                filter.vel_ecef = (0.0, 0.0, 0.0);
                filter.geo = sol_geo;
                filter.track_deg = None;
                filter.speed_kts = None;
                filter.last_update = now;
                filter.consecutive_rejects = 0;
                filter.hits = 2;
                filter.anchor_pos = sol_ecef;
                filter.anchor_time = now;
                return Some((sol_geo, None, None, None));
            }
            
            // If track was completely dark for > 25 seconds, reset state cleanly
            // ONLY if solution is within physical subsonic distance of last known fix!
            if dt > 25.0 && dist_from_last <= max_phys_jump {
                filter.pos_ecef = sol_ecef;
                filter.vel_ecef = (0.0, 0.0, 0.0);
                filter.geo = sol_geo;
                filter.track_deg = None;
                filter.speed_kts = None;
                filter.last_update = now;
                filter.consecutive_rejects = 0;
                filter.hits = 1;
                filter.anchor_pos = sol_ecef;
                filter.anchor_time = now;
                return None;
            }
            return None;
        }
        filter.consecutive_rejects = 0;

        // 4. GDOP-Adaptive Alpha-Beta Filter update (Smooth trajectory without zigzag)
        let rx = sol_ecef.x - pred_x;
        let ry = sol_ecef.y - pred_y;
        let rz = sol_ecef.z - pred_z;

        let (base_alpha, base_beta) = if filter.hits < 6 {
            (0.28, 0.05) // Acquisition
        } else {
            (0.16, 0.020) // Smooth cruising
        };

        // Dynamically weight by GDOP, stations and residual RMS:
        // On 3-station fixes, residual is mathematically unconstrained (zero degrees of freedom).
        // Heavily damp 3-station fixes during cruise to prevent lateral track wobbling!
        let (alpha, beta) = if receiver_count == 3 {
            if filter.hits >= 4 {
                (0.08, 0.008) // Cruise damping: locks track to straight heading
            } else {
                (0.16, 0.025)
            }
        } else {
            // 4+ stations: full geometric redundancy & RAIM verified
            let gdop_factor = (3.5 / gdop.clamp(1.0, 20.0)).clamp(0.4, 1.3);
            let rms_factor = (12.0 / residual_rms.clamp(4.0, 25.0)).clamp(0.5, 1.2);
            let quality_scale = (gdop_factor * rms_factor).clamp(0.30, 1.4);
            (
                (base_alpha * quality_scale).clamp(0.08, 0.40),
                (base_beta * quality_scale).clamp(0.01, 0.08),
            )
        };

        let new_x = pred_x + alpha * rx;
        let new_y = pred_y + alpha * ry;
        let new_z = pred_z + alpha * rz;
        let new_ecef = EcefPoint::new(new_x, new_y, new_z);

        // Physical acceleration limit: max 10.0 m/s^2 (~1g)
        let max_dv = (10.0 * dt).max(0.5);
        let raw_dv_x = beta * (rx / dt);
        let raw_dv_y = beta * (ry / dt);
        let raw_dv_z = beta * (rz / dt);

        let clamped_dv_x = raw_dv_x.clamp(-max_dv, max_dv);
        let clamped_dv_y = raw_dv_y.clamp(-max_dv, max_dv);
        let clamped_dv_z = raw_dv_z.clamp(-max_dv, max_dv);

        let mut new_vx = filter.vel_ecef.0 + clamped_dv_x;
        let mut new_vy = filter.vel_ecef.1 + clamped_dv_y;
        let mut new_vz = filter.vel_ecef.2 + clamped_dv_z;

        // Clamp velocity magnitude to max 380 m/s
        let v_mag = (new_vx * new_vx + new_vy * new_vy + new_vz * new_vz).sqrt();
        if v_mag > 380.0 {
            let scale = 380.0 / v_mag;
            new_vx *= scale;
            new_vy *= scale;
            new_vz *= scale;
        }

        let new_geo = ecef2llh(&new_ecef);
        let (raw_track_deg, speed_kts, v_up_fpm) = ecef_vel_to_track_speed(&new_geo, (new_vx, new_vy, new_vz));

        // 5. Angular track smoothing: only emit heading when track has accumulated enough hits and speed (>55 kts)
        let mut track_opt = if speed_kts > 55.0 && filter.hits >= 4 {
            if let Some(old_track) = filter.track_deg {
                let max_turn = (10.0 * dt as f32).max(2.0);
                Some(smooth_heading(old_track, raw_track_deg, max_turn))
            } else {
                Some(raw_track_deg)
            }
        } else {
            None
        };

        let mut speed_opt = if speed_kts > 55.0 && filter.hits >= 4 {
            Some(speed_kts)
        } else {
            None
        };

        // Stationary target detection (towers, surface transponders, parked aircraft):
        // If net displacement over >= 6 seconds is < 40 kts, zero out velocity and suppress speed/track
        let anchor_dt = filter.anchor_time.elapsed().as_secs_f64();
        if anchor_dt >= 6.0 {
            let net_disp = ecef_distance(&filter.anchor_pos, &new_ecef);
            let avg_kts = (net_disp / anchor_dt) * 1.94384;
            if avg_kts < 40.0 {
                new_vx = 0.0;
                new_vy = 0.0;
                new_vz = 0.0;
                track_opt = None;
                speed_opt = None;
            }
            filter.anchor_pos = new_ecef;
            filter.anchor_time = now;
        }

        // 6. SBS emission throttle: emit at most every 400ms to prevent jittering on maps
        let should_emit = filter.last_sbs_emission.elapsed() >= Duration::from_millis(400);
        if should_emit {
            filter.last_sbs_emission = now;
        }

        filter.pos_ecef = new_ecef;
        filter.vel_ecef = (new_vx, new_vy, new_vz);
        filter.geo = new_geo;
        filter.track_deg = track_opt;
        filter.speed_kts = speed_opt;
        filter.last_update = now;
        filter.hits += 1;

        // Vertical rate: ONLY emit when derived from genuine barometric altitude change.
        // Never use 3D TDoA vertical velocity which suffers from ground-station geometric dilution (GDOP).
        let vrate_opt = cached_baro_vrate.map(|baro_fpm| {
            if baro_fpm.abs() < 100.0 {
                0
            } else {
                ((baro_fpm / 64.0).round() as i32) * 64
            }
        });

        if should_emit {
            Some((new_geo, track_opt, speed_opt, vrate_opt))
        } else {
            None
        }
    }

    pub fn get_altitude(&self, icao: u32) -> Option<i32> {
        self.aircraft.get(&icao).and_then(|ac| ac.altitude_ft)
    }

    /// Interpolates or extrapolates the barometric altitude based on last reported altitude and vertical speed (fpm).
    /// Prevents stale altitude errors during climbs/descents while waiting for the next Mode-S altitude frame.
    pub fn get_interpolated_altitude(&self, icao: u32) -> Option<i32> {
        let ac = self.aircraft.get(&icao)?;
        let base_alt = ac.altitude_ft?;
        let last_time = ac.last_altitude_time?;
        let elapsed = last_time.elapsed().as_secs_f64();

        // If older than 60 seconds, barometric altitude is considered stale
        if elapsed > 60.0 {
            return None;
        }

        // If recent (< 2 sec) or no vertical rate calculated yet, return base altitude
        if elapsed < 2.0 || ac.vertical_rate_fpm.is_none() {
            return Some(base_alt);
        }

        if let Some(fpm) = ac.vertical_rate_fpm {
            let projected_delta = (fpm as f64) * (elapsed / 60.0);
            let projected_alt = (base_alt as f64) + projected_delta;
            Some(projected_alt.round() as i32)
        } else {
            Some(base_alt)
        }
    }

    pub fn update_altitude_baro(&self, icao: u32, alt_ft: i32) {
        let mut entry = self.aircraft.entry(icao).or_insert_with(|| TrackedAircraft::new(icao));
        let now = Instant::now();

        if let (Some(prev_alt), Some(prev_time)) = (entry.altitude_ft, entry.last_altitude_time) {
            let dt = prev_time.elapsed().as_secs_f64();
            if dt >= 0.5 && dt <= 25.0 {
                let d_alt = (alt_ft - prev_alt) as f64;
                let raw_fpm = (d_alt / dt) * 60.0;
                // Plausible vertical rate range: -8000 to +8000 fpm
                if raw_fpm.abs() < 8000.0 {
                    let new_fpm = if let Some(old_fpm) = entry.vertical_rate_fpm {
                        0.65 * old_fpm + 0.35 * (raw_fpm as f32)
                    } else {
                        raw_fpm as f32
                    };
                    entry.vertical_rate_fpm = Some(new_fpm);
                }
            }
        }

        entry.altitude_ft = Some(alt_ft);
        entry.last_altitude_time = Some(now);
    }

    pub fn cleanup_stale(&self) {
        self.aircraft.retain(|_, ac| ac.last_seen.elapsed() < Duration::from_secs(180));
        let active_icaos: HashSet<u32> = self.aircraft.iter().map(|kv| *kv.key()).collect();
        for mut set in self.receiver_tracking.iter_mut() {
            set.retain(|icao| active_icaos.contains(icao));
        }
    }
}

fn smooth_heading(old: f32, new: f32, max_change_deg: f32) -> f32 {
    let mut diff = (new - old) % 360.0;
    if diff > 180.0 {
        diff -= 360.0;
    } else if diff < -180.0 {
        diff += 360.0;
    }
    let clamped_diff = diff.clamp(-max_change_deg, max_change_deg);
    let mut res = (old + clamped_diff) % 360.0;
    if res < 0.0 {
        res += 360.0;
    }
    res
}
