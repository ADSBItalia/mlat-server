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
    pub is_locked_stationary: bool,
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
    pub last_confirmed_pos: Option<(EcefPoint, Instant)>,
    pub last_confirmed_vel: Option<(f64, f64, f64)>,
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
            last_confirmed_pos: None,
            last_confirmed_vel: None,
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

    /// Returns the best initial guess ECEF position for TDoA solving.
    /// Uses confirmed velocity vector to extrapolate trajectory for up to 180 seconds.
    pub fn get_last_ecef(&self, icao: u32) -> Option<EcefPoint> {
        self.aircraft.get(&icao).and_then(|ac| {
            if let Some((pos, time)) = ac.last_confirmed_pos {
                let dt = time.elapsed().as_secs_f64();
                if dt <= 180.0 {
                    if let Some(vel) = ac.last_confirmed_vel {
                        return Some(EcefPoint::new(
                            pos.x + vel.0 * dt,
                            pos.y + vel.1 * dt,
                            pos.z + vel.2 * dt,
                        ));
                    }
                    return Some(pos);
                }
            }
            ac.filter.as_ref().and_then(|f| {
                if f.last_update.elapsed() <= Duration::from_secs(60) {
                    Some(f.pos_ecef)
                } else {
                    None
                }
            })
        })
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

        // 1. Unconditional Global Physical Speed Barrier from last confirmed position
        // Completely eliminates any possibility of teleportation or mirror jumps across time gaps!
        if let Some((prev_pos, prev_time)) = entry.last_confirmed_pos {
            let dt = prev_time.elapsed().as_secs_f64();
            if dt <= 600.0 {
                let dist = ecef_distance(&prev_pos, &sol_ecef);
                // Max physical speed: 340 m/s (660 kts, ~Mach 1) + 1,200m measurement jitter allowance
                let max_allowed_dist = 340.0 * dt + 1200.0;
                if dist > max_allowed_dist {
                    // Physically impossible displacement: reject outlier/mirror root immediately!
                    if let Some(f) = entry.filter.as_mut() {
                        f.consecutive_rejects += 1;
                    }
                    return None;
                }
            }
        }

        // 2. Strict Station Count Requirement:
        // A 3-station solve has zero degrees of freedom and two symmetric mirror roots.
        // It CANNOT initialize or re-acquire a track!
        let has_confirmed_track = entry.filter.as_ref().map_or(false, |f| {
            f.hits >= 3 && f.last_update.elapsed() <= Duration::from_secs(30)
        });

        if receiver_count < 4 && !has_confirmed_track {
            // Reject 3-station solves on unconfirmed or stale tracks!
            return None;
        }

        // Tighter GDOP & RMS thresholds to discard bad geometries
        let max_gdop = if receiver_count == 3 { 3.8 } else { 4.6 };
        if gdop > max_gdop || residual_rms > 4.5 {
            return None;
        }

        // 3. Check if we have an active, recent track filter
        let is_active = entry.filter.as_ref().map_or(false, |f| {
            f.last_update.elapsed() <= Duration::from_secs(45)
        });

        if !is_active {
            // New track acquisition or re-acquisition after signal drop:
            // MUST HAVE AT LEAST 4 RECEIVERS!
            if receiver_count < 4 {
                return None;
            }

            let is_fixed = self.is_fixed_beacon(icao);
            let filter = TrackFilter {
                pos_ecef: sol_ecef,
                vel_ecef: (0.0, 0.0, 0.0),
                geo: sol_geo,
                track_deg: None,
                speed_kts: if is_fixed { Some(0.0) } else { None },
                last_update: now,
                last_sbs_emission: now,
                hits: 1, // Pending 2nd confirmation fix
                consecutive_rejects: 0,
                anchor_pos: sol_ecef,
                anchor_time: now,
                is_locked_stationary: is_fixed,
            };
            entry.filter = Some(filter);
            // Require at least 2 correlated observations before painting on map.
            return None;
        }

        let cached_baro_vrate = entry.vertical_rate_fpm;
        let filter = entry.filter.as_mut().unwrap();
        let dt = filter.last_update.elapsed().as_secs_f64();

        // Avoid duplicate frames
        if dt < 0.08 {
            return None;
        }

        // Stationary tower / ground beacon anchor:
        if filter.is_locked_stationary {
            let dist_from_anchor = ecef_distance(&filter.anchor_pos, &sol_ecef);
            if dist_from_anchor > 3500.0 {
                return None;
            }

            if filter.hits < 3 {
                let count = filter.hits as f64;
                let new_x = (filter.pos_ecef.x * count + sol_ecef.x) / (count + 1.0);
                let new_y = (filter.pos_ecef.y * count + sol_ecef.y) / (count + 1.0);
                let new_z = (filter.pos_ecef.z * count + sol_ecef.z) / (count + 1.0);
                filter.pos_ecef = EcefPoint::new(new_x, new_y, new_z);
                filter.geo = ecef2llh(&filter.pos_ecef);
                filter.anchor_pos = filter.pos_ecef;
                filter.hits += 1;
            }

            filter.last_update = now;

            if filter.last_sbs_emission.elapsed() >= Duration::from_secs(5) {
                filter.last_sbs_emission = now;
                return Some((filter.geo, None, Some(0.0), Some(0)));
            } else {
                return None;
            }
        }

        // Track correlation confirmation (hits < 2)
        if filter.hits < 2 {
            if receiver_count < 4 {
                return None;
            }
            if dt > 15.0 {
                // Too long since first hit, restart 1st hit
                filter.pos_ecef = sol_ecef;
                filter.geo = sol_geo;
                filter.last_update = now;
                return None;
            }

            let dist = ecef_distance(&filter.pos_ecef, &sol_ecef);
            let max_phys = 340.0 * dt + 500.0;
            if dist > max_phys {
                // Inconsistent with 1st hit, do not confirm, restart candidate
                filter.pos_ecef = sol_ecef;
                filter.geo = sol_geo;
                filter.last_update = now;
                return None;
            }

            // Successfully correlated 2nd fix!
            let raw_vx = (sol_ecef.x - filter.pos_ecef.x) / dt;
            let raw_vy = (sol_ecef.y - filter.pos_ecef.y) / dt;
            let raw_vz = (sol_ecef.z - filter.pos_ecef.z) / dt;
            let v_mag = (raw_vx * raw_vx + raw_vy * raw_vy + raw_vz * raw_vz).sqrt();
            let (init_vx, init_vy, init_vz) = if v_mag > 280.0 {
                let s = 280.0 / v_mag;
                (raw_vx * s, raw_vy * s, raw_vz * s)
            } else {
                (raw_vx, raw_vy, raw_vz)
            };

            filter.pos_ecef = sol_ecef;
            filter.vel_ecef = (init_vx, init_vy, init_vz);
            filter.geo = sol_geo;
            filter.hits = 2;
            filter.last_update = now;
            filter.last_sbs_emission = now;
            filter.anchor_pos = sol_ecef;
            filter.anchor_time = now;

            entry.last_confirmed_pos = Some((sol_ecef, now));
            entry.last_confirmed_vel = Some((init_vx, init_vy, init_vz));

            return Some((sol_geo, None, None, None));
        }

        // 4. Predict position using current velocity
        let pred_x = filter.pos_ecef.x + filter.vel_ecef.0 * dt;
        let pred_y = filter.pos_ecef.y + filter.vel_ecef.1 * dt;
        let pred_z = filter.pos_ecef.z + filter.vel_ecef.2 * dt;
        let pred_ecef = EcefPoint::new(pred_x, pred_y, pred_z);

        // Distance from predicted position
        let dist_pred = ecef_distance(&pred_ecef, &sol_ecef);

        // 3-station consistency gate:
        // When receiver_count == 3, MUST be strictly within predicted trajectory
        if receiver_count == 3 {
            let max_3stn_dev = (160.0 * dt + 200.0).min(500.0);
            if dist_pred > max_3stn_dev {
                filter.consecutive_rejects += 1;
                return None;
            }
        }

        // General innovation gate for 4+ stations
        let max_allowed = (270.0 * dt + 350.0).max(500.0);
        if dist_pred > max_allowed {
            filter.consecutive_rejects += 1;
            
            // Maneuver recovery: ONLY allowed with 4+ stations, clean GDOP (<= 3.5), within 2000m and 3 consecutive rejects
            if receiver_count >= 4 && gdop <= 3.5 && dist_pred < 2000.0 && filter.consecutive_rejects >= 3 {
                let raw_vx = (sol_ecef.x - filter.pos_ecef.x) / dt;
                let raw_vy = (sol_ecef.y - filter.pos_ecef.y) / dt;
                let raw_vz = (sol_ecef.z - filter.pos_ecef.z) / dt;
                let v_mag = (raw_vx * raw_vx + raw_vy * raw_vy + raw_vz * raw_vz).sqrt();
                let (init_vx, init_vy, init_vz) = if v_mag > 280.0 {
                    let s = 280.0 / v_mag;
                    (raw_vx * s, raw_vy * s, raw_vz * s)
                } else {
                    (raw_vx, raw_vy, raw_vz)
                };
                filter.pos_ecef = sol_ecef;
                filter.vel_ecef = (init_vx, init_vy, init_vz);
                filter.geo = sol_geo;
                filter.track_deg = None;
                filter.speed_kts = None;
                filter.last_update = now;
                filter.consecutive_rejects = 0;
                filter.hits = 3;
                filter.anchor_pos = sol_ecef;
                filter.anchor_time = now;
                entry.last_confirmed_pos = Some((sol_ecef, now));
                entry.last_confirmed_vel = Some((init_vx, init_vy, init_vz));
                return Some((sol_geo, None, None, None));
            }
            return None;
        }

        filter.consecutive_rejects = 0;

        // 5. GDOP-Adaptive Alpha-Beta Filter update (Smooth trajectory without zigzag)
        let rx = sol_ecef.x - pred_x;
        let ry = sol_ecef.y - pred_y;
        let rz = sol_ecef.z - pred_z;

        let (base_alpha, base_beta) = if filter.hits < 4 {
            (0.50, 0.20)
        } else {
            (0.25, 0.08)
        };

        let (alpha, beta) = if receiver_count == 3 {
            (0.10, 0.02) // Very gentle cruise tracking on 3 stations
        } else {
            let gdop_factor = (3.5 / gdop.clamp(1.0, 20.0)).clamp(0.6, 1.2);
            let rms_factor = (12.0 / residual_rms.clamp(4.0, 25.0)).clamp(0.7, 1.2);
            let quality_scale = (gdop_factor * rms_factor).clamp(0.50, 1.3);
            (
                (base_alpha * quality_scale).clamp(0.15, 0.35),
                (base_beta * quality_scale).clamp(0.04, 0.12),
            )
        };

        let new_x = pred_x + alpha * rx;
        let new_y = pred_y + alpha * ry;
        let new_z = pred_z + alpha * rz;
        let new_ecef = EcefPoint::new(new_x, new_y, new_z);

        // Physical acceleration limit: max 4.0 m/s^2 (~0.4g)
        let max_dv = (4.0 * dt).max(0.2);
        let raw_dv_x = beta * (rx / dt);
        let raw_dv_y = beta * (ry / dt);
        let raw_dv_z = beta * (rz / dt);

        let clamped_dv_x = raw_dv_x.clamp(-max_dv, max_dv);
        let clamped_dv_y = raw_dv_y.clamp(-max_dv, max_dv);
        let clamped_dv_z = raw_dv_z.clamp(-max_dv, max_dv);

        let mut new_vx = filter.vel_ecef.0 + clamped_dv_x;
        let mut new_vy = filter.vel_ecef.1 + clamped_dv_y;
        let mut new_vz = filter.vel_ecef.2 + clamped_dv_z;

        // Clamp velocity magnitude to max 280 m/s (544 kts)
        let v_mag = (new_vx * new_vx + new_vy * new_vy + new_vz * new_vz).sqrt();
        if v_mag > 280.0 {
            let scale = 280.0 / v_mag;
            new_vx *= scale;
            new_vy *= scale;
            new_vz *= scale;
        }

        let new_geo = ecef2llh(&new_ecef);
        let (raw_track_deg, speed_kts, _) = ecef_vel_to_track_speed(&new_geo, (new_vx, new_vy, new_vz));

        // Angular track smoothing: only emit heading when track has accumulated enough hits and speed (>55 kts)
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

        // Stationary target detection:
        let anchor_dt = filter.anchor_time.elapsed().as_secs_f64();
        if anchor_dt >= 6.0 {
            let net_disp = ecef_distance(&filter.anchor_pos, &new_ecef);
            let avg_kts = (net_disp / anchor_dt) * 1.94384;
            if new_geo.alt < 1500.0 && avg_kts < 40.0 {
                new_vx = 0.0;
                new_vy = 0.0;
                new_vz = 0.0;
                track_opt = None;
                speed_opt = None;
            }
            filter.anchor_pos = new_ecef;
            filter.anchor_time = now;
        }

        // SBS emission throttle: emit at most every 900ms
        let should_emit = filter.last_sbs_emission.elapsed() >= Duration::from_millis(900);
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

        entry.last_confirmed_pos = Some((new_ecef, now));
        entry.last_confirmed_vel = Some((new_vx, new_vy, new_vz));

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
