use crate::coordinates::{ecef_distance, EcefPoint, GeodeticPoint, SPEED_OF_LIGHT};
use dashmap::DashMap;
use log::{debug, info, warn};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::{Duration, Instant};

const CP_SIZE: usize = 32;
const DRIFT_N_STABLE: i32 = 12;
const MAX_PAIRING_AGE_SECS: f64 = 180.0;

#[derive(Debug, Clone)]
pub struct Clock {
    pub freq: f64,
    pub max_freq_error: f64,
    pub jitter: f64,
    pub delay_factor: f64,
}

impl Default for Clock {
    fn default() -> Self {
        Self {
            freq: 12e6,
            max_freq_error: 200e-6,
            jitter: 500e-9,
            delay_factor: 12e6 / SPEED_OF_LIGHT,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ClockPairing {
    pub base_user: String,
    pub peer_user: String,
    pub valid: bool,
    pub updated: Instant,
    pub variance: f64,
    pub error: f64,
    pub n: usize,
    pub drift_n: i32,
    pub raw_drift: f64,
    pub drift: f64,
    pub i_drift: f64,
    pub drift_outliers: i32,
    pub outliers: i32,
    pub jumped: i32,
    pub outlier_reset_cooldown: i32,
    pub outlier_total: f64,
    pub update_total: f64,

    pub factor: f64,
    pub i_factor: f64,
    pub base_avg: f64,
    pub peer_avg: f64,

    pub relative_freq: f64,
    pub i_relative_freq: f64,
    pub drift_max: f64,
    pub drift_max_delta: f64,
    pub outlier_threshold: f64,
    pub cumulative_error: f64,

    pub ts_base: [f64; CP_SIZE],
    pub ts_peer: [f64; CP_SIZE],
    pub var: [f64; CP_SIZE],
    pub var_sum: f64,
}

impl ClockPairing {
    pub fn new(base: String, peer: String) -> Self {
        let clock = Clock::default();
        let drift_max = 2.0 * (clock.max_freq_error + clock.max_freq_error);
        let drift_max_delta = drift_max / 10.0;
        let outlier_threshold = 50.0e-6; // Tolleranza realistica per internet/SDR (50 microsecondi)

        Self {
            base_user: base,
            peer_user: peer,
            valid: false,
            updated: Instant::now(),
            variance: 1e-12,
            error: 1e-6,
            n: 0,
            drift_n: 0,
            raw_drift: 0.0,
            drift: 0.0,
            i_drift: 0.0,
            drift_outliers: 0,
            outliers: 0,
            jumped: 0,
            outlier_reset_cooldown: 0,
            outlier_total: 0.0,
            update_total: 1e-3,

            factor: 1.0,
            i_factor: 1.0,
            base_avg: 0.0,
            peer_avg: 0.0,

            relative_freq: 1.0,
            i_relative_freq: 1.0,
            drift_max,
            drift_max_delta,
            outlier_threshold,
            cumulative_error: 0.0,

            ts_base: [0.0; CP_SIZE],
            ts_peer: [0.0; CP_SIZE],
            var: [0.0; CP_SIZE],
            var_sum: 0.0,
        }
    }

    pub fn predict_peer(&self, base_ts: f64) -> f64 {
        if self.n == 0 {
            return base_ts;
        }
        let last_base = self.ts_base[self.n - 1];
        let last_peer = self.ts_peer[self.n - 1];
        last_peer + (base_ts - last_base) * self.factor
    }

    pub fn predict_base(&self, peer_ts: f64) -> f64 {
        if self.n == 0 {
            return peer_ts;
        }
        let last_base = self.ts_base[self.n - 1];
        let last_peer = self.ts_peer[self.n - 1];
        last_base + (peer_ts - last_peer) * self.i_factor
    }

    pub fn check_valid(&mut self) -> bool {
        if self.n < 2 {
            self.valid = false;
            return false;
        }

        self.variance = (self.var_sum / (self.n as f64)).max(1e-16);
        self.error = self.variance.sqrt();

        // Considera valido se ha almeno 2 campioni recenti
        self.valid = self.n >= 2
            && self.updated.elapsed().as_secs_f64() < MAX_PAIRING_AGE_SECS;

        self.valid
    }

    pub fn reset_offsets(&mut self) {
        self.valid = false;
        self.n = 0;
        self.var_sum = 0.0;
    }

    pub fn update(
        &mut self,
        base_ts: f64,
        peer_ts: f64,
        base_interval: f64,
        peer_interval: f64,
    ) -> bool {
        let peer_freq = 12e6;

        if self.n >= CP_SIZE - 1 {
            self.prune_old_data();
        }

        self.update_total += 1.0;

        let mut prediction_error = 0.0;
        if self.n > 0 {
            let prediction = self.predict_peer(base_ts);
            prediction_error = (prediction - peer_ts) / peer_freq;
        }

        let abs_error = prediction_error.abs();
        if abs_error > self.outlier_threshold * 4.0 {
            self.outlier_total += 1.0;
            self.outliers += 1;
            if self.outliers >= 3 || (self.update_total > 5.0 && self.outlier_total / self.update_total > 0.80) {
                self.reset_offsets();
                self.outliers = 0;
            }
            return false;
        } else {
            self.outliers = 0;
        }

        self.cumulative_error = (self.cumulative_error + prediction_error).clamp(-100e-6, 100e-6);

        let _ = self.update_drift(base_interval, peer_interval);

        self.factor = self.relative_freq * (1.0 + self.drift);
        self.i_factor = self.i_relative_freq * (1.0 + self.i_drift);

        self.update_offset(base_ts, peer_ts, prediction_error);
        self.updated = Instant::now();
        self.check_valid();
        true
    }

    fn update_drift(&mut self, base_interval: f64, peer_interval: f64) -> bool {
        if base_interval <= 0.0 || peer_interval <= 0.0 {
            return false;
        }
        let adjusted_base_interval = base_interval * self.relative_freq;
        let new_drift = (peer_interval - adjusted_base_interval) / adjusted_base_interval;

        if new_drift.abs() > self.drift_max {
            return false;
        }

        if self.drift_n <= 0 {
            self.raw_drift = new_drift;
            self.drift = new_drift;
            self.i_drift = -1.0 * self.drift / (1.0 + self.drift);
            self.drift_n = 1;
            return true;
        }

        let kp = 0.08;
        let ki = 0.005;

        self.drift_n += 1;
        self.raw_drift += (new_drift - self.raw_drift) * kp;
        self.drift = self.raw_drift - ki * self.cumulative_error;
        self.i_drift = -1.0 * self.drift / (1.0 + self.drift);
        true
    }

    fn update_offset(&mut self, base_ts: f64, peer_ts: f64, prediction_error: f64) {
        let var = prediction_error * prediction_error;

        if self.n < CP_SIZE {
            self.ts_base[self.n] = base_ts;
            self.ts_peer[self.n] = peer_ts;
            self.var[self.n] = var;
            self.var_sum += var;
            self.n += 1;
        } else {
            self.var_sum -= self.var[0];
            for i in 0..(CP_SIZE - 1) {
                self.ts_base[i] = self.ts_base[i + 1];
                self.ts_peer[i] = self.ts_peer[i + 1];
                self.var[i] = self.var[i + 1];
            }
            self.ts_base[CP_SIZE - 1] = base_ts;
            self.ts_peer[CP_SIZE - 1] = peer_ts;
            self.var[CP_SIZE - 1] = var;
            self.var_sum += var;
        }
    }

    fn prune_old_data(&mut self) {
        if self.n < 8 {
            return;
        }
        let keep = self.n - 4;
        let shift = self.n - keep;
        for i in 0..keep {
            self.ts_base[i] = self.ts_base[i + shift];
            self.ts_peer[i] = self.ts_peer[i + shift];
            self.var[i] = self.var[i + shift];
        }
        self.n = keep;
        self.var_sum = 0.0;
        for i in 0..self.n {
            self.var_sum += self.var[i];
        }
    }
}

pub struct ClockSyncGraph {
    pairings: Arc<DashMap<(String, String), ClockPairing>>,
    receiver_positions: Arc<DashMap<String, (EcefPoint, GeodeticPoint)>>,
}

impl Default for ClockSyncGraph {
    fn default() -> Self {
        Self {
            pairings: Arc::new(DashMap::new()),
            receiver_positions: Arc::new(DashMap::new()),
        }
    }
}

impl ClockSyncGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_receiver_position(&self, user: String, ecef: EcefPoint, geo: GeodeticPoint) {
        self.receiver_positions.insert(user, (ecef, geo));
    }

    pub fn get_receiver_position(&self, user: &str) -> Option<(EcefPoint, GeodeticPoint)> {
        self.receiver_positions.get(user).map(|r| *r)
    }

    pub fn remove_receiver(&self, user: &str) {
        self.receiver_positions.remove(user);
        self.pairings.retain(|(u1, u2), _| u1 != user && u2 != user);
    }

    pub fn cleanup_stale(&self) {
        let now = Instant::now();
        self.pairings.retain(|_, pairing| {
            now.duration_since(pairing.updated) < Duration::from_secs(300)
        });
    }

    pub fn update_pairing(
        &self,
        u_base: &str,
        u_peer: &str,
        base_ts: f64,
        peer_ts: f64,
        base_interval: f64,
        peer_interval: f64,
    ) {
        if u_base == u_peer {
            return;
        }
        let key = if u_base < u_peer {
            (u_base.to_string(), u_peer.to_string())
        } else {
            (u_peer.to_string(), u_base.to_string())
        };

        let mut entry = self.pairings.entry(key.clone()).or_insert_with(|| {
            ClockPairing::new(key.0.clone(), key.1.clone())
        });

        if u_base < u_peer {
            entry.update(base_ts, peer_ts, base_interval, peer_interval);
        } else {
            entry.update(peer_ts, base_ts, peer_interval, base_interval);
        }
    }

    pub fn synchronize_observations(
        &self,
        observations: &[(String, f64)],
    ) -> Option<(String, Vec<(String, f64)>)> {
        if observations.len() < 3 {
            return None;
        }

        for candidate_idx in 0..observations.len() {
            let root = &observations[candidate_idx].0;
            let mut synchronized = Vec::with_capacity(observations.len());
            synchronized.push((root.clone(), observations[candidate_idx].1));

            for (i, (peer, peer_ts)) in observations.iter().enumerate() {
                if i == candidate_idx {
                    continue;
                }
                if let Some(t_in_root) = self.convert_clock(peer, *peer_ts, root) {
                    synchronized.push((peer.clone(), t_in_root));
                }
            }

            if synchronized.len() >= 3 {
                return Some((root.clone(), synchronized));
            }
        }

        None
    }

    pub fn convert_clock(&self, src: &str, t_src: f64, dst: &str) -> Option<f64> {
        if src == dst {
            return Some(t_src);
        }

        let key = if src < dst {
            (src.to_string(), dst.to_string())
        } else {
            (dst.to_string(), src.to_string())
        };

        if let Some(pairing) = self.pairings.get(&key) {
            if pairing.valid && pairing.updated.elapsed().as_secs_f64() < MAX_PAIRING_AGE_SECS {
                return if src < dst {
                    Some(pairing.predict_peer(t_src))
                } else {
                    Some(pairing.predict_base(t_src))
                };
            }
        }

        for entry in self.pairings.iter() {
            let (u1, u2) = entry.key();
            let p = entry.value();
            if !p.valid || p.updated.elapsed().as_secs_f64() >= MAX_PAIRING_AGE_SECS {
                continue;
            }

            let mid = if u1 == src {
                u2.as_str()
            } else if u2 == src {
                u1.as_str()
            } else {
                continue;
            };

            let t_mid = if u1 == src { p.predict_peer(t_src) } else { p.predict_base(t_src) };

            let key_dst = if mid < dst {
                (mid.to_string(), dst.to_string())
            } else {
                (dst.to_string(), mid.to_string())
            };

            if let Some(p2) = self.pairings.get(&key_dst) {
                if p2.valid && p2.updated.elapsed().as_secs_f64() < MAX_PAIRING_AGE_SECS {
                    let t_final = if mid < dst { p2.predict_peer(t_mid) } else { p2.predict_base(t_mid) };
                    return Some(t_final);
                }
            }
        }

        None
    }

    pub fn get_sync_peer_count(&self, user: &str) -> usize {
        let mut count = 0;
        for entry in self.pairings.iter() {
            let (u1, u2) = entry.key();
            if u1 == user || u2 == user {
                let p = entry.value();
                if p.valid && p.updated.elapsed().as_secs_f64() < MAX_PAIRING_AGE_SECS {
                    count += 1;
                }
            }
        }
        count
    }

    pub fn export_sync_map(&self) -> HashMap<String, HashMap<String, serde_json::Value>> {
        let mut map: HashMap<String, HashMap<String, serde_json::Value>> = HashMap::new();

        for entry in self.pairings.iter() {
            let (u1, u2) = entry.key();
            let p = entry.value();
            if p.n < 2 {
                continue;
            }

            let outlier_pct = if p.update_total < 4.0 {
                50.0 * p.outlier_total / p.update_total
            } else {
                100.0 * p.outlier_total / p.update_total
            };

            let age_s = p.updated.elapsed().as_secs() as i64;
            let p_val1 = serde_json::json!([
                p.n,
                (p.error * 1e6 * 10.0).round() / 10.0,
                (p.drift * 1e6).round(),
                0.0,
                p.jumped,
                (outlier_pct * 10.0).round() / 10.0,
                age_s,
                self.get_sync_peer_count(u2)
            ]);

            let p_val2 = serde_json::json!([
                p.n,
                (p.error * 1e6 * 10.0).round() / 10.0,
                (p.i_drift * 1e6).round(),
                0.0,
                p.jumped,
                (outlier_pct * 10.0).round() / 10.0,
                age_s,
                self.get_sync_peer_count(u1)
            ]);

            map.entry(u1.clone()).or_default().insert(u2.clone(), p_val1);
            map.entry(u2.clone()).or_default().insert(u1.clone(), p_val2);
        }
        map
    }
}
