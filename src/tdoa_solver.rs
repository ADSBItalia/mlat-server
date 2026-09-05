use crate::coordinates::{ecef_distance, ecef2llh, llh2ecef, EcefPoint, GeodeticPoint, SPEED_OF_LIGHT};
use nalgebra::{DMatrix, DVector};

#[derive(Debug, Clone)]
pub struct Measurement {
    pub receiver_ecef: EcefPoint,
    pub timestamp_sec: f64,
    pub variance: f64,
}

#[derive(Debug, Clone)]
pub struct SolverSolution {
    pub position_ecef: EcefPoint,
    pub position_geodetic: GeodeticPoint,
    pub offset_meters: f64,
    pub gdop: f64,
    pub residual_rms: f64,
}

pub struct ExactSolver;

impl Default for ExactSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl ExactSolver {
    pub fn new() -> Self {
        Self
    }

    /// Solves the TDOA system using Levenberg-Marquardt with RAIM (Receiver Autonomous Integrity Monitoring).
    /// If all measurements yield high residual or fail, and n >= 4 (with alt) or n >= 5 (without alt),
    /// it automatically performs Leave-One-Out subset pruning to isolate and eliminate noisy/jittery receivers.
    pub fn solve(
        &self,
        measurements: &[Measurement],
        altitude_m: Option<f64>,
        max_gdop: Option<f64>,
        initial_guess: EcefPoint,
    ) -> Option<SolverSolution> {
        let n = measurements.len();
        let has_alt = altitude_m.is_some();

        if n < 3 || (!has_alt && n < 4) {
            return None;
        }

        // 1. Try full measurement set first
        let full_sol = self.solve_raw(measurements, altitude_m, max_gdop, initial_guess);

        // If the full solution is good (RMS <= 8.0m), use it directly without subset pruning
        if let Some(ref sol) = full_sol {
            if sol.residual_rms <= 8.0 {
                return full_sol;
            }
        }

        // 2. RAIM Leave-One-Out Fault Exclusion:
        // Requires genuine redundancy in the remaining subset!
        // With altitude, 4 variables (x,y,z,dt) require at least 4 stations in the subset
        // (4 stations + 1 altitude = 5 equations, 1 degree of freedom).
        // Therefore, Leave-One-Out subset pruning is ONLY mathematically valid when n >= 5 (with alt) or n >= 6 (without alt)!
        // When n == 4 with altitude, the overdetermined 4-station solution is geometrically stable and must NEVER be pruned to 3!
        let min_subset_size = if has_alt { 4 } else { 5 };
        if n > min_subset_size {
            let mut best_subset_sol: Option<SolverSolution> = None;
            let mut best_rms = full_sol.as_ref().map_or(999.0, |s| s.residual_rms);

            for skip_idx in 0..n {
                let mut subset = Vec::with_capacity(n - 1);
                for (i, m) in measurements.iter().enumerate() {
                    if i != skip_idx {
                        subset.push(m.clone());
                    }
                }

                if let Some(sol) = self.solve_raw(&subset, altitude_m, max_gdop, initial_guess) {
                    if sol.residual_rms < best_rms {
                        best_rms = sol.residual_rms;
                        best_subset_sol = Some(sol);
                    }
                }
            }

            if let Some(sol) = best_subset_sol {
                // Prefer pruned subset if full solution failed or if subset improved RMS by >= 35%
                if full_sol.is_none() || sol.residual_rms < full_sol.as_ref().unwrap().residual_rms * 0.65 {
                    return Some(sol);
                }
            }
        }

        full_sol
    }

    /// Low-level Levenberg-Marquardt nonlinear least-squares solver
    pub fn solve_raw(
        &self,
        measurements: &[Measurement],
        altitude_m: Option<f64>,
        max_gdop: Option<f64>,
        initial_guess: EcefPoint,
    ) -> Option<SolverSolution> {
        let n = measurements.len();
        let has_alt = altitude_m.is_some();

        if n < 3 || (!has_alt && n < 4) {
            return None;
        }

        let base_ts = measurements[0].timestamp_sec;
        let pseudorange_data: Vec<(EcefPoint, f64, f64)> = measurements
            .iter()
            .map(|m| {
                let pr = (m.timestamp_sec - base_ts) * SPEED_OF_LIGHT;
                let err = (m.variance.sqrt() * SPEED_OF_LIGHT).max(15.0);
                (m.receiver_ecef, pr, err)
            })
            .collect();

        let mut guess_geo = ecef2llh(&initial_guess);
        if guess_geo.alt < -500.0 {
            guess_geo.alt = -500.0;
        }
        if guess_geo.alt > 20_000.0 {
            guess_geo.alt = 20_000.0;
        }
        let clamped_guess = llh2ecef(&guess_geo);

        let init_off = ecef_distance(&clamped_guess, &pseudorange_data[0].0);
        let mut x_state = DVector::from_vec(vec![clamped_guess.x, clamped_guess.y, clamped_guess.z, init_off]);
        let target_alt = altitude_m.unwrap_or(guess_geo.alt);
        let alt_err = 75.0; // 250 ft, matches Python MLAT altitude_error for pressure tolerance

        let mut lambda = 1e-2;
        let mut final_rms = 999.0;

        for _iter in 0..40 {
            let cur_pos = EcefPoint::new(x_state[0], x_state[1], x_state[2]);
            let cur_off = x_state[3];

            let m_rows = if has_alt { n + 1 } else { n };
            let mut r = DVector::zeros(m_rows);
            let mut j = DMatrix::zeros(m_rows, 4);
            let mut cost = 0.0;

            for (i, (rx_pos, measured_pr, err)) in pseudorange_data.iter().enumerate() {
                let dist = ecef_distance(&cur_pos, rx_pos).max(1.0);
                let pr_guess = dist - cur_off;
                let res = (*measured_pr - pr_guess) / err;
                r[i] = res;
                cost += res * res;

                let factor = 1.0 / (err * dist);
                j[(i, 0)] = -(cur_pos.x - rx_pos.x) * factor;
                j[(i, 1)] = -(cur_pos.y - rx_pos.y) * factor;
                j[(i, 2)] = -(cur_pos.z - rx_pos.z) * factor;
                j[(i, 3)] = 1.0 / err;
            }

            if has_alt {
                let geo_guess = ecef2llh(&cur_pos);
                let res_alt = (target_alt - geo_guess.alt) / alt_err;
                r[n] = res_alt;
                cost += res_alt * res_alt;

                let r_earth = 6371000.0;
                let norm = (cur_pos.x * cur_pos.x + cur_pos.y * cur_pos.y + cur_pos.z * cur_pos.z).sqrt().max(1.0);
                let f_alt = (norm - r_earth) / norm;
                j[(n, 0)] = (cur_pos.x * f_alt) / (alt_err * norm);
                j[(n, 1)] = (cur_pos.y * f_alt) / (alt_err * norm);
                j[(n, 2)] = (cur_pos.z * f_alt) / (alt_err * norm);
                j[(n, 3)] = 0.0;
            }

            final_rms = (cost / (m_rows as f64)).sqrt();

            let jt = j.transpose();
            let mut jtj = &jt * &j;
            let g = &jt * &r;

            for d in 0..4 {
                jtj[(d, d)] += lambda * (jtj[(d, d)].max(1e-2));
            }

            if let Some(delta) = jtj.lu().solve(&(-g)) {
                let step_norm = delta.norm();
                let step_limit = 50_000.0;
                let limited_delta = if step_norm > step_limit {
                    delta * (step_limit / step_norm)
                } else {
                    delta
                };

                let new_state = &x_state + &limited_delta;
                let new_pos = EcefPoint::new(new_state[0], new_state[1], new_state[2]);
                let new_off = new_state[3];

                let mut new_cost = 0.0;
                for (rx_pos, measured_pr, err) in &pseudorange_data {
                    let d = ecef_distance(&new_pos, rx_pos).max(1.0);
                    let pr_g = d - new_off;
                    let res = (*measured_pr - pr_g) / err;
                    new_cost += res * res;
                }
                if has_alt {
                    let geo_g = ecef2llh(&new_pos);
                    let res_a = (target_alt - geo_g.alt) / alt_err;
                    new_cost += res_a * res_a;
                }

                if new_cost < cost {
                    let cost_drop = cost - new_cost;
                    x_state = new_state;
                    lambda = (lambda * 0.5).max(1e-5);
                    if _iter >= 3 && (step_norm < 0.5 || cost_drop < 1e-4) {
                        break;
                    }
                } else {
                    lambda = (lambda * 3.0).min(1e5);
                }
            } else {
                lambda *= 4.0;
            }
        }

        let final_pos = EcefPoint::new(x_state[0], x_state[1], x_state[2]);
        let final_off = x_state[3];

        // 1. Physical validation: offset within realistic range
        if final_off < -5_000.0 || final_off > 600_000.0 {
            return None;
        }

        // 2. Radio horizon validation: receiving antennas within 550 km line of sight
        for (rx_pos, _, _) in &pseudorange_data {
            let dist = ecef_distance(&final_pos, rx_pos);
            if dist > 450_000.0 {
                return None;
            }
        }

        let final_geo = ecef2llh(&final_pos);
        if final_geo.lat < -85.0 || final_geo.lat > 85.0 || final_geo.alt < -500.0 || final_geo.alt > 25_000.0 {
            return None;
        }

        // 3. Pure geometric GDOP calculation (including altitude constraint when available)
        let m_gdop_rows = if has_alt { n + 1 } else { n };
        let mut h = DMatrix::zeros(m_gdop_rows, 4);
        for (i, (rx_pos, _, _)) in pseudorange_data.iter().enumerate() {
            let dist = ecef_distance(&final_pos, rx_pos).max(1.0);
            h[(i, 0)] = -(final_pos.x - rx_pos.x) / dist;
            h[(i, 1)] = -(final_pos.y - rx_pos.y) / dist;
            h[(i, 2)] = -(final_pos.z - rx_pos.z) / dist;
            h[(i, 3)] = 1.0;
        }

        if has_alt {
            let norm = (final_pos.x * final_pos.x + final_pos.y * final_pos.y + final_pos.z * final_pos.z).sqrt().max(1.0);
            h[(n, 0)] = final_pos.x / norm;
            h[(n, 1)] = final_pos.y / norm;
            h[(n, 2)] = final_pos.z / norm;
            h[(n, 3)] = 0.0;
        }

        let hth = h.transpose() * h;
        let real_gdop = match hth.pseudo_inverse(1e-6) {
            Ok(inv) => {
                let tr = inv.trace();
                if tr > 0.0 && !tr.is_nan() {
                    tr.sqrt()
                } else {
                    99.0
                }
            }
            Err(_) => 99.0,
        };

        // 4. GDOP cutoff to eliminate geometrically ambiguous solutions
        if real_gdop > max_gdop.unwrap_or(12.0) {
            return None;
        }

        // 5. Residual RMS cutoff: reject inconsistent solutions (bad clock/multipath)
        if final_rms > 18.0 {
            return None;
        }

        Some(SolverSolution {
            position_ecef: final_pos,
            position_geodetic: final_geo,
            offset_meters: final_off,
            gdop: real_gdop,
            residual_rms: final_rms,
        })
    }
}
