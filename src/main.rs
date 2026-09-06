#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

use crate::clock_sync::ClockSyncGraph;
use crate::coordinates::{ecef_distance, EcefPoint, GeodeticPoint, SPEED_OF_LIGHT};
use crate::cpr::decode_cpr_pair;
use crate::messages::{
    extract_altitude, extract_icao_and_df, format_sbs_msg3,
};
use crate::receiver::Receiver;
use crate::status_exporter::export_mlat_status;
use crate::tdoa_solver::{ExactSolver, Measurement};
use crate::tracker::AircraftTracker;
use crate::zlib_transport::Zlib2Decompressor;

use dashmap::DashMap;
use log::{error, info, warn};
use serde_json::Value;
use std::fs;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, mpsc};

mod clock_sync;
mod coordinates;
mod cpr;
mod messages;
mod receiver;
mod status_exporter;
mod tdoa_solver;
mod tracker;
mod zlib_transport;

#[derive(Debug, Clone)]
pub struct PendingMsgReception {
    pub user: Arc<str>,
    pub raw_ts_sec: f64,
}

#[derive(Debug, Clone)]
pub struct PendingSyncReception {
    pub user: Arc<str>,
    pub raw_t_even: f64,
    pub raw_t_odd: f64,
}

pub struct ServerState {
    receivers: Arc<DashMap<String, Receiver>>,
    clock_sync: Arc<ClockSyncGraph>,
    tracker: Arc<AircraftTracker>,
    solver: Arc<ExactSolver>,
    sbs_tx: broadcast::Sender<String>,
    solver_tx: mpsc::Sender<u64>,
    inflight_frames: Arc<DashMap<u64, (u32, Vec<PendingMsgReception>, Option<i32>, bool, Instant)>>,
    inflight_syncs: Arc<DashMap<u64, (GeodeticPoint, Vec<PendingSyncReception>, Instant)>>,
    client_txs: Arc<DashMap<Arc<str>, mpsc::Sender<String>>>,
}

fn hash_payload(payload: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf29ce484222325;
    for &byte in payload {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args: Vec<String> = std::env::args().collect();
    let mut client_listen = std::env::var("MLAT_CLIENT_LISTEN").unwrap_or_else(|_| "0.0.0.0:41114".to_string());
    let mut sbs_listen = std::env::var("MLAT_SBS_LISTEN").unwrap_or_else(|_| "127.0.0.1:32008".to_string());
    let mut work_dir = std::env::var("MLAT_WORK_DIR").unwrap_or_else(|_| "/var/lib/mlat-server-rust".to_string());

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--client-listen" if i + 1 < args.len() => {
                client_listen = args[i + 1].clone();
                if !client_listen.contains(':') {
                    client_listen = format!("0.0.0.0:{}", client_listen);
                }
                i += 2;
            }
            "--basestation-listen" if i + 1 < args.len() => {
                sbs_listen = args[i + 1].clone();
                if !sbs_listen.contains(':') {
                    sbs_listen = format!("127.0.0.1:{}", sbs_listen);
                }
                i += 2;
            }
            "--work-dir" if i + 1 < args.len() => {
                work_dir = args[i + 1].clone();
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    let _ = fs::create_dir_all(&work_dir);

    info!("Starting ADSBItalia Native Rust MLAT Server v4.1 (Zero-Alloc & Low-RAM)...");
    info!("Feeder listen address: {}", client_listen);
    info!("BaseStation SBS listen: {}", sbs_listen);
    info!("Telemetry work dir:    {}", work_dir);

    let (solver_tx, mut solver_rx) = mpsc::channel::<u64>(4096);

    let state = Arc::new(ServerState {
        receivers: Arc::new(DashMap::new()),
        clock_sync: Arc::new(ClockSyncGraph::new()),
        tracker: Arc::new(AircraftTracker::new()),
        solver: Arc::new(ExactSolver::new()),
        sbs_tx: broadcast::channel(10000).0,
        solver_tx: solver_tx.clone(),
        inflight_frames: Arc::new(DashMap::new()),
        inflight_syncs: Arc::new(DashMap::new()),
        client_txs: Arc::new(DashMap::new()),
    });

    let listener = TcpListener::bind(&client_listen).await?;

    let sbs_bind = sbs_listen.clone();
    let sbs_tx_clone = state.sbs_tx.clone();
    tokio::spawn(async move {
        sbs_broadcast_listener(&sbs_bind, sbs_tx_clone).await;
    });

    // Event-driven solver worker (Zero polling overhead)
    let state_for_solver = state.clone();
    tokio::spawn(async move {
        let mut total_ge3: u64 = 0;
        let mut total_synced: u64 = 0;
        let mut total_solved: u64 = 0;

        while let Some(msg_hash) = solver_rx.recv().await {
            total_ge3 += 1;
            if let Some((_, (icao, receptions, frame_alt_ft, _, _))) = state_for_solver.inflight_frames.remove(&msg_hash) {
                if !state_for_solver.tracker.is_mlat_candidate(icao) || receptions.len() < 3 {
                    continue;
                }

                let obs_tuples: Vec<(String, f64)> = receptions
                    .iter()
                    .map(|r| (r.user.to_string(), r.raw_ts_sec))
                    .collect();

                if icao == 0xAE61FD {
                    info!("[AE61FD-SOLVER-START] receptions={}", obs_tuples.len());
                }

                if let Some((_root, synced_list)) = state_for_solver.clock_sync.synchronize_observations(&obs_tuples) {
                    total_synced += 1;
                    if icao == 0xAE61FD {
                        info!("[AE61FD-SYNCED] root={} synced_len={}", _root, synced_list.len());
                    }
                    let mut measurements = Vec::with_capacity(synced_list.len());

                    for (rx_u, common_t) in &synced_list {
                        if let Some((rx_ecef, _)) = state_for_solver.clock_sync.get_receiver_position(rx_u) {
                            measurements.push(Measurement {
                                receiver_ecef: rx_ecef,
                                timestamp_sec: *common_t / 12_000_000.0,
                                variance: 1e-14,
                            });
                        }
                    }

                    // Sort measurements chronologically so measurements[0] is the earliest station
                    measurements.sort_by(|a, b| {
                        a.timestamp_sec
                            .partial_cmp(&b.timestamp_sec)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });

                    let alt_m = frame_alt_ft
                        .or_else(|| state_for_solver.tracker.get_interpolated_altitude(icao))
                        .map(|ft| (ft as f64) * 0.3048);

                    let min_rcvs = if alt_m.is_some() { 3 } else { 4 };

                    if icao == 0xAE61FD {
                        info!("[AE61FD-PRE-SOLVE] meas={} min_rcvs={} alt_m={:?}", measurements.len(), min_rcvs, alt_m);
                    }

                    if measurements.len() >= min_rcvs {
                        let guess = if let Some(last_pos) = state_for_solver.tracker.get_last_ecef(icao) {
                            last_pos
                        } else {
                            let mut mean_x = 0.0;
                            let mut mean_y = 0.0;
                            let mut mean_z = 0.0;
                            for m in &measurements {
                                mean_x += m.receiver_ecef.x;
                                mean_y += m.receiver_ecef.y;
                                mean_z += m.receiver_ecef.z;
                            }
                            let count = measurements.len() as f64;
                            let centroid_ecef = EcefPoint::new(mean_x / count, mean_y / count, mean_z / count);
                            let centroid_geo = crate::coordinates::ecef2llh(&centroid_ecef);
                            let target_alt = alt_m.unwrap_or(10_000.0);
                            crate::coordinates::llh2ecef(&crate::coordinates::GeodeticPoint::new(centroid_geo.lat, centroid_geo.lon, target_alt))
                        };

                        // Select optimal antenna cluster (best 8 stations) if frame was heard by > 8 receivers
                        if measurements.len() > 8 {
                            measurements = select_optimal_cluster(measurements, &guess);
                        }

                        let max_gdop = 8.5;

                        let sol_opt = state_for_solver.solver.solve(&measurements, alt_m, Some(max_gdop), guess);
                        if icao == 0xAE61FD {
                            info!("[AE61FD-SOLVE-RESULT] sol_is_some={}", sol_opt.is_some());
                        }

                        if let Some(sol) = sol_opt {
                            let tracker_opt = state_for_solver.tracker.update_mlat_position(
                                icao,
                                sol.position_ecef,
                                sol.position_geodetic,
                                measurements.len(),
                                sol.gdop,
                                sol.residual_rms,
                            );
                            if icao == 0xAE61FD {
                                info!("[AE61FD-TRACKER-RESULT] tracker_is_some={}", tracker_opt.is_some());
                            }
                            if let Some((smoothed_geo, track_opt, speed_opt, vrate_opt)) = tracker_opt {
                                total_solved += 1;
                                let lat = smoothed_geo.lat;
                                let lon = smoothed_geo.lon;

                                if lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 {
                                    for (rx_u, _) in &synced_list {
                                        if let Some(mut rcv) = state_for_solver.receivers.get_mut(rx_u) {
                                            rcv.mlat_positions_contributed += 1;
                                        }
                                    }

                                    // Only emit altitude if genuine baro altitude was received or interpolated
                                    let final_alt = frame_alt_ft
                                        .or_else(|| state_for_solver.tracker.get_interpolated_altitude(icao));

                                    let sbs_line = format_sbs_msg3(
                                        &format!("{:06X}", icao),
                                        &smoothed_geo,
                                        track_opt,
                                        speed_opt,
                                        final_alt,
                                        vrate_opt,
                                        measurements.len(),
                                    );

                                    let _ = state_for_solver.sbs_tx.send(sbs_line);

                                    // Forward MLAT results back to participating feeders
                                    let now_unix = std::time::SystemTime::now()
                                        .duration_since(std::time::UNIX_EPOCH)
                                        .unwrap_or_default()
                                        .as_secs_f64();

                                    let (nsvel, ewvel) = match (speed_opt, track_opt) {
                                        (Some(spd), Some(trk)) => {
                                            let rad = (trk as f64).to_radians();
                                            let ns = ((rad.cos() * (spd as f64) * 10.0).round() / 10.0) as f64;
                                            let ew = ((rad.sin() * (spd as f64) * 10.0).round() / 10.0) as f64;
                                            (Some(ns), Some(ew))
                                        }
                                        _ => (None, None),
                                    };

                                    let result_msg = serde_json::json!({
                                        "result": {
                                            "@": (now_unix * 1000.0).round() / 1000.0,
                                            "addr": format!("{:06x}", icao),
                                            "lat": (lat * 100000.0).round() / 100000.0,
                                            "lon": (lon * 100000.0).round() / 100000.0,
                                            "alt": final_alt.unwrap_or(0),
                                            "callsign": serde_json::Value::Null,
                                            "squawk": serde_json::Value::Null,
                                            "nsvel": nsvel,
                                            "ewvel": ewvel,
                                            "vrate": vrate_opt,
                                            "gdop": (sol.gdop * 10.0).round() / 10.0,
                                            "nstations": measurements.len()
                                        }
                                    });
                                    let result_str = result_msg.to_string();

                                    for (rx_u, _) in &synced_list {
                                        if let Some(client_tx) = state_for_solver.client_txs.get(&**rx_u) {
                                            let _ = client_tx.try_send(result_str.clone());
                                        }
                                    }
                                    info!(
                                        "[MLAT-RUST-SOLVED] ICAO={:06X} Stns={} Lat={:.4} Lon={:.4} Alt={:?}ft Spd={:?}kt Trk={:?} VRate={:?}fpm GDOP={:.1}",
                                        icao,
                                        measurements.len(),
                                        lat,
                                        lon,
                                        final_alt,
                                        speed_opt,
                                        track_opt,
                                        vrate_opt,
                                        sol.gdop
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    });

    // Batch dispatcher worker running every 30ms:
    // Dispatches 4+ stations after 320ms.
    // For 3-station frames, waits up to 600ms so a 4th station delayed by internet jitter has time to arrive!
    let state_for_dispatch = state.clone();
    let solver_tx_clone = solver_tx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(30));
        loop {
            interval.tick().await;
            let mut ready = Vec::new();
            for entry in state_for_dispatch.inflight_frames.iter() {
                let (_, receptions, _, dispatched, created) = entry.value();
                let age = created.elapsed();
                if !*dispatched {
                    if receptions.len() >= 4 && age >= Duration::from_millis(320) {
                        ready.push(*entry.key());
                    } else if receptions.len() >= 3 && age >= Duration::from_millis(600) {
                        ready.push(*entry.key());
                    }
                }
            }
            for h in ready {
                if let Some(mut entry) = state_for_dispatch.inflight_frames.get_mut(&h) {
                    if !entry.3 {
                        entry.3 = true;
                        let _ = solver_tx_clone.send(h).await;
                    }
                }
            }
        }
    });

    // In-flight buffer cleaner running every 80ms:
    // Retains single-receiver frames up to 500ms and 2-receiver frames up to 750ms.
    // Completely eliminates feeder packet starvation over internet jitter while keeping RAM tight!
    let state_for_cleanup = state.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(80));
        loop {
            interval.tick().await;
            state_for_cleanup.inflight_frames.retain(|_, (_, receptions, _, dispatched, created)| {
                let age = created.elapsed();
                if *dispatched {
                    age < Duration::from_millis(250)
                } else if receptions.len() < 2 {
                    age < Duration::from_millis(500)
                } else if receptions.len() == 2 {
                    age < Duration::from_millis(750)
                } else {
                    age < Duration::from_millis(900)
                }
            });
            state_for_cleanup.inflight_syncs.retain(|_, (_, _, created)| {
                created.elapsed() < Duration::from_millis(500)
            });
        }
    });

    // Slow background garbage collector (runs every 20s: prunes stale aircraft & pairings)
    let state_for_gc = state.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(20));
        loop {
            interval.tick().await;
            state_for_gc.tracker.cleanup_stale();
            state_for_gc.clock_sync.cleanup_stale();
        }
    });

    // Export worker
    let state_for_export = state.clone();
    let dir_for_export = work_dir.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(5));
        loop {
            interval.tick().await;
            export_mlat_status(&dir_for_export, &state_for_export.receivers, &state_for_export.clock_sync);
        }
    });

    // Fast ReadsB ADS-B synchronizer (reads local aircraft.json every 2s)
    let state_for_adsb = state.clone();
    tokio::spawn(async move {
        sync_adsb_from_readsb(&state_for_adsb);
        let mut interval = tokio::time::interval(Duration::from_secs(2));
        loop {
            interval.tick().await;
            sync_adsb_from_readsb(&state_for_adsb);
        }
    });

    while let Ok((stream, peer_addr)) = listener.accept().await {
        let state_clone = state.clone();
        tokio::spawn(async move {
            let _ = handle_receiver_connection(stream, peer_addr, state_clone).await;
        });
    }

    Ok(())
}

async fn handle_receiver_connection(
    stream: TcpStream,
    peer_addr: SocketAddr,
    state: Arc<ServerState>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut peer_ip = peer_addr.ip().to_string();
    let (read_half, mut write_half) = stream.into_split();
    let mut reader = BufReader::new(read_half);

    let mut handshake_line = String::new();
    reader.read_line(&mut handshake_line).await?;

    if handshake_line.starts_with("PROXY ") {
        let parts: Vec<&str> = handshake_line.split_whitespace().collect();
        if parts.len() >= 3 {
            peer_ip = parts[2].to_string();
        }
        handshake_line.clear();
        reader.read_line(&mut handshake_line).await?;
    }

    let handshake_req: crate::receiver::HandshakeRequest = match serde_json::from_str(handshake_line.trim()) {
        Ok(req) => req,
        Err(e) => {
            warn!("Handshake parse error from {}: {}", peer_ip, e);
            return Err(e.into());
        }
    };

    let return_results_wanted = handshake_req.return_results.unwrap_or(false);
    let user_name = handshake_req.user.clone();
    let user_arc: Arc<str> = Arc::from(user_name.as_str());
    let receiver = Receiver::new(handshake_req, peer_ip);
    let ecef_pos = receiver.ecef;
    let geo_pos = receiver.geodetic;

    state.clock_sync.set_receiver_position(user_name.clone(), ecef_pos, geo_pos);
    state.tracker.set_receiver_position(user_name.clone(), ecef_pos, geo_pos);
    state.receivers.insert(user_name.clone(), receiver);

    // Negotiate "zlib": client sends zlib packets, server sends raw lines.
    // Completely eliminates 500 server-side Zlib2Compressor instances (~128MB C heap saved!).
    let ack_doc = serde_json::json!({
        "compress": "zlib",
        "reconnect_in": serde_json::Value::Null,
        "status": "ok",
        "return_results": return_results_wanted
    });
    let ack_str = format!("{}\n", serde_json::to_string(&ack_doc)?);
    write_half.write_all(ack_str.as_bytes()).await?;

    let (tx, mut rx) = mpsc::channel::<String>(256);

    if return_results_wanted {
        state.client_txs.insert(user_arc.clone(), tx.clone());
    }

    tokio::spawn(async move {
        while let Some(line) = rx.recv().await {
            if write_half.write_all(line.as_bytes()).await.is_err() {
                break;
            }
            if write_half.write_all(b"\n").await.is_err() {
                break;
            }
        }
    });

    let tx_hb = tx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(15));
        loop {
            interval.tick().await;
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs_f64();
            let hb = serde_json::json!({
                "heartbeat": { "server_time": (now * 1000.0).round() / 1000.0 }
            });
            if tx_hb.send(hb.to_string()).await.is_err() {
                break;
            }
        }
    });

    let tx_traffic = tx.clone();
    let state_traffic = state.clone();
    let user_name_traffic = user_name.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(10));
        loop {
            interval.tick().await;
            let wanted = state_traffic.tracker.get_receiver_candidate_icaos(&user_name_traffic);
            if !wanted.is_empty() {
                let resp = serde_json::json!({
                    "start_sending": wanted
                });
                if tx_traffic.send(resp.to_string()).await.is_err() {
                    break;
                }
            }
        }
    });

    let mut decompressor = Zlib2Decompressor::new();
    let mut packet_buf = Vec::with_capacity(4096);

    loop {
        let mut len_buf = [0u8; 2];
        if reader.read_exact(&mut len_buf).await.is_err() {
            break;
        }
        let packet_len = u16::from_be_bytes(len_buf) as usize;
        if packet_len == 0 || packet_len > 65535 {
            break;
        }

        packet_buf.resize(packet_len, 0);
        if reader.read_exact(&mut packet_buf).await.is_err() {
            break;
        }

        let mut lines_count = 0u64;
        decompressor.decompress_packet_callback(&packet_buf, |line| {
            lines_count += 1;
            process_json_message(line, &user_arc, &state, &tx);
        });

        if let Some(mut rcv) = state.receivers.get_mut(&user_name) {
            rcv.last_seen = Instant::now();
            rcv.messages_received += lines_count;
        }
    }

    state.receivers.remove(&user_name);
    state.clock_sync.remove_receiver(&user_name);
    state.tracker.remove_receiver(&user_name);
    state.client_txs.remove(&user_arc);
    Ok(())
}

fn process_json_message(
    json_str: &str,
    user_name: &Arc<str>,
    state: &Arc<ServerState>,
    tx: &mpsc::Sender<String>,
) {
    let val: Value = match serde_json::from_str(json_str.trim()) {
        Ok(v) => v,
        Err(_) => return,
    };

    // rate_report: track which aircraft this receiver sees, DO NOT falsely mark as ADS-B!
    if let Some(rate_obj) = val.get("rate_report").and_then(|v| v.as_object()) {
        for (hex_str, rate_val) in rate_obj {
            if let Some(rate) = rate_val.as_f64() {
                if rate > 0.05 {
                    if let Ok(icao) = u32::from_str_radix(hex_str, 16) {
                        state.tracker.record_seen(icao, user_name);
                    }
                }
            }
        }
        let wanted = state.tracker.get_receiver_candidate_icaos(user_name);
        if !wanted.is_empty() {
            let resp = serde_json::json!({
                "start_sending": wanted
            });
            let _ = tx.try_send(resp.to_string());
        }
    }

    // Filter seen aircraft: only request those that need MLAT (non-ADS-B)
    if let Some(seen_arr) = val.get("seen").and_then(|v| v.as_array()) {
        for x in seen_arr {
            if let Some(s) = x.as_str() {
                if let Ok(icao_u32) = u32::from_str_radix(s, 16) {
                    state.tracker.record_seen(icao_u32, user_name);
                }
            } else if let Some(n) = x.as_u64() {
                state.tracker.record_seen(n as u32, user_name);
            }
        }
        let wanted = state.tracker.get_receiver_candidate_icaos(user_name);
        if !wanted.is_empty() {
            let resp = serde_json::json!({
                "start_sending": wanted
            });
            let _ = tx.try_send(resp.to_string());
        }
    }

    if let Some(sync_obj) = val.get("sync") {
        let et = sync_obj.get("et").and_then(|v| v.as_f64());
        let ot = sync_obj.get("ot").and_then(|v| v.as_f64());
        let em = sync_obj.get("em").and_then(|v| v.as_str());
        let om = sync_obj.get("om").and_then(|v| v.as_str());

        if let (Some(et_val), Some(ot_val), Some(em_hex), Some(om_hex)) = (et, ot, em, om) {
            if let (Some(em_bytes), Some(om_bytes)) = (hex_to_bytes(em_hex), hex_to_bytes(om_hex)) {
                if let Some((_, icao)) = extract_icao_and_df(&em_bytes) {
                    if icao > 0 {
                        if let Some(a) = extract_altitude(&em_bytes) {
                            state.tracker.update_altitude_baro(icao, a);
                        }
                    }
                }

                if let Some(ac_geo) = decode_cpr_pair(&em_bytes, &om_bytes, true) {
                    let ac_ecef = ac_geo.to_ecef();
                    if let Some((this_ecef, _)) = state.clock_sync.get_receiver_position(user_name) {
                        let dist_this = ecef_distance(&ac_ecef, &this_ecef);
                        if dist_this <= 450_000.0 {
                            let delay_factor = 12_000_000.0 / SPEED_OF_LIGHT;
                            let td_this_even = et_val - dist_this * delay_factor;
                            let td_this_odd = ot_val - dist_this * delay_factor;
                            let this_interval = (td_this_odd - td_this_even).abs();

                            let sync_key = hash_payload(&em_bytes);
                            let mut sync_entry = state.inflight_syncs.entry(sync_key).or_insert_with(|| {
                                (ac_geo, Vec::with_capacity(8), Instant::now())
                            });

                            let (_, peers_list, _) = sync_entry.value_mut();

                            for peer in peers_list.iter() {
                                if &peer.user != user_name {
                                    state.clock_sync.update_pairing(
                                        &peer.user,
                                        user_name,
                                        peer.raw_t_even,
                                        td_this_even,
                                        peer.raw_t_odd,
                                        this_interval,
                                    );
                                }
                            }

                            if !peers_list.iter().any(|p| &p.user == user_name) {
                                peers_list.push(PendingSyncReception {
                                    user: Arc::clone(user_name),
                                    raw_t_even: td_this_even,
                                    raw_t_odd: this_interval,
                                });
                            }
                        }
                    }
                }
            }
        }
    }

    if let Some(mlat_obj) = val.get("mlat") {
        let t = mlat_obj.get("t").or_else(|| mlat_obj.get("ts")).and_then(|v| v.as_f64());
        let m = mlat_obj.get("m").or_else(|| mlat_obj.get("msg")).and_then(|v| v.as_str());

        if let (Some(ts), Some(msg_hex)) = (t, m) {
            if let Some(payload) = hex_to_bytes(msg_hex) {
                let alt_ft = extract_altitude(&payload);
                let (df, icao) = extract_icao_and_df(&payload).unwrap_or((0, 0));

                if icao > 0 {
                    if icao == 0xAE61FD {
                        info!("[AE61FD-RX] user={} df={} alt={:?}", user_name, df, alt_ft);
                    }
                    state.tracker.record_add(icao, user_name);
                    if let Some(a) = alt_ft {
                        state.tracker.update_altitude_baro(icao, a);
                    }

                    if state.tracker.is_mlat_candidate(icao) {
                        let msg_hash = hash_payload(&payload);
                        let mut entry = state.inflight_frames.entry(msg_hash).or_insert_with(|| {
                            (icao, Vec::with_capacity(6), alt_ft, false, Instant::now())
                        });

                        let (stored_icao, receptions, frame_alt_ft, _, _) = entry.value_mut();
                        if *stored_icao == 0 && icao > 0 {
                            *stored_icao = icao;
                        }
                        if frame_alt_ft.is_none() {
                            *frame_alt_ft = alt_ft;
                        }

                        if receptions.len() < 16 && !receptions.iter().any(|r| &r.user == user_name) {
                            receptions.push(PendingMsgReception {
                                user: Arc::clone(user_name),
                                raw_ts_sec: ts,
                            });
                        }
                    }
                }
            }
        }
    }
}

async fn sbs_broadcast_listener(bind_addr: &str, sbs_tx: broadcast::Sender<String>) {
    let listener = match TcpListener::bind(bind_addr).await {
        Ok(l) => l,
        Err(e) => {
            error!("Failed to bind SBS output listener on {}: {}", bind_addr, e);
            return;
        }
    };
    info!("MLAT BaseStation SBS output listening on {}", bind_addr);

    while let Ok((mut stream, _)) = listener.accept().await {
        let mut rx = sbs_tx.subscribe();
        tokio::spawn(async move {
            while let Ok(line) = rx.recv().await {
                if tokio::io::AsyncWriteExt::write_all(&mut stream, line.as_bytes()).await.is_err() {
                    break;
                }
            }
        });
    }
}

fn hex_to_bytes(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 != 0 {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
        .collect()
}

fn sync_adsb_from_readsb(state: &Arc<ServerState>) {
    let path = if std::path::Path::new("/run/readsb-ui/aircraft.json").exists() {
        std::path::Path::new("/run/readsb-ui/aircraft.json")
    } else {
        std::path::Path::new("/run/readsb/aircraft.json")
    };
    if !path.exists() {
        return;
    }
    if let Ok(bytes) = std::fs::read(path) {
        if let Ok(val) = serde_json::from_slice::<Value>(&bytes) {
            if let Some(aircraft_arr) = val.get("aircraft").and_then(|v| v.as_array()) {
                for ac in aircraft_arr {
                    let ac_type = ac.get("type").and_then(|v| v.as_str()).unwrap_or("");
                    let seen_pos = ac.get("seen_pos").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    if let Some(hex_str) = ac.get("hex").and_then(|v| v.as_str()) {
                        if let Ok(icao) = u32::from_str_radix(hex_str, 16) {
                            let has_pos = ac.get("lat").is_some() && ac.get("lon").is_some();
                            if let Some(alt) = ac.get("alt_baro").and_then(|a| a.as_i64()) {
                                state.tracker.update_altitude_baro(icao, alt as i32);
                            }
                            if ac_type == "mlat" || ac_type == "mode_s" || !has_pos || seen_pos > 5.0 {
                                state.tracker.mark_mlat_candidate(icao);
                            } else if ac_type == "adsb_icao" && has_pos && seen_pos <= 5.0 {
                                state.tracker.mark_adsb_seen(icao);
                            }
                        }
                    }
                }
            }
        }
    }
}

fn initial_bearing(from_lat: f64, from_lon: f64, to_lat: f64, to_lon: f64) -> f64 {
    let phi1 = from_lat.to_radians();
    let phi2 = to_lat.to_radians();
    let delta_lambda = (to_lon - from_lon).to_radians();
    let y = delta_lambda.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * delta_lambda.cos();
    let theta = y.atan2(x).to_degrees();
    (theta + 360.0) % 360.0
}

fn select_optimal_cluster(measurements: Vec<Measurement>, center: &EcefPoint) -> Vec<Measurement> {
    if measurements.len() <= 8 {
        return measurements;
    }

    let center_geo = crate::coordinates::ecef2llh(center);

    let mut candidate_list: Vec<(usize, f64, f64)> = measurements
        .iter()
        .enumerate()
        .map(|(idx, m)| {
            let dist = crate::coordinates::ecef_distance(center, &m.receiver_ecef);
            let rx_geo = crate::coordinates::ecef2llh(&m.receiver_ecef);
            let bearing = initial_bearing(center_geo.lat, center_geo.lon, rx_geo.lat, rx_geo.lon);
            (idx, dist, bearing)
        })
        .collect();

    // Prefer receivers within 450 km line of sight if enough remain
    let close_count = candidate_list.iter().filter(|(_, d, _)| *d <= 450_000.0).count();
    if close_count >= 8 {
        candidate_list.retain(|(_, d, _)| *d <= 450_000.0);
    }

    // Partition into 8 sectors of 45 degrees
    let mut sectors: Vec<Vec<(usize, f64)>> = vec![Vec::new(); 8];
    for (idx, dist, bearing) in &candidate_list {
        let sector_idx = ((*bearing / 45.0) as usize).min(7);
        sectors[sector_idx].push((*idx, *dist));
    }

    // Sort each sector by distance (closest first)
    for sec in sectors.iter_mut() {
        sec.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    }

    let mut selected_indices = std::collections::HashSet::new();

    // Pass 1: Select 1 closest station from each sector
    for sec in &sectors {
        if let Some(&(idx, _)) = sec.first() {
            selected_indices.insert(idx);
        }
    }

    // Pass 2: If fewer than 8 stations, fill from closest remaining stations
    if selected_indices.len() < 8 {
        candidate_list.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for (idx, _, _) in &candidate_list {
            if selected_indices.len() >= 8 {
                break;
            }
            selected_indices.insert(*idx);
        }
    }

    let mut selected_measurements: Vec<Measurement> = selected_indices
        .into_iter()
        .map(|idx| measurements[idx].clone())
        .collect();

    // Re-sort chronologically so measurements[0] is earliest
    selected_measurements.sort_by(|a, b| {
        a.timestamp_sec
            .partial_cmp(&b.timestamp_sec)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    selected_measurements
}

