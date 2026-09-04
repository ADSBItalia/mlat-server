use crate::clock_sync::ClockSyncGraph;
use crate::receiver::Receiver;
use dashmap::DashMap;
use serde_json::json;
use std::fs;
use std::time::SystemTime;

/// Periodically export clients.json and sync.json to workdir
pub fn export_mlat_status(
    work_dir: &str,
    receivers: &DashMap<String, Receiver>,
    clock_sync: &ClockSyncGraph,
) {
    let now = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64();

    // 1. Build clients.json
    let mut clients_obj = serde_json::Map::new();
    for entry in receivers.iter() {
        let r = entry.value();
        let peer_count = clock_sync.get_sync_peer_count(&r.user);
        let elapsed_secs = r.connected_at.elapsed().as_secs_f64().max(1.0);
        let msg_rate = ((r.messages_received as f64) / elapsed_secs).round() as u64;

        let cdata = json!({
            "user": r.user,
            "source_ip": r.source_ip,
            "lat": r.geodetic.lat,
            "lon": r.geodetic.lon,
            "alt": r.geodetic.alt,
            "privacy": r.privacy,
            "version": r.version,
            "connected_since": now - r.connected_at.elapsed().as_secs_f64(),
            "last_message": now - r.last_seen.elapsed().as_secs_f64(),
            "messages": r.messages_received,
            "message_rate": msg_rate,
            "positions": r.mlat_positions_contributed,
            "peers": peer_count,
            "peer_count": peer_count,
        });
        clients_obj.insert(r.user.clone(), cdata);
    }

    let clients_doc = json!({
        "now": now,
        "clients": clients_obj,
    });

    let clients_path = format!("{}/clients.json", work_dir);
    let tmp_clients_path = format!("{}/clients.json.tmp", work_dir);
    if let Ok(content) = serde_json::to_string(&clients_doc) {
        if fs::write(&tmp_clients_path, content).is_ok() {
            let _ = fs::rename(&tmp_clients_path, &clients_path);
        }
    }

    // 2. Build sync.json
    let sync_map = clock_sync.export_sync_map();
    let sync_doc = json!({
        "now": now,
        "sync": sync_map,
    });

    let sync_path = format!("{}/sync.json", work_dir);
    let tmp_sync_path = format!("{}/sync.json.tmp", work_dir);
    if let Ok(content) = serde_json::to_string(&sync_doc) {
        if fs::write(&tmp_sync_path, content).is_ok() {
            let _ = fs::rename(&tmp_sync_path, &sync_path);
        }
    }
}
