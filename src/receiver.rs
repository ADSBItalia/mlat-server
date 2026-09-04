use crate::coordinates::{EcefPoint, GeodeticPoint};
use serde::{Deserialize, Serialize};
use std::time::Instant;

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct HandshakeRequest {
    pub user: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64, // meters
    #[serde(default)]
    pub privacy: bool,
    #[serde(default)]
    pub client_version: Option<String>,
    #[serde(default)]
    pub version: Option<serde_json::Value>,
    #[serde(default)]
    pub uuid: Option<String>,
    #[serde(default)]
    pub compress: Option<serde_json::Value>,
    #[serde(default)]
    pub return_results: Option<bool>,
    #[serde(default)]
    pub return_result_format: Option<String>,
}

#[derive(Debug, Clone)]
pub struct Receiver {
    pub user: String,
    pub source_ip: String,
    pub geodetic: GeodeticPoint,
    pub ecef: EcefPoint,
    pub privacy: bool,
    pub version: String,
    pub connected_at: Instant,
    pub last_seen: Instant,
    pub messages_received: u64,
    pub mlat_positions_contributed: u64,
    pub return_results: bool,
}

impl Receiver {
    pub fn new(req: HandshakeRequest, source_ip: String) -> Self {
        let geo = GeodeticPoint::new(req.lat, req.lon, req.alt);
        let ecef = geo.to_ecef();
        let now = Instant::now();
        let ver = req.client_version.unwrap_or_else(|| "0.4.2".to_string());
        let ret_res = req.return_results.unwrap_or(false);

        Self {
            user: req.user,
            source_ip,
            geodetic: geo,
            ecef,
            privacy: req.privacy,
            version: ver,
            connected_at: now,
            last_seen: now,
            messages_received: 0,
            mlat_positions_contributed: 0,
            return_results: ret_res,
        }
    }
}
