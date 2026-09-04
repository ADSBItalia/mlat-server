use chrono::Utc;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct HandshakeRequest {
    pub user: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    #[serde(default)]
    pub privacy: bool,
    #[serde(default)]
    pub version: String,
}

const MODES_CRC_POLY: u32 = 0xfff409;

const fn make_crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = (i as u32) << 16;
        let mut j = 0;
        while j < 8 {
            if (c & 0x800000) != 0 {
                c = (c << 1) ^ MODES_CRC_POLY;
            } else {
                c <<= 1;
            }
            j += 1;
        }
        table[i] = c & 0xffffff;
        i += 1;
    }
    table
}

pub const CRC_TABLE: [u32; 256] = make_crc_table();

pub fn modes_crc_residual(payload: &[u8]) -> u32 {
    if payload.is_empty() {
        return 0;
    }
    let df = (payload[0] & 0xf8) >> 3;
    let n = if df > 15 { 14 } else { 7 };
    if payload.len() < n {
        return 0;
    }
    let mut rem = CRC_TABLE[payload[0] as usize];
    let mut i = 1;
    while i < n - 3 {
        let idx = ((payload[i] as u32) ^ (rem >> 16)) as usize;
        rem = ((rem & 0xffff) << 8) ^ CRC_TABLE[idx & 0xff];
        i += 1;
    }
    rem ^ ((payload[n - 3] as u32) << 16) ^ ((payload[n - 2] as u32) << 8) ^ (payload[n - 1] as u32)
}

pub fn decode_ac13(ac13: u32) -> Option<i32> {
    if ac13 == 0 || (ac13 & 0x0040) != 0 {
        return None;
    }
    if (ac13 & 0x0010) != 0 {
        // Q bit set: 25ft binary altitude
        let n = ((ac13 & 0x1f80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000f);
        let alt = (n as i32) * 25 - 1000;
        return if (-1500..=75000).contains(&alt) { Some(alt) } else { None };
    }

    // Convert from Gillham code (Gray code)
    if (ac13 & 0x1500) == 0 {
        return None;
    }

    let mut h = 0i32;
    if (ac13 & 0x1000) != 0 { h ^= 7; } // C1
    if (ac13 & 0x0400) != 0 { h ^= 3; } // C2
    if (ac13 & 0x0100) != 0 { h ^= 1; } // C4
    if (h & 5) != 0 { h ^= 5; }
    if h > 5 { return None; }

    let mut f = 0i32;
    if (ac13 & 0x0010) != 0 { f ^= 0x1ff; } // D1
    if (ac13 & 0x0004) != 0 { f ^= 0x0ff; } // D2
    if (ac13 & 0x0001) != 0 { f ^= 0x07f; } // D4
    if (ac13 & 0x0800) != 0 { f ^= 0x03f; } // A1
    if (ac13 & 0x0200) != 0 { f ^= 0x01f; } // A2
    if (ac13 & 0x0080) != 0 { f ^= 0x00f; } // A4
    if (ac13 & 0x0020) != 0 { f ^= 0x007; } // B1
    if (ac13 & 0x0008) != 0 { f ^= 0x003; } // B2
    if (ac13 & 0x0002) != 0 { f ^= 0x001; } // B4

    if (f & 1) != 0 {
        h = 6 - h;
    }

    let alt = 500 * f + 100 * h - 1300;
    if (-1500..=75000).contains(&alt) {
        Some(alt)
    } else {
        None
    }
}

pub fn extract_altitude(payload: &[u8]) -> Option<i32> {
    if payload.is_empty() {
        return None;
    }
    let df = (payload[0] >> 3) & 0x1f;
    match df {
        // DF0, DF4, DF16, DF20 (Altitude reply)
        0 | 4 | 16 | 20 => {
            if payload.len() >= 4 {
                let ac13 = (((payload[2] as u32) & 0x1f) << 8) | (payload[3] as u32);
                decode_ac13(ac13)
            } else {
                None
            }
        }
        // DF17, DF18 Extended Squitter (Airborne position)
        17 | 18 => {
            if payload.len() >= 8 {
                let type_code = (payload[4] >> 3) & 0x1f;
                if (9..=18).contains(&type_code) || (20..=22).contains(&type_code) {
                    let alt_enc = (((payload[4] as u32) & 0x07) << 10)
                        | ((payload[5] as u32) << 2)
                        | ((payload[6] as u32) >> 6);
                    let q_bit = (payload[5] & 0x10) != 0;
                    if q_bit {
                        let alt_val = if (alt_enc & 0x10) != 0 {
                            ((alt_enc & 0x0ff0) >> 1) | (alt_enc & 0x000f)
                        } else {
                            alt_enc
                        };
                        let alt_ft = (alt_val as i32 * 25) - 1000;
                        if (-1500..=75000).contains(&alt_ft) {
                            return Some(alt_ft);
                        }
                    }
                }
            }
            None
        }
        _ => None,
    }
}

pub fn extract_icao_and_df(payload: &[u8]) -> Option<(u8, u32)> {
    if payload.is_empty() {
        return None;
    }
    let df = (payload[0] >> 3) & 0x1f;
    match df {
        // DF11 (All-Call), DF17 (Extended Squitter), DF18 (Non-Transponder ES)
        11 | 17 | 18 => {
            if payload.len() >= 4 {
                let icao = ((payload[1] as u32) << 16) | ((payload[2] as u32) << 8) | (payload[3] as u32);
                Some((df, icao))
            } else {
                None
            }
        }
        // DF0, DF4, DF5, DF16, DF20, DF21: ICAO address in Parity/Address (AP)
        0 | 4 | 5 | 16 | 20 | 21 => {
            let icao = modes_crc_residual(payload);
            if icao > 0 && icao < 0x1000000 {
                Some((df, icao))
            } else {
                None
            }
        }
        _ => None,
    }
}

pub fn extract_raw_timestamps(payload: &[u8]) -> Option<(f64, f64, u64, Option<crate::coordinates::GeodeticPoint>)> {
    if payload.len() < 14 {
        return None;
    }
    let t_even = u32::from_le_bytes([payload[6], payload[7], payload[8], payload[9]]) as f64;
    let t_odd = u32::from_le_bytes([payload[10], payload[11], payload[12], payload[13]]) as f64;
    let sync_key = u64::from_le_bytes([payload[2], payload[3], payload[4], payload[5], 0, 0, 0, 0]);
    Some((t_even, t_odd, sync_key, None))
}

pub fn format_sbs_msg3(
    icao: &str,
    geo: &crate::coordinates::GeodeticPoint,
    track: Option<f32>,
    speed: Option<f32>,
    alt_ft: Option<i32>,
    vrate_fpm: Option<i32>,
    receivers_count: usize,
) -> String {
    let now = Utc::now();
    let date_str = now.format("%Y/%m/%d").to_string();
    let time_str = now.format("%H:%M:%S%.3f").to_string();

    let alt_str = alt_ft.map(|a| a.to_string()).unwrap_or_default();
    let heading_str = track.map(|t| (t.round() as i32).to_string()).unwrap_or_default();
    let speed_str = speed.map(|s| (s.round() as i32).to_string()).unwrap_or_default();
    let vrate_str = vrate_fpm.map(|v| v.to_string()).unwrap_or_default();

    format!(
        "MSG,3,1,1,{},1,{},{},{},{},,{},{},{},{:.6},{:.6},{},,{},,,0\r\n",
        icao,
        date_str,
        time_str,
        date_str,
        time_str,
        alt_str,
        speed_str,
        heading_str,
        geo.lat,
        geo.lon,
        vrate_str,
        receivers_count
    )
}
