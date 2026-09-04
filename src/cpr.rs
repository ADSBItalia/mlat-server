use crate::coordinates::GeodeticPoint;

fn cpr_nl(lat_deg: f64) -> i32 {
    let lat = lat_deg.abs();
    if lat >= 87.0 {
        return 1;
    }
    if lat < 1e-6 {
        return 59;
    }

    let lat_rad = lat.to_radians();
    let num = 1.0 - (std::f64::consts::PI / 30.0).cos();
    let den = lat_rad.cos().powi(2);
    let cos_arg = 1.0 - num / den;

    if cos_arg < -1.0 || cos_arg > 1.0 {
        return 1;
    }

    let val = (2.0 * std::f64::consts::PI) / cos_arg.acos();
    val.floor() as i32
}

fn cpr_dlon(lat_deg: f64, is_odd: bool) -> f64 {
    let nl = cpr_nl(lat_deg) - (if is_odd { 1 } else { 0 });
    if nl > 0 {
        360.0 / (nl as f64)
    } else {
        360.0
    }
}

pub fn decode_cpr_pair(
    even_msg: &[u8],
    odd_msg: &[u8],
    even_is_newer: bool,
) -> Option<GeodeticPoint> {
    if even_msg.len() < 14 || odd_msg.len() < 14 {
        return None;
    }

    // Extract raw CPR coordinates (17 bits each)
    let raw_lat_e = (((even_msg[6] as u32) & 0x03) << 15)
        | (((even_msg[7] as u32) & 0xFF) << 7)
        | (((even_msg[8] as u32) >> 1) & 0x7F);
    let raw_lon_e = (((even_msg[8] as u32) & 0x01) << 16)
        | (((even_msg[9] as u32) & 0xFF) << 8)
        | ((even_msg[10] as u32) & 0xFF);

    let raw_lat_o = (((odd_msg[6] as u32) & 0x03) << 15)
        | (((odd_msg[7] as u32) & 0xFF) << 7)
        | (((odd_msg[8] as u32) >> 1) & 0x7F);
    let raw_lon_o = (((odd_msg[8] as u32) & 0x01) << 16)
        | (((odd_msg[9] as u32) & 0xFF) << 8)
        | ((odd_msg[10] as u32) & 0xFF);

    let y_e = (raw_lat_e as f64) / 131072.0;
    let x_e = (raw_lon_e as f64) / 131072.0;
    let y_o = (raw_lat_o as f64) / 131072.0;
    let x_o = (raw_lon_o as f64) / 131072.0;

    let d_lat_e = 360.0 / 60.0;
    let d_lat_o = 360.0 / 59.0;

    let j = (59.0 * y_e - 60.0 * y_o + 0.5).floor();

    let mut lat_e = d_lat_e * ((j % 60.0 + 60.0) % 60.0 + y_e);
    let mut lat_o = d_lat_o * ((j % 59.0 + 59.0) % 59.0 + y_o);

    if lat_e >= 270.0 {
        lat_e -= 360.0;
    }
    if lat_o >= 270.0 {
        lat_o -= 360.0;
    }

    let lat = if even_is_newer { lat_e } else { lat_o };
    if lat < -90.0 || lat > 90.0 {
        return None;
    }

    if cpr_nl(lat_e) != cpr_nl(lat_o) {
        return None; // Different latitude zone
    }

    let nl = cpr_nl(lat);
    let (d_lon, x, y_sel) = if even_is_newer {
        (360.0 / (nl as f64), x_e, y_e)
    } else {
        let nl_odd = (nl - 1).max(1);
        (360.0 / (nl_odd as f64), x_o, y_o)
    };

    let m = (x_e * (nl as f64 - 1.0) - x_o * (nl as f64) + 0.5).floor();
    let mut lon = d_lon * ((m % (nl as f64) + (nl as f64)) % (nl as f64) + x);
    if lon >= 180.0 {
        lon -= 360.0;
    }

    // Extract altitude from newer message
    let newer_msg = if even_is_newer { even_msg } else { odd_msg };
    let alt_bits = (((newer_msg[5] as u32) & 0xFF) << 4) | (((newer_msg[6] as u32) >> 4) & 0x0F);
    let q_bit = (alt_bits & 0x10) != 0;
    let alt_ft = if q_bit {
        let n = ((alt_bits & 0xFE0) >> 1) | (alt_bits & 0x0F);
        (n as i32) * 25 - 1000
    } else {
        5000 // Default estimated
    };

    let alt_m = (alt_ft as f64) * 0.3048;

    Some(GeodeticPoint::new(lat, lon, alt_m))
}
