use crate::coordinates::GeodeticPoint;
use chrono::Utc;

/// Format an MLAT solution into a standard BaseStation MSG,3 airborne position record
/// matching Python mlat-server output.py exactly:
/// MSG,{mtype},1,1,{addr:06X},1,{rcv_date},{rcv_time},{now_date},{now_time},{callsign},{altitude},{speed},{heading},{lat},{lon},{vrate},{squawk},{fs},{emerg},{ident},{aog}\n
pub fn format_sbs_msg3(
    hex: &str,
    geo: &GeodeticPoint,
    callsign: Option<&str>,
    alt_ft: Option<i32>,
    num_receivers: usize,
) -> String {
    let now = Utc::now();
    let date_str = now.format("%Y/%m/%d").to_string();
    let time_str = now.format("%H:%M:%S%.3f").to_string();

    let alt_str = alt_ft.map(|a| a.to_string()).unwrap_or_default();
    let cs = callsign.unwrap_or("");
    let fs_str = if num_receivers > 0 { num_receivers.to_string() } else { String::new() };

    format!(
        "MSG,3,1,1,{},1,{},{},{},{},{},{},,,{:.6},{:.6},,,{},,,,\n",
        hex.to_uppercase(),
        date_str,
        time_str,
        date_str,
        time_str,
        cs,
        alt_str,
        geo.lat,
        geo.lon,
        fs_str
    )
}
