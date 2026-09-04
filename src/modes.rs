use std::sync::LazyLock;

static CRC_TABLE: LazyLock<[u32; 256]> = LazyLock::new(|| {
    let mut table = [0u32; 256];
    let poly = 0xfff409u32;
    for i in 0..256 {
        let mut c = (i as u32) << 16;
        for _ in 0..8 {
            if (c & 0x800000) != 0 {
                c = ((c << 1) ^ poly) & 0xFFFFFF;
            } else {
                c = (c << 1) & 0xFFFFFF;
            }
        }
        table[i] = c;
    }
    table
});

pub fn crc_residual(payload: &[u8]) -> u32 {
    if payload.is_empty() {
        return 0;
    }
    let df = (payload[0] >> 3) & 0x1F;
    let n = if df > 15 { 14 } else { 7 };
    if payload.len() < n {
        return 0;
    }
    let mut rem = CRC_TABLE[payload[0] as usize];
    for i in 1..(n - 3) {
        let idx = ((payload[i] as u32) ^ (rem >> 16)) as usize;
        rem = (((rem & 0xFFFF) << 8) ^ CRC_TABLE[idx & 0xFF]) & 0xFFFFFF;
    }
    rem ^= ((payload[n - 3] as u32) << 16) | ((payload[n - 2] as u32) << 8) | (payload[n - 1] as u32);
    rem & 0xFFFFFF
}

pub fn extract_altitude(payload: &[u8]) -> Option<i32> {
    if payload.len() < 4 {
        return None;
    }
    let df = (payload[0] >> 3) & 0x1F;
    let ac13 = match df {
        0 | 4 | 16 | 20 => (((payload[2] as u16) & 0x1F) << 8) | (payload[3] as u16),
        17 | 18 if payload.len() >= 7 => {
            let tc = (payload[4] >> 3) & 0x1F;
            if (tc >= 9 && tc <= 18) || (tc >= 20 && tc <= 22) {
                let raw12 = (((payload[5] as u16) << 4) | ((payload[6] as u16) >> 4)) & 0x0FFF;
                let q_bit = (raw12 & 0x0010) != 0;
                if q_bit {
                    let n = ((raw12 & 0x0FE0) >> 1) | (raw12 & 0x000F);
                    return Some((n as i32) * 25 - 1000);
                }
            }
            return None;
        }
        _ => return None,
    };

    let q_bit = (ac13 & 0x0010) != 0;
    let m_bit = (ac13 & 0x0040) != 0;

    if m_bit {
        return None;
    }

    if q_bit {
        let n = ((ac13 & 0x1F80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000F);
        Some((n as i32) * 25 - 1000)
    } else {
        None
    }
}

pub fn extract_icao_and_df(payload: &[u8]) -> Option<(u8, u32)> {
    if payload.is_empty() {
        return None;
    }
    let df = (payload[0] >> 3) & 0x1F;
    let address = match df {
        11 | 17 | 18 => {
            if payload.len() < 4 {
                return None;
            }
            ((payload[1] as u32) << 16) | ((payload[2] as u32) << 8) | (payload[3] as u32)
        }
        0 | 4 | 5 | 16 | 20 | 21 => {
            crc_residual(payload)
        }
        _ => 0,
    };
    Some((df, address))
}
