use flate2::{Decompress, FlushDecompress};

pub struct Zlib2Decompressor {
    decompressor: Decompress,
    line_buf: String,
    out_buf: Vec<u8>,
    full_packet: Vec<u8>,
}

impl Default for Zlib2Decompressor {
    fn default() -> Self {
        Self {
            decompressor: Decompress::new(true),
            line_buf: String::with_capacity(1024),
            out_buf: vec![0u8; 4096],
            full_packet: Vec::with_capacity(1024),
        }
    }
}

impl Zlib2Decompressor {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn decompress_packet_callback<F: FnMut(&str)>(&mut self, packet: &[u8], mut callback: F) {
        self.full_packet.clear();
        self.full_packet.extend_from_slice(packet);
        self.full_packet.extend_from_slice(&[0x00, 0x00, 0xFF, 0xFF]);

        let mut total_in = 0;

        while total_in < self.full_packet.len() {
            let before_in = self.decompressor.total_in();
            let before_out = self.decompressor.total_out();

            let res = self.decompressor.decompress(
                &self.full_packet[total_in..],
                &mut self.out_buf,
                FlushDecompress::Sync,
            );

            if res.is_err() {
                self.decompressor.reset(true);
                break;
            }

            let consumed = (self.decompressor.total_in() - before_in) as usize;
            let written = (self.decompressor.total_out() - before_out) as usize;

            if written > 0 {
                if let Ok(chunk_str) = std::str::from_utf8(&self.out_buf[..written]) {
                    self.line_buf.push_str(chunk_str);
                    let mut start = 0;
                    while let Some(rel_pos) = self.line_buf[start..].find('\n') {
                        let end = start + rel_pos;
                        let line = self.line_buf[start..end].trim();
                        if !line.is_empty() {
                            callback(line);
                        }
                        start = end + 1;
                    }
                    if start > 0 {
                        self.line_buf.drain(..start);
                    }
                }
            }

            if consumed == 0 {
                break;
            }
            total_in += consumed;
        }
    }
}
