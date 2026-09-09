/// Parses a `&str` as `u32`. A leading "0x" selects hex, otherwise decimal.
/// Saturates at `u32::MAX` on overflow.
pub fn parse_u32(s: &str) -> u32 {
    let (s, radix) = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(rest) => (rest, 16u32),
        None => (s, 10u32),
    };

    let mut n: u32 = 0;
    for b in s.bytes() {
        let d = match b {
            b'0'..=b'9' => (b - b'0') as u32,
            b'a'..=b'f' => (b - b'a' + 10) as u32,
            b'A'..=b'F' => (b - b'A' + 10) as u32,
            _ => break, // stop at first invalid character
        };
        n = n.saturating_mul(radix).saturating_add(d);
    }
    n
}

/// Parses a hex string (no "0x" prefix) into a fixed 16-byte array + length:
/// Returns None on: > 32 hex chars, odd length, or non-hex character.
pub fn parse_hex(s: &str) -> Option<([u8; 16], usize)> {
    let b = s.as_bytes();
    if b.len() > 32 || b.len() % 2 != 0 {
        return None;
    }

    let mut out = [0u8; 16];
    for (i, pair) in b.chunks_exact(2).enumerate() {
        out[i] = (hex_val(pair[0])? << 4) | hex_val(pair[1])?;
    }
    Some((out, b.len() / 2))
}

#[inline]
fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}
