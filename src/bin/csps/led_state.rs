#[repr(u8)]
pub enum LedState {
    Off = 0,
    On = 1,
    SlowFlash = 2,
    FastFlash = 3,
    Unknown = 4,
}

impl From<u8> for LedState {
    fn from(v: u8) -> Self {
        match v {
            0 => LedState::Off,
            1 => LedState::On,
            2 => LedState::SlowFlash,
            3 => LedState::FastFlash,
            _ => LedState::Unknown,
        }
    }
}
