#[repr(u8)]
pub enum LedState {
    Off = 0,
    On = 1,
    SlowFlash = 2,
    FastFlash = 3,
}

impl TryFrom<u8> for LedState {
    type Error = u8;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(LedState::Off),
            1 => Ok(LedState::On),
            2 => Ok(LedState::SlowFlash),
            3 => Ok(LedState::FastFlash),
            other => Err(other),
        }
    }
}
