use core::convert::Infallible;
use ufmt::uWrite;

pub struct Sdi;

impl uWrite for Sdi {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Infallible> {
        crate::sdi_write::sdi_write(s.as_bytes());
        Ok(())
    }
}

pub struct UartFmt<M>
where
    M: ch32_hal::mode::Mode,
{
    uart_tx: ch32_hal::usart::UartTx<'static, ch32_hal::peripherals::USART1, M>,
}

impl<M> UartFmt<M>
where
    M: ch32_hal::mode::Mode,
{
    pub fn new(
        uart_tx: ch32_hal::usart::UartTx<'static, ch32_hal::peripherals::USART1, M>,
    ) -> Self {
        Self { uart_tx }
    }
    pub fn write(&mut self, buffer: &[u8]) -> Result<(), ch32_hal::usart::Error> {
        self.uart_tx.blocking_write(buffer)
    }
    pub fn flush(&mut self) -> Result<(), ch32_hal::usart::Error> {
        self.uart_tx.blocking_flush()
    }
}

impl<M> uWrite for UartFmt<M>
where
    M: ch32_hal::mode::Mode,
{
    type Error = ch32_hal::usart::Error;

    // Translate '\n' into '\r\n' for serial output
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        let mut rest = s.as_bytes();
        while let Some(i) = rest.iter().position(|&b| b == b'\n') {
            let (head, tail) = rest.split_at(i);
            self.uart_tx.blocking_write(head)?;
            if head.last() == Some(&b'\r') {
                self.uart_tx.blocking_write(b"\n")?;
            } else {
                self.uart_tx.blocking_write(b"\r\n")?;
            }
            rest = &tail[1..];
        }
        self.uart_tx.blocking_write(rest)
    }
}

#[macro_export]
macro_rules! sdi_println {
    ($($arg:tt)*) => {
        {
            #[cfg(feature = "debug")]
            let _ = ufmt::uwriteln!(&mut $crate::ufmt::Sdi, $($arg)*);
        }
    };
}

#[macro_export]
macro_rules! sdi_print {
    ($($arg:tt)*) => {
        {
            #[cfg(feature = "debug")]
            let _ = ufmt::uwrite!(&mut $crate::ufmt::Sdi, $($arg)*);
        }
    };
}
