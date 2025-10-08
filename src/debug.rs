//! The debug module.
//!
//! See-also: https://github.com/openwch/ch32v003/blob/main/EVT/EXAM/SDI_Printf/SDI_Printf/Debug/debug.c

use qingke::riscv;
use crate::Peripheral;

#[cfg(any(qingke_v3, qingke_v4))]
mod regs {
    pub const DEBUG_DATA0_ADDRESS: *mut u32 = 0xE000_0380 as *mut u32;
    pub const DEBUG_DATA1_ADDRESS: *mut u32 = 0xE000_0384 as *mut u32;
}

#[cfg(qingke_v2)]
mod regs {
    pub const DEBUG_DATA0_ADDRESS: *mut u32 = 0xE00000F4 as *mut u32;
    pub const DEBUG_DATA1_ADDRESS: *mut u32 = 0xE00000F8 as *mut u32;
}

pub struct SDIPrint;
    
impl SDIPrint {
    pub fn enable() {
        unsafe {
            // Enable SDI print
            core::ptr::write_volatile(regs::DEBUG_DATA0_ADDRESS, 0);
            riscv::asm::delay(100000);
        }
    }

    #[inline]
    fn is_busy() -> bool {
        unsafe { core::ptr::read_volatile(regs::DEBUG_DATA0_ADDRESS) != 0 }
    }
}

pub struct UARTPrint;

impl UARTPrint {
    /// Enable UART-based debug printing.
    /// After calling this, you can use the `println!` and `print!` macros to print to the UART.
    pub fn enable<'d, T: crate::usart::Instance, const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl crate::usart::TxPin<T, REMAP>> + 'd,
        config: crate::usart::Config,
    ) {
        crate::into_ref!(tx);
        
        // Configure the TX pin
        tx.set_as_af_output(crate::gpio::AFType::OutputPushPull, crate::gpio::Speed::High);
        T::set_remap(REMAP);
        
        // Enable and reset the peripheral
        T::enable_and_reset();
        
        // Initialize the UART using the internal helper
        uart_init::<T>(&config);
        
        // Store the send function for this instance
        unsafe {
            SENDER = Some(send_bytes::<T>);
            IS_BUSY_FN = Some(is_busy_impl::<T>);
            ENABLED = true;
        }
    }
    
    #[inline]
    fn is_busy() -> bool {
        unsafe {
            if let Some(is_busy_fn) = IS_BUSY_FN {
                is_busy_fn()
            } else {
                false
            }
        }
    }
}

/// Internal UART initialization helper that has access to SealedInstance
fn uart_init<T: crate::usart::Instance>(config: &crate::usart::Config) {
    // Access registers through the seal trait (internal to this crate)
    let rb = <T as crate::usart::sealed::SealedInstance>::regs();
    
    // Configure UART with the provided config
    rb.ctlr2().modify(|w| w.set_stop(config.stop_bits as u8));
    
    rb.ctlr1().modify(|w| {
        w.set_m(config.data_bits as u8 != 0);
        w.set_pce(config.parity != crate::usart::Parity::ParityNone);
        w.set_ps(config.parity == crate::usart::Parity::ParityOdd);
        w.set_te(true);  // Enable transmitter
        w.set_re(false); // Disable receiver (TX only)
    });
    
    // Calculate and set baud rate
    let clock_in = T::frequency().0;
    let div_m = 25 * clock_in / (4 * config.baudrate);
    let mut tmpreg = (div_m / 100) << 4;
    
    let div_f = div_m - 100 * (tmpreg >> 4);
    tmpreg |= ((div_f * 16 + 50) / 100) & 0x0F;
    
    rb.brr().write(|w| w.0 = tmpreg);
    
    // Enable UART
    rb.ctlr1().modify(|w| w.set_ue(true));
}

impl core::fmt::Write for SDIPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut data = [0u8; 8];
        for chunk in s.as_bytes().chunks(7) {
            data[1..chunk.len() + 1].copy_from_slice(chunk);
            data[0] = chunk.len() as u8;

            // data1 is the last 4 bytes of data
            let data1 = u32::from_le_bytes(data[4..].try_into().unwrap());
            let data0 = u32::from_le_bytes(data[..4].try_into().unwrap());

            while SDIPrint::is_busy() {}

            unsafe {
                core::ptr::write_volatile(regs::DEBUG_DATA1_ADDRESS, data1);
                core::ptr::write_volatile(regs::DEBUG_DATA0_ADDRESS, data0);
            }
        }

        Ok(())
    }
}

// ===== UART backend plumbing
/// Type of a simple byte send function
type SendFn = fn(&[u8]);
type IsBusyFn = fn() -> bool;

static mut SENDER: Option<SendFn> = None;
static mut IS_BUSY_FN: Option<IsBusyFn> = None;
#[doc(hidden)]
pub static mut ENABLED: bool = false;

/// Generic send implementation for a concrete USART instance T. This is a
/// very small polling loop that writes the bytes to the data register and
/// waits for TX complete.
fn send_bytes<T: crate::usart::Instance>(buf: &[u8]) {
    let r = <T as crate::usart::sealed::SealedInstance>::regs();
    for &b in buf {
        // Wait until transmit data register empty
        while !r.statr().read().txe() {}
        r.datar().write(|w| w.set_dr(b as u16));
    }
    // wait for final transmission complete
    while !r.statr().read().tc() {}
}

/// is_busy implementation for the concrete USART instance T
fn is_busy_impl<T: crate::usart::Instance>() -> bool {
    let r = <T as crate::usart::sealed::SealedInstance>::regs();
    // busy when either TXE is false (data register not empty) or TC is false
    !(r.statr().read().txe() && r.statr().read().tc())
}

impl core::fmt::Write for UARTPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        unsafe {
            if !ENABLED {
                return Err(core::fmt::Error);
            }
            
            if let Some(sender) = SENDER {
                // Wait for any pending transmission to complete
                while UARTPrint::is_busy() {}
                sender(s.as_bytes());
                Ok(())
            } else {
                Err(core::fmt::Error)
            }
        }
    }
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::writeln;

            // Try UART first, fall back to SDI
            if unsafe { $crate::debug::ENABLED } {
                let _ = writeln!(&mut $crate::debug::UARTPrint, $($arg)*);
            } else {
                let _ = writeln!(&mut $crate::debug::SDIPrint, $($arg)*);
            }
        }
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::write;

            // Try UART first, fall back to SDI
            if unsafe { $crate::debug::ENABLED } {
                let _ = write!(&mut $crate::debug::UARTPrint, $($arg)*);
            } else {
                let _ = write!(&mut $crate::debug::SDIPrint, $($arg)*);
            }
        }
    }
}
