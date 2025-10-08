#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::println;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    hal::debug::UARTPrint::enable(p.USART4, p.PC17, Default::default());

    loop {
        Timer::after(Duration::from_millis(1000)).await;
        println!("Hello from ch32-hal");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", info);
    loop {}
}
