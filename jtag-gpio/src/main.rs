#![no_main]
#![no_std]

mod device;
mod led;
use cortex_m::prelude::*;
use device::device::Device;
use led::LED;

#[cortex_m_rt::entry]
fn main() -> ! {
    // initialize the device
    Device::new(|device: &mut Device| loop {
        device.led_g.toggle();
        device.delay.delay_ms(1000u32);
    });

    loop {}
}

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    unsafe {
        LED::Green.off();
        LED::Blue.off();
        LED::Red.on();
    };

    loop {}
}
