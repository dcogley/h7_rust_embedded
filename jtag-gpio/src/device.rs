pub mod device {
    use {
        core::cell::RefCell,
        cortex_m::interrupt::Mutex,
        stm32h7xx_hal::{
            gpio::{self, gpiob, gpioh, gpiok},
            pac,
            prelude::*,
            pwr, rcc,
            rtc::{self, Rtc},
            usb_hs::{self, UsbBus, USB1},
        },
    };

    pub use {
        stm32h7xx_hal::{
            delay::Delay,
            gpio::{Alternate, OpenDrain, Output, Pin},
            i2c::I2c,
        },
        usb_device::{class_prelude::UsbBusAllocator, prelude::*},
        usbd_serial::SerialPort,
    };

    const I2C_ALT_FUNC: u8 = 4;
    const USB_ALT_FUNC: u8 = 12;
    const EP_MEMORY_SIZE: usize = 512;

    static mut USB_MEMORY: [u32; EP_MEMORY_SIZE] = [0u32; EP_MEMORY_SIZE];

    pub struct Device<'a> {
        pub delay: Delay,
        pub led_r: Pin<'K', 5, Output>,
        pub led_g: Pin<'K', 6, Output>,
        pub led_b: Pin<'K', 7, Output>,
        pub rtc: Rtc,
        pub i2c1: I2c<pac::I2C1>,
        pub usb_device: Mutex<RefCell<UsbDevice<'a, UsbBus<USB1>>>>,
        pub serial: Mutex<RefCell<SerialPort<'a, UsbBus<USB1>>>>,
    }

    impl Device<'static> {
        pub fn new(callback: impl FnOnce(&mut Device) -> ()) -> () {
            let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();

            cp.SCB.enable_icache();
            cp.SCB.enable_dcache(&mut cp.CPUID);

            // Power
            let pwr: pwr::Pwr = dp.PWR.constrain();
            let mut pwrcfg: pwr::PowerConfiguration = pwr.vos0(&dp.SYSCFG).freeze();
            let backup: rcc::backup::BackupREC = pwrcfg.backup().unwrap();

            // RCC
            // Constrain and Freeze clocks
            let ccdr = {
                let mut ccdr = dp
                    .RCC
                    .constrain()
                    .bypass_hse()
                    .sys_ck(480u32.MHz())
                    .hclk(240u32.MHz())
                    .pll1_strategy(rcc::PllConfigStrategy::Iterative)
                    .pll1_q_ck(240u32.MHz())
                    .pll2_strategy(rcc::PllConfigStrategy::Iterative)
                    .pll2_p_ck(100u32.MHz())
                    .pll3_strategy(rcc::PllConfigStrategy::Iterative)
                    .pll3_p_ck(100u32.MHz())
                    .pll3_r_ck(100u32.MHz())
                    .freeze(pwrcfg, &dp.SYSCFG);

                // USB Clock
                let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
                ccdr.peripheral
                    .kernel_usb_clk_mux(rcc::rec::UsbClkSel::Hsi48);
                ccdr
            };

            let mut delay: Delay = cp.SYST.delay(ccdr.clocks);

            // gpios
            let gpiob: gpiob::Parts = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let gpioh: gpioh::Parts = dp.GPIOH.split(ccdr.peripheral.GPIOH);
            let gpiok: gpiok::Parts = dp.GPIOK.split(ccdr.peripheral.GPIOK);

            // I2C1
            let scl: Pin<'B', 6, Alternate<I2C_ALT_FUNC, OpenDrain>> =
                gpiob.pb6.into_alternate_open_drain::<I2C_ALT_FUNC>();
            let sda: Pin<'B', 7, Alternate<I2C_ALT_FUNC, OpenDrain>> =
                gpiob.pb7.into_alternate_open_drain::<I2C_ALT_FUNC>();

            let mut oscen: Pin<'H', 1, Output> = gpioh.ph1.into_push_pull_output();
            delay.delay_ms(10u32);
            oscen.set_high();
            delay.delay_ms(1000u32);

            // USB
            let pdm: gpio::PB14<Alternate<USB_ALT_FUNC>> =
                gpiob.pb14.into_alternate::<USB_ALT_FUNC>();
            let pdp: gpio::PB15<Alternate<USB_ALT_FUNC>> =
                gpiob.pb15.into_alternate::<USB_ALT_FUNC>();

            let usb_alloc: UsbBusAllocator<UsbBus<USB1>> = usb_hs::UsbBus::new(
                USB1::new(
                    dp.OTG1_HS_GLOBAL,
                    dp.OTG1_HS_DEVICE,
                    dp.OTG1_HS_PWRCLK,
                    pdm,
                    pdp,
                    ccdr.peripheral.USB1OTG,
                    &ccdr.clocks,
                ),
                unsafe { &mut USB_MEMORY },
            );

            let mut device: Device = Device {
                delay: delay,
                // Configure PK5, PK6, PK7 as LEDs.
                led_r: gpiok.pk5.into_push_pull_output(),
                led_g: gpiok.pk6.into_push_pull_output(),
                led_b: gpiok.pk7.into_push_pull_output(),
                // configure the RTC
                rtc: Rtc::open_or_init(dp.RTC, backup.RTC, rtc::RtcClock::Lsi, &ccdr.clocks),
                i2c1: {
                    // initialize i2c1 and bring-up the PMIC
                    let address: u8 = 0x08;
                    let data: [u8; 2] = [0x42, 0x1];
                    let mut i2c1: I2c<pac::I2C1> =
                        dp.I2C1
                            .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);
                    i2c1.write(address, &data).unwrap();
                    i2c1
                },
                usb_device: Mutex::new(RefCell::new(
                    UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x2341, 0x035b))
                        .manufacturer("Arduino")
                        .product("Serial Port")
                        .serial_number("SN00001")
                        .device_class(usbd_serial::USB_CLASS_CDC)
                        .build(),
                )),
                serial: Mutex::new(RefCell::new(SerialPort::new(&usb_alloc))),
            };

            callback(&mut device);
        }
    }
}
