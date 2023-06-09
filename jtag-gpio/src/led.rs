pub enum LED {
    Red,
    Green,
    Blue,
}

impl LED {
    #[inline(always)]
    pub unsafe fn on(self) {
        (*stm32h7xx_hal::pac::GPIOK::ptr())
            .bsrr
            .write(move |w| match self {
                Self::Red => w.br5().set_bit(),
                Self::Green => w.br6().set_bit(),
                Self::Blue => w.br7().set_bit(),
            })
    }
    #[inline(always)]
    pub unsafe fn off(self) {
        (*stm32h7xx_hal::pac::GPIOK::ptr())
            .bsrr
            .write(move |w| match self {
                Self::Red => w.bs5().set_bit(),
                Self::Green => w.bs6().set_bit(),
                Self::Blue => w.bs7().set_bit(),
            })
    }
}
