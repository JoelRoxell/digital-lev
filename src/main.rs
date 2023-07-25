#![cfg_attr(test, allow(unused_imports))]
#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

// Set panicking behavior
#[cfg(not(test))]
use panic_halt as _;

use lsm303agr::{AccelOutputDataRate, Lsm303agr};

use cortex_m::{asm, iprintln, Peripherals};
use cortex_m_rt::entry;

use stm32f3xx_hal::{
    hal::digital::v2::PinState, i2c, pac, prelude::*, rcc::RccExt, time::rate::Hertz,
};

#[cfg(not(test))]
#[entry]
fn main() -> ! {
    let mut p = Peripherals::take().unwrap();
    let sw_out_buff = &mut p.ITM.stim[0];

    //  Board config & clock freeze
    let board = pac::Peripherals::take().unwrap();
    let mut rcc = board.RCC.constrain();
    let mut flash = board.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut gpioe = board.GPIOE.split(&mut rcc.ahb);
    // ---

    // IC2 setup to lsm303agr
    let mut gpiob = board.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob
        .pb6
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let sda = gpiob
        .pb7
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let i2c = i2c::I2c::new(
        board.I2C1,
        (scl, sda),
        Hertz(400_000),
        clocks,
        &mut rcc.apb1,
    );
    // ---

    // Acc sensor
    let mut sensor = Lsm303agr::new_with_i2c(i2c);

    sensor.init().unwrap();
    sensor.set_accel_odr(AccelOutputDataRate::Hz10).unwrap();
    // ---

    // LEDs
    let mut left = gpioe
        .pe15
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut center = gpioe
        .pe9
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut right = gpioe
        .pe11
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    left.set_high().unwrap();
    center.set_high().unwrap();
    right.set_high().unwrap();
    // ---

    asm::delay(8_000_000);

    let angle_limit = 10;
    // let offset = 76;

    loop {
        let res = sensor.accel_data().unwrap();
        let x = res.x;
        let tilt = translate(x, -1000, 1000, -90, 90);
        let config = consolidate_lights(tilt, angle_limit);
        let (left_active, center_active, right_active) = config;

        iprintln!(sw_out_buff, "raw: {:?} tilt: {:?} -> {:?}", x, tilt, config);

        left.set_state(left_active).unwrap();
        center.set_state(center_active).unwrap();
        right.set_state(right_active).unwrap();
    }
}

fn consolidate_lights(tilt: i32, limit: i32) -> (PinState, PinState, PinState) {
    match tilt {
        n if n < -limit => (PinState::High, PinState::Low, PinState::Low),
        n if n > limit => (PinState::Low, PinState::Low, PinState::High),
        _ => (PinState::Low, PinState::High, PinState::Low),
    }
}

fn translate(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i32 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[cfg(test)]
mod test {

    #[cfg(test)]
    mod test {
        use crate::translate;

        #[test]
        fn foo() {
            let res = translate(0, -1000, 1000, -90, 90);

            assert_eq!(res, 0);

            let res = translate(-1000, -1000, 1000, -90, 90);

            assert_eq!(res, -90);

            let res = translate(1000, -1000, 1000, -90, 90);

            assert_eq!(res, 91);
        }
    }
}
