#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// Async cool stuff
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{self, embassy, gpio::AnyPin, Blocking};
use defmt_rtt as _; // global logger

// ESP basic stuff
use esp_hal::{clock::ClockControl, gpio::{OpenDrain, Output, IO}, peripherals::Peripherals, prelude::*, reset};
use esp_println::println;
use core::cell::RefCell;

// button interrupt
use esp_hal::gpio::{Input, PullUp};

// I2C peripherals
use static_cell::StaticCell;
use esp_hal::{peripherals::I2C0, i2c::I2C};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, NoopMutex};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use ds323x::{Ds323x, Rtcc}; // RTC
use tea5767::defs::*; // Radio

// type definition abstraction
type I2CModeESP = Blocking;
type I2CRawMutex = NoopRawMutex;
type I2CMutex<T> = NoopMutex<T>;
type I2CTypeESP<T> = I2C<'static, T, I2CModeESP>;
type I2CDeviceESP<T> = I2cDevice<'static,I2CRawMutex, I2C<'static, T, I2CModeESP>>;


/* ========================== *\
|         PANIC HANDLER        |
\* ========================== */

// dev profile: easier to debug panics; can put a breakpoint on `rust_begin_unwind`
#[cfg(debug_assertions)]
use panic_halt as _;
// release profile: minimize the binary size of the application
#[cfg(not(debug_assertions))]
use panic_abort as _;


/* ========================== *\
|            TASKS             |
\* ========================== */


#[embassy_executor::task]
async fn rtc_task(con: I2CDeviceESP<I2C0>) -> ! {
    // no async implem for DS232x with latest embedded hal
    let mut rtc = Ds323x::new_ds3231(con);

    loop {
        let time = rtc.time().unwrap();
        println!("Time: {}", time);
        let temp = rtc.temperature().unwrap();
        println!("Temperature: {}°C", temp);

        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task]
async fn radio_task(con: I2CDeviceESP<I2C0>) {
    let mut radio_tuner = TEA5767::new(
        con,
        107.0,
        BandLimits::EuropeUS,
        SoundMode::Stereo
    ).or_else(|e| {
        reset::software_reset();
        Err(e)
    }).unwrap();

    // radio
    radio_tuner.set_frequency(91.2).unwrap();
    println!("Set frequency to {}FM", radio_tuner.get_frequency().unwrap());
}


/* ========================== *\
|             MAIN             |
\* ========================== */


#[main]
 async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    embassy::init(
        &clocks,
        esp_hal::timer::TimerGroup::new_async(peripherals.TIMG0, &clocks)
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let p = io.pins;

    // I2C setup
    let sda = p.gpio21;
    let scl = p.gpio22;
    let i2c = I2C::new(peripherals.I2C0, sda, scl, 400_u32.kHz(), &clocks, None);
    static I2C_BUS: StaticCell<I2CMutex<RefCell<I2CTypeESP<I2C0>>>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(I2CMutex::new(RefCell::new(i2c)));
    let rtc_device = I2cDevice::new(i2c_bus);
    let radio_device = I2cDevice::new(i2c_bus);

    // led setup
    let mut led = p.gpio5.into_open_drain_output();
    led.set_low();

    // button setup
    let button = p.gpio18.into_pull_up_input();

    spawner.spawn(radio_task(radio_device)).unwrap();
    spawner.spawn(rtc_task(rtc_device)).unwrap();

    btnled_task(button.into(), led.into()).await
}


/* ========================== *\
|          BUTTON TASK         |
\* ========================== */


async fn btnled_task(mut button: AnyPin<Input<PullUp>>, mut led: AnyPin<Output<OpenDrain>>) -> ! {
    loop {
        button.wait_for_any_edge().await;

        let state: bool = button.is_low();
        println!("State : {}", &state);

        led.set_state(state);
    }
}
