#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// Async cool stuff
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32_hal::{self, embassy};
use defmt_rtt as _; // global logger

// ESP basic stuff
use esp32_hal::{clock::ClockControl, gpio::{OpenDrain, Output}, peripherals::Peripherals, prelude::*, IO, reset};
use esp_backtrace as _;
use esp_println::println;
use core::cell::RefCell;

// button interrupt
use esp32_hal::gpio::{Gpio18, Gpio5, Input, PullUp};
use embedded_hal_async::digital::Wait;

// I2C peripherals
use static_cell::StaticCell;
use esp32_hal::{peripherals::I2C0, i2c::I2C};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, NoopMutex};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use ds323x::{Ds323x, Rtcc}; // RTC
use tea5767::defs::*; // Radio
// type definition abstraction
type I2CRawMutex = NoopRawMutex;
type I2cDeviceESP<T> = I2cDevice<'static,I2CRawMutex, I2C<'static, T>>;

/* ========================== *\
|            TASKS             |
\* ========================== */


#[embassy_executor::task]
async fn rtc_task(con: I2cDeviceESP<I2C0>) -> ! {
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
async fn radio_task(con: I2cDeviceESP<I2C0>) {
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
        esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks)
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let p = io.pins;

    // I2C setup
    let sda = p.gpio21;
    let scl = p.gpio22;
    let i2c = esp32_hal::i2c::I2C::new(peripherals.I2C0, sda, scl, 400_u32.kHz(), &clocks);
    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2C<'static, I2C0>>>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(NoopMutex::new(RefCell::new(i2c)));
    let rtc_device = I2cDevice::new(i2c_bus);
    let radio_device = I2cDevice::new(i2c_bus);

    // led setup
    let mut led = p.gpio5.into_open_drain_output();
    led.set_low().unwrap();

    // button setup
    let button = p.gpio18.into_pull_up_input();

    spawner.spawn(radio_task(radio_device)).unwrap();
    spawner.spawn(rtc_task(rtc_device)).unwrap();

    btnled_task(button, led).await
}


/* ========================== *\
|          BUTTON TASK         |
\* ========================== */


async fn btnled_task(mut button: Gpio18<Input<PullUp>>, mut led: Gpio5<Output<OpenDrain>>) -> ! {
    loop {
        if button.wait_for_any_edge().await.is_ok() {
            let state = button.is_low().unwrap();
            println!("State : {}", &state);

            led.set_state(state.into()).unwrap();
        }
    }
}
