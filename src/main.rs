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

// button interrupt
use esp32_hal::{interrupt, gpio::{Event, Gpio18, Gpio5, Input, PullUp}, peripherals::Interrupt as InterruptSource};
use core::cell::RefCell;
use critical_section::Mutex;

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

struct ButtonLed {
    button: Gpio18<Input<PullUp>>,
    led: Gpio5<Output<OpenDrain>>
}
static G_BUTTON_LED: Mutex<RefCell<Option<ButtonLed>>> = Mutex::new(RefCell::new(None));


/* ========================== *\
|            TASKS             |
\* ========================== */


#[embassy_executor::task]
async fn rtc_task(con: I2cDeviceESP<I2C0>) -> ! {
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

    // button and interrupt setup
    let mut button = p.gpio18.into_pull_up_input();
    button.listen(Event::AnyEdge);
    interrupt::enable(InterruptSource::GPIO, interrupt::Priority::Priority3).unwrap();

    // consume button into mutex
    critical_section::with(|cs| {
        G_BUTTON_LED.borrow_ref_mut(cs).replace(ButtonLed {
            button,
            led
        });
    });

    spawner.spawn(radio_task(radio_device)).unwrap();
    spawner.spawn(rtc_task(rtc_device)).unwrap();

    println!("Hello world!");
    loop {
        println!("Loop...");

        // sleep a little
        Timer::after(Duration::from_secs(1)).await;
    }
}


/* ========================== *\
|           INTERRUPT          |
\* ========================== */

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        let mut bl_ref = G_BUTTON_LED
            .borrow_ref_mut(cs);

        let ButtonLed {button, led} = bl_ref
            .as_mut()
            .unwrap();

        let state = button.is_low().unwrap();
        println!("State : {}", &state);

        led.set_state(state.into()).unwrap();

        button.clear_interrupt();
    });
}
