#![no_std]
#![no_main]

use esp32_hal::{clock::ClockControl, gpio::{OpenDrain, Output}, i2c::I2C, peripherals::Peripherals, prelude::*, Delay, IO, reset};
use esp_backtrace as _;
use esp_println::println;

// button interrupt
use esp32_hal::{interrupt, gpio::{Event, Gpio18, Gpio5, Input, PullUp}, peripherals::Interrupt as InterruptSource};
use core::cell::RefCell;
use critical_section::Mutex;

// I2C peripherals
use ds323x::{Ds323x, Rtcc}; // RTC
use tea5767::defs::*; // Radio
use shared_bus::BusManagerSimple; // share I2C

static G_BUTTON: Mutex<RefCell<Option<Gpio18<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static G_LED:    Mutex<RefCell<Option<Gpio5<Output<OpenDrain>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(peripherals.I2C0, io.pins.gpio21, io.pins.gpio22, 400_u32.kHz(), &clocks);
    let i2c_bus = BusManagerSimple::new(i2c);

    let proxy = i2c_bus.acquire_i2c();
    let mut radio_tuner = TEA5767::new(
        proxy,
        107.0,
        BandLimits::EuropeUS,
        SoundMode::Stereo
    ).or_else(|e| {
        reset::software_reset();
        Err(e)
    }).unwrap();
    delay.delay_ms(2000u32);
    let mut rtc = Ds323x::new_ds3231(i2c_bus.acquire_i2c());

    let mut led = io.pins.gpio5.into_open_drain_output();
    // init led state
    led.set_low().unwrap();

    // button and interrupt setup
    let mut button = io.pins.gpio18.into_pull_up_input();
    button.listen(Event::AnyEdge);
    interrupt::enable(InterruptSource::GPIO, interrupt::Priority::Priority3).unwrap();

    // consume button into mutex
    critical_section::with(|cs| {
        G_BUTTON.borrow_ref_mut(cs).replace(button);
        G_LED.borrow_ref_mut(cs).replace(led);
    });

    // radio
    radio_tuner.set_frequency(91.2).unwrap();

    println!("Hello world!");
    loop {
        println!("Loop...");

        // BUTTON
        // let state = button.is_low().unwrap();
        // println!("State : {}", &state);

        // LED
        // led.set_state(state.into()).unwrap();
        // led.set_high().unwrap();
        // delay.delay_ms(1000u32);
        // led.set_low().unwrap();
        // delay.delay_ms(1000u32);

        // RTC
        let time = rtc.time().unwrap();
        println!("Time: {}", time);
        let temp = rtc.temperature().unwrap();
        println!("Temperature: {}°C", temp);

        // sleep a little
        delay.delay_ms(1000u32);
    }
}

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        let mut btn = G_BUTTON
            .borrow_ref_mut(cs);

        let val = btn
            .as_mut()
            .unwrap();

        let state = val.is_low().unwrap();
        println!("State : {}", &state);

        G_LED
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .set_state(state.into())
            .unwrap();

        val.clear_interrupt();
    });
}
