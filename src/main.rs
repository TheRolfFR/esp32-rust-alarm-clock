#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// Async cool stuff
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32_hal::{self, embassy};

// ESP basic stuff
use esp32_hal::{clock::ClockControl, gpio::{OpenDrain, Output}, i2c::I2C, peripherals::Peripherals, prelude::*, IO, reset};
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

struct ButtonLed {
    button: Gpio18<Input<PullUp>>,
    led: Gpio5<Output<OpenDrain>>
}

static G_BUTTON_LED: Mutex<RefCell<Option<ButtonLed>>> = Mutex::new(RefCell::new(None));

#[embassy_executor::task]
async fn one_second_task() {
  let mut count = 0;
    loop {
        esp_println::println!("Spawn Task Count: {}", count);
        count += 1;
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[main]
 async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    embassy::init(
        &clocks,
        esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks)
    );
    spawner.spawn(one_second_task()).unwrap();

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
    Timer::after(Duration::from_secs(2)).await;
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
        G_BUTTON_LED.borrow_ref_mut(cs).replace(ButtonLed {
            button,
            led
        });
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
        Timer::after(Duration::from_secs(1)).await;
    }
}

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
