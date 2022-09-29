#![no_std]
#![no_main]

extern crate alloc;

use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals, TIMG0, TIMG1},
    prelude::*,
    timer::{Timer, Timer0, TimerGroup},
    Rtc,
};
use esp32c3_hal::{ prelude::*,
    gpio::{Gpio5, IO},
    gpio_types::{Event, Input, Pin, PullDown, PullUp, Floating},
    pulse_control::ClockSource,
    Delay,
    PulseControl,
    utils::{smartLedAdapter, SmartLedsAdapter},
};
use esp_backtrace as _;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

use smart_leds::{
    brightness,
    gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

use esp_println::println;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}


#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    // let clocks = ClockControl::configure(system.clock_control, c)  .freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let mut timer0 = timer_group0.timer0;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();


    const FREQ: u32 = 38_000;
    interrupt::enable(pac::Interrupt::TG0_T0_LEVEL, interrupt::Priority::Priority1).unwrap();


    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();


    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    // let mut led = <smartLedAdapter!(16)>::new(pulse.channel0, io.pins.gpio8);
    let mut led = <smartLedAdapter!(16)>::new(pulse.channel0, io.pins.gpio1);

    let mut pir_sensor = io.pins.gpio3.into_pull_up_input();

    let mut delay = Delay::new(&clocks);

    // Warm white
    let mut color = Hsv {
        hue: 44,
        sat: 13,
        val: 99,
    };

    println!("Starting main loop");
    let mut data = [hsv2rgb(color);16];
    led.write(brightness(gamma(data.iter().cloned()), 0)).unwrap();
    loop {
        if pir_sensor.is_high().unwrap() {
            println!("- pir high - turning on the light");
            for led_brightness in 150..=255 {
                led.write(brightness(gamma(data.iter().cloned()), led_brightness)).unwrap();
                delay.delay_ms(20u8);
            }

            println!("- light is on");
            loop {
                delay.delay_ms(5000u16);
                if pir_sensor.is_low().unwrap() {
                    println!("- pir low - turning off the light");
                    break;
                } else {
                    println!("- pir high - keeping the light");
                }
            }

            for led_brightness in (0..=255).rev() {
                if pir_sensor.is_high().unwrap() {
                    println!("- pir high - stopped turning off the light");
                    led.write(brightness(gamma(data.iter().cloned()), 0)).unwrap();
                    break;
                }
                led.write(brightness(gamma(data.iter().cloned()), led_brightness)).unwrap();
                delay.delay_ms(50u8);
            }
            println!("- light is off");
        }
        delay.delay_ms(500u16);
    }
}
