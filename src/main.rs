#![no_std]
#![no_main]

extern crate alloc;
use esp32c3_hal::{clock::ClockControl, pac::Peripherals, prelude::*, timer::TimerGroup, Rtc,
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

use infrared::{
    protocol::{AppleNec, Nec, NecDebug},
    remotecontrol::{nec::*, rc5::*},
    remotecontrol::{Action, Button},
    Receiver,
    cmd::AddressCommand
};

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


type IrPin = Gpio5<Input<Floating>>;
type IrReceiver = infrared::PeriodicPoll<NecDebug, IrPin>;
const SAMPLERATE: u32 = 1_000_000;

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

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
    let mut led = <smartLedAdapter!(16)>::new(pulse.channel0, io.pins.gpio8);

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    // let mut data;

    let mut irda_pin = io.pins.gpio5.into_floating_input();
    // let mut receiver:IrReceiver = infrared::PeriodicPoll::with_pin(SAMPLERATE, irda_pin);

 use infrared::{Receiver,
     remotecontrol::rc5::CdPlayer, cmd::AddressCommand,
    
     protocol::rc5::Rc5Command,
 };

    const RESOLUTION: u32 = 1_000_000;
    let mut receiver = Receiver::builder()
         .nec()
         .frequency(RESOLUTION)
         .pin(irda_pin)
         .remotecontrol(Apple2009)
         .build();

    println!("Starting main loop");
    loop {
        for hue in 0..=255 {

            let dt = 0; // Time since last pin flip

            if let Ok(Some(button)) = receiver.event(dt) {
                // Get the command associated with this button
                let cmd = button.command();
                println!(
                    "Action: {:?} - (Address, Command) = ({}, {})",
                    button.action(), cmd.address(), cmd.command()
                );
            }
                        // color.hue = hue;
            // // Convert from the HSV color space (where we can easily transition from one
            // // color to the other) to the RGB color space that we can then send to the LED
            // data = [hsv2rgb(color);16];
            // // When sending to the LED, we do a gamma correction first (see smart_leds
            // // documentation for details) and then limit the brightness to 10 out of 255 so
            // // that the output it's not too bright.
            // led.write(brightness(gamma(data.iter().cloned()), 10))
            //     .unwrap();
            // delay.delay_ms(20u8);


            // let r = receiver.poll();
            // match r {
            //     Ok(Some(cmd)) => {
            //         // println!(cmd.cmd); {
            //             println!("a");
            //             // match button {
            //             //     Action::Play_Pause => println!("Play was pressed!"),
            //             //     Action::Power => println!("Power on/off"),
            //             //     Action::Down => { delay.delay_ms(2000u32); }
            //             //     _ => println!("Button pressed: {:?}", button),
            //             // };
            //         // }
            //     }
            //     Ok(None) => { }
            //     Err(err) => println!("Err: {:?}", err),
            // }
        }
    }
}
