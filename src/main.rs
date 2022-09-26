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
use critical_section::Mutex;
use core::cell::RefCell;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
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
type IrReceiver = infrared::PeriodicPoll<Nec, IrPin>;
static mut RECEIVER: Option<IrReceiver> = None;

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

    // const RESOLUTION: u32 = 1_000_000;
    // let mut receiver = Receiver::builder()
    //      .nec()
    //      .frequency(RESOLUTION)
    //      .pin(irda_pin)
    //      .build();


    use infrared::{
        // Receiver,
        receiver::{NoPin, Builder},
        protocol::{Rc6, Nec},
    };
    // use dummy_pin::DummyPin;
    use infrared::receiver::BufferInputReceiver;
    // Frequency of the timer interrupt in Hz.
    let mut receiver = infrared::PeriodicPoll::<Nec, Gpio5<Input<Floating>>>::with_pin(FREQ, irda_pin);
    // unsafe {
    //     RECEIVER.replace(receiver);
    //     }

    // timer0.start(25u64.micros());
    // timer0.listen();

    // critical_section::with(|cs| {
    //     TIMER0.borrow_ref_mut(cs).replace(timer0);
    // });

    // unsafe {
    //     riscv::interrupt::enable();
    // }



    // // Receiver for Rc6 signals, event based with embedded-hal pin
    // let pin = irda_pin;
    // let r1: Receiver<Rc6, Gpio5<Input<PullDown>>> = Receiver::with_pin(40_000, pin);

    // Periodic polled Nec Receiver
    // let pin = DummyPin::new_low();
    // let r2: infrared::PeriodicPoll<Nec, DummyPin> = infrared::PeriodicPoll::with_pin(40_000, pin);

    // let mut r3: BufferInputReceiver<NecDebug> = BufferInputReceiver::with_frequenzy(20_000);

    // let buf: &[u32] = &[20, 40, 20];
    // let cmd_iter = r3.iter(buf);

    println!("Starting main loop");
    loop {
        // for hue in 0..=255 {
            // let receiver = unsafe { RECEIVER.as_mut().unwrap() };

            if let Ok(Some(cmd)) = receiver.poll() {
                println!("A:{} C:{}", cmd.addr, cmd.cmd);
            }
            // if let Ok(Some(cmd)) = receiver.poll() {
            //     println!("x{}", cmd.addr);
            // }
            delay.delay_us(26u32);

            // let dt = 0; // Time since last pin flip

            // if let Ok(Some(button)) = receiver.event(dt) {
            //     // Get the command associated with this button
            //     let cmd = button.cmd;
            //     println!(
            //         "Action: {:?} - (Address, Command) = ()",
            //         // "Action: {:?} - (Address, Command) = ({}, {})",
            //         cmd
            //         // button.addr .action(), cmd.address(), cmd.command()
            //     );
            // }
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
        // }
    }
}


#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        // esp_println::println!("Interrupt 1");
        let receiver = unsafe { RECEIVER.as_mut().unwrap() };

        if let Ok(Some(cmd)) = receiver.poll() {
            println!("x{}", cmd.addr);
        }
        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();
        timer0.start(25u64.micros());
    });
}