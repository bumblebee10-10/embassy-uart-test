#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

mod usb_debug;

use crc::{Crc, CRC_16_MODBUS};
pub const CRCCALC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);
pub static mut USB_MSG: [u8; 256] = [0u8; 256];

use embassy_executor::Spawner;
use embassy_rp::uart;
use {defmt_rtt as _, panic_probe as _};

use embassy_time::{Duration, Timer};
use embassy_rp::gpio::{Level, Output};

pub enum ModbusFnCode {
    ReadCoils = 0x01,
    ReadDiscreteInputs,
    ReadHoldingRegisters,
    ReadInputRegisters
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 8192;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    // unsafe { USB_MSG.copy_from_slice(b"start"); };
    let p = embassy_rp::init(Default::default());
    _spawner.spawn( usb_debug::initialize_usb(p.USB) ).unwrap();

    // use embassy_rp::watchdog::*;
    // let mut watchdog = Watchdog::new(p.WATCHDOG);
    // watchdog.start(Duration::from_millis(4_050));
    let mut led = Output::new(p.PIN_1, Level::High);
    Timer::after(Duration::from_millis(500)).await;

    let mut config = uart::Config::default();
    config.baudrate = 9600;

    let mut uart = uart::Uart::new_blocking(p.UART0, p.PIN_16, p.PIN_17, config);

    let modbus_addr = 1u8; // modbus device has addr 1
    let modbus_func = ModbusFnCode::ReadHoldingRegisters as u8;

    let modbus_reg1 = 0x1000u16; // this might be also 1000 dec = 0x3e8 = i'll check both
    let modbus_reg2 = 0x03e8u16; // this might be also 1000 dec = 0x3e8 = i'll check both

    let modbus_reg3 = 0x0108u16; // this might be also  108 dec = 0x06c
    let modbus_reg4 = 0x006cu16; // this might be also  108 dec = 0x06c

    let modbus_reg5 = 0x0100u16; // this might be also  100 dec = 0x0064
    let modbus_reg6 = 0x0064u16;

    let mut data = [ modbus_addr, modbus_func,   0,0,  0,0,  0,0 ];   // reg_base, reg_cnt, crc

    for value in [
            modbus_reg1,
            modbus_reg2,
            modbus_reg3,
            modbus_reg4,
            modbus_reg5,
            modbus_reg6
            ] {

        for _i in 0..5 {
            led.set_low();
            Timer::after(Duration::from_millis(100)).await;
            led.set_high(); // done write
            Timer::after(Duration::from_millis(100)).await;
        }
        led.set_high();


        data[2..4].copy_from_slice(&(value as u16).to_be_bytes());
        data[4..6].copy_from_slice(&[0, 2]); // how many registers = N * u16 to read
        let crc_buf = CRCCALC.checksum(&data[0..6]);
        data[6..8].copy_from_slice(&crc_buf.to_be_bytes());

        uart.blocking_write(&data).unwrap();

        led.set_low(); // done write
        Timer::after(Duration::from_millis(12)).await;
        let mut buf: [u8; 32] = [0u8; 32];
        uart.blocking_read(&mut buf).unwrap(); // never goes past this step
        led.set_high(); // done read
        unsafe {
            USB_MSG.copy_from_slice(&buf);
        }
        // watchdog.feed();
        Timer::after(Duration::from_millis(500)).await;
    }
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);

}
