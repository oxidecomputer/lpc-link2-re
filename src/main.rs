//! `lpc-cat`: reference implementation for reading from the LPC-Link2 SWO
//! endpoint.

use std::convert::TryInto;
use std::error::Error;
use std::io::Write;
use std::thread::sleep;
use std::time::Duration;

use structopt::StructOpt;

/// A tool for extracting SWO trace data from an LPC-Link2.
///
/// Note: this tool will not magically cause your microcontroller to begin
/// emitting SWO trace data. This is only useful once SWO is configured.
///
/// Alternatively, you can use this tool to make the LPC-Link2 into the world's
/// least convenient unidirectional UART.
#[derive(StructOpt)]
#[structopt(max_term_width = 80)]
struct LpcCat {
    /// USB Vendor ID of the debug probe, in hex.
    #[structopt(long, short, default_value = "1fc9")]
    vid: String,
    /// USB Product ID of the debug probe, in hex.
    #[structopt(long, short, default_value = "0090")]
    pid: String,
    /// Serial number of the particular debug probe to use. This is only
    /// required when more than one identical probe are connected to the system.
    #[structopt(long, short)]
    serial: Option<String>,

    /// Allow the LPC-Link2 to choose a bitrate somewhere near the requested
    /// rate. The chosen bitrate will be printed. This is rarely useful.
    #[structopt(long)]
    allow_approx: bool,

    /// Exit before actually extracting any data. This is only useful for
    /// testing the setup and bitrate code.
    #[structopt(long)]
    no_cat: bool,

    /// Bitrate of (UART) SWO traffic, in bits per second.
    bitrate: u32,
}

fn main() -> Result<(), Box<dyn Error>> {
    pretty_env_logger::init();
    let args = LpcCat::from_args();

    let vid = u16::from_str_radix(&args.vid, 16)
        .map_err(|_| "can't parse vid as hex")?;
    let pid = u16::from_str_radix(&args.pid, 16)
        .map_err(|_| "can't parse pid as hex")?;

    let handle =
        Handle::open(vid, pid, args.serial.as_ref().map(|s| s.as_str()))?;

    // Set up the trace session.
    const MYSTERIOUS_MODE: u8 = 0xFF;
    handle.ohai(MYSTERIOUS_MODE)?;
    handle.init_uart()?;

    let actual_rate = handle.set_bit_rate(args.bitrate)?;
    if actual_rate != args.bitrate {
        if args.allow_approx {
            eprintln!(
                "actual bit rate: {} (requested: {})",
                actual_rate, args.bitrate
            );
        } else {
            log::error!(
                "can't achieve bit rate {} (closest: {})",
                args.bitrate,
                actual_rate
            );
            std::process::exit(1);
        }
    } else {
        log::info!("probe confirms rate: {}", actual_rate);
    }

    if args.no_cat {
        return Ok(());
    }

    const MAX_PACKET: usize = 1024;
    const POLL_INTERVAL: Duration = Duration::from_millis(10);

    let mut buffer = [0; MAX_PACKET];
    let mut last: Option<(u8, u16)> = None;
    let out = std::io::stdout();
    let mut out = out.lock();

    loop {
        let (epoch, result) = handle.poll(&mut buffer)?;
        match result {
            PollResult::Empty => {
                // Try back in a bit.
                sleep(POLL_INTERVAL);
            }
            PollResult::Incremental {
                start,
                end,
                fragment,
            } => {
                let continuous = if let Some((last_epoch, last_end)) = last {
                    last_epoch == epoch && last_end == start
                } else {
                    // If this is the first data we've seen, we'll accept it
                    // unconditionally.
                    true
                };

                if !continuous {
                    eprintln!(
                        "lost stream sync at {:02x}:{:03x}, data may be lost",
                        epoch, start
                    );
                }
                out.write_all(fragment)?;
                last = Some((epoch, end));
            }
            PollResult::Total(packet) => {
                if let Some((last_epoch, last_end)) = last {
                    if epoch == last_epoch {
                        // We need to collect the tail of the data for this
                        // epoch from the end of the packet buffer.
                        out.write_all(&packet[usize::from(last_end)..])?;
                    } else {
                        eprintln!(
                            "lost stream sync at {:02x}:000, data may be lost",
                            epoch
                        );
                    }
                } else {
                    // This is kind of a boring first packet, but ok.
                    out.write_all(packet)?;
                }
                last = Some((epoch.wrapping_add(1), 0));
            }
        }
    }
}

struct Handle(hidapi::HidDevice);

impl Handle {
    /// Opens the LPC-Link2 device with the given vid/pid and optional serial
    /// number.
    ///
    /// If `serial` is `None`, the first matching device will be chosen.
    pub fn open(
        vid: u16,
        pid: u16,
        serial: Option<&str>,
    ) -> Result<Self, Box<dyn Error>> {
        let api = hidapi::HidApi::new()?;

        const TRACE_IF_NO: i32 = 4;

        let device_info = api
            .device_list()
            .filter(|d| d.vendor_id() == vid && d.product_id() == pid)
            .filter(|d| d.interface_number() == TRACE_IF_NO)
            .filter(|d| {
                if let Some(serial) = serial {
                    d.serial_number() == Some(serial)
                } else {
                    true
                }
            })
            .next()
            .ok_or("can't find matching device")?;

        log::info!(
            "found matching device at {}",
            device_info.path().to_string_lossy()
        );

        let device = device_info.open_device(&api)?;

        Ok(Self(device))
    }

    /// Initializes communications with the probe.
    pub fn ohai(&self, mode: u8) -> Result<(), Box<dyn Error>> {
        self.0.write(&[0x1f, mode])?;

        let mut response = [0; 1024];
        self.0.read(&mut response)?;

        check_cmd(response[0], 0x1f)?;
        if response[1] != 0x38 {
            return Err("unexpected ohai response".into());
        }

        Ok(())
    }

    /// Does basic UART setup and returns the highest available bit rate.
    pub fn init_uart(&self) -> Result<u32, Box<dyn Error>> {
        self.0.write(&[0x03])?;

        let mut response = [0; 1024];
        self.0.read(&mut response)?;

        check_cmd(response[0], 0x03)?;

        Ok(u32::from_le_bytes(response[5..9].try_into().unwrap()))
    }

    /// Configures the UART for a particular bit rate (in bits per second). The
    /// probe can't achieve just *any* rate, and will return a nearby achievable
    /// rate.
    pub fn set_bit_rate(&self, rate: u32) -> Result<u32, Box<dyn Error>> {
        let mut req = [0x01, 0, 0, 0, 0];
        req[1..].copy_from_slice(&rate.to_le_bytes());
        self.0.write(&req)?;

        let mut response = [0; 1024];
        self.0.read(&mut response)?;

        check_cmd(response[0], 0x01)?;

        Ok(u32::from_le_bytes(response[1..5].try_into().unwrap()))
    }

    /// Asks the probe for an update on its capture buffer.
    ///
    /// Returns a tuple of `(epoch, poll_result)`.
    pub fn poll<'a>(
        &self,
        buffer: &'a mut [u8],
    ) -> Result<(u8, PollResult<'a>), Box<dyn Error>> {
        assert!(buffer.len() >= 1024);

        self.0.write(&[0x02])?;

        let n = self.0.read(buffer)?;

        let response = &mut buffer[..n];

        let kind = response[0];
        let epoch = response[1];

        match kind {
            0x04 => {
                let packed_levels = u32::from(response[2])
                    | u32::from(response[3]) << 8
                    | u32::from(response[4]) << 16;

                if packed_levels == 0 {
                    return Ok((epoch, PollResult::Empty));
                }
                let start = (packed_levels & 0xFFF) as u16;
                let end = (packed_levels >> 12) as u16;

                if end < start {
                    return Err("invalid fill levels!".into());
                }
                let n = usize::from(end - start);
                Ok((
                    epoch,
                    PollResult::Incremental {
                        start,
                        end,
                        fragment: &mut response[5..5 + n],
                    },
                ))
            }
            0x82 => Ok((epoch, PollResult::Total(&mut response[2..1024]))),
            _ => Err("unexpected poll response".into()),
        }
    }
}

pub enum PollResult<'a> {
    /// No new traffic in buffer since last poll.
    Empty,
    /// New traffic has appeared in the buffer at offset `start` (inclusive)
    /// through `end` (exclusive).
    Incremental {
        start: u16,
        end: u16,
        fragment: &'a mut [u8],
    },
    /// The buffer has filled up; here's the whole thing. If you've been polling
    /// regularly, this will repeat data received in previous `Incremental`
    /// messages.
    Total(&'a mut [u8]),
}

fn check_cmd(c: u8, expected: u8) -> Result<(), Box<dyn Error>> {
    if c != expected {
        Err("unexpected response".into())
    } else {
        Ok(())
    }
}
