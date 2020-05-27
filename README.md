# LPC-Link2 SWO Protocol

(Written by @cbiffle with major contributions by @kc8apf and @bcantrill.)

Here is a reverse engineered minimal specification of the LPC-Link2's USB
interface for capturing SWO trace data.

## Context

The LPC-Link2 is presented as implementing the CMSIS-DAP interface, but only
implements parts of it -- and, in particular, uses a proprietary and
undocumented interface for reading SWO trace.

We like being able to get trace output from microcontrollers, and there
currently appears to be *not a single* DAP-level SWD debug probe on the market
that can do so using an open interface. (If you know of one, please let us know,
we'd love to support it.)

This document explains the proprietary USB interface in enough detail to
successfully capture data from the LPC-Link2, making it a potential option for
our needs.

## Protocol endpoint

The LPC-Link reports as *several* USB interfaces. Interface 1 is the CMSIS-DAP
port; interface 4 is the SWO port.

This interface is identified as follows:

- Interface name: `LPC-LINK2 DATA PORT`
- Two endpoints, both interrupt, EP4 IN/OUT, max packet 1024 bytes

## Command-response packet protocol

Packets are sent from the host to the LPC-Link (OUT) and from the LPC-Link to
the host (IN) as USB interrupt transactions. Packets are padded: the host
consistently sends packets of either 1 or 1024 bytes, and the device always
appears to send 1024. You must understand the length of each packet to
distinguish it from trailing garbage.

Atop interrupt transactions, the protocol is command-response. Each operation
consists of a transmission from the host (a single OUT transaction) and a
transmission from the device (a single IN transaction). This is very similar to
how CMSIS-DAP works.

For OUT transactions, the first byte of each packet gives the command number
(described below). For IN transactions, the first byte is *often* the command
number that is being replied to, but varies by command.

For the rest of this document we'll refer to the OUT transactions as "commands"
and the IN transactions as "responses."

## How to get data

To elicit SWO capture from the LPC-Link2, you need to perform the following
sequence of commands (as described below).

- One `Ohai` to kick things off.
- One `Initialize UART`.
- Some number of `Configure SWO bit rate` commands to find a rate that your
  microcontroller's trace output can match.
- Repeating `Poll capture buffer` for as long as you want data.

## Command set

Because this information was generated by cleanroom reverse engineering, all the
names are made up, and we may have missed trailing fields in packets if they're
always sent as zero in practice.

### Ohai (`0x1f`)

Sent by the host before other operations. Appears to request that the LPC-Link2
transition into a particular mode.

#### Command

```
byte[0] = 0x1f
byte[1] = mode  # ff requests UART-encoded SWO capture, other values unknown
```

#### Response

```
byte[0] = 0x1f
byte[1] = response  # contents not understood, always 0x38 in practice
```

### Initialize UART (`0x03`)

This appears to do some important UART setup, and then return the maximum bit
rate the UART could hope to achieve, along with some other data that is always
zero in practice.

#### Command

```
byte[0] = 0x03
```

#### Response

```
byte[0] = 0x03
byte[1:4] = zeroes
byte[5:8] = little endian u32 uart max bitrate in hertz
```

### Configure SWO bit rate (`0x01`)

This requests a UART bit rate. The LPC-Link2 will respond with an achievable
bit rate near the requested one. USB captures of the proprietary tools show
them going back and forth several times to negotiate a mutually agreeable rate.

#### Command

```
byte[0] = 0x01
byte[1:4] = little endian u32 target bit rate in hertz
```

#### Response

```
byte[0] = 0x01
byte[1:4] = little endian u32 achieved bit rate in hertz
```

### Poll capture buffer (`0x02`)

This requests an update if any new data has appeared in the SWO capture buffer.
USB captures of the proprietary tools show them sending this about every 12ms.

This command is unusual: there are two types of possible responses.

#### Command

```
byte[0] = 0x02
```

#### Response #1: Incremental

```
byte[0] = 0x04
byte[1] = capture epoch
byte[2:4] = packed fill levels
byte[5+] = data
```

This type of response indicates that data has (or has not) appeared in the
capture buffer since last poll.

The `capture epoch` starts at 1 and increments with each flush response (below).
This *sort of* gives the number of whole kibibytes transferred plus 1, except
that the buffer flush unit is 1022 bytes, not 1024, so... not really.

If no new data has appeared, the `packed fill levels` field in `byte[2:4]` will
contain all zeroes.

Otherwise, `byte[2:4]` contains two packed 12-bit numbers:

```
packed = bytes[2:4] as little endian 24-bit integer
fill level before this transmission = packed[11:0]
fill level after this transmission = packed[23:12]
```

The five-byte header is followed by some number of data bytes; the number can be
found by subtracting the "before" fill level from the "after" fill level.

One might assume that "no new bytes" would be signaled by sending this packet
with the current fill level repeated twice, but no, it's indicated by zeroes.

#### Response #2: Flush

```
byte[0] = 0x82
byte[1] = capture epoch
byte[2:1023] = data
```

When the capture buffer accumulates exactly 1022 bytes of data, it is flushed
using this type of response. This delivers the entire 1022 bytes, which repeats
the data you've received in `04`-type responses already.

Capture buffers appear to be double-buffered: it looks like the LPC-Link2 goes
on receiving SWO data even after the buffer fills. As a result, if the
proprietary software gets an `82`-type response, USB captures show it following
up with a rapid succession of `04`-type responses to get any additional data
that may have piled up in the buffer while the `82` response was waiting.

An `82`-type response is the final packet sent in a given `capture epoch`. The
next response will increment it by one.