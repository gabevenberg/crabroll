A Rust-based roller blind firmware compatible with [Home Assistant](https://www.home-assistant.io/) via the Home Assistant [MQTT cover](https://www.home-assistant.io/integrations/cover.mqtt/) integration.

## Hardware

The `hardware` directory contains a KiCad project with a basic through-hole-and-module board.
The major items you will need are a Stepstick-format TMC2209 (the one from [BTT](https://biqu.equipment/products/bigtreetech-tmc2209-stepper-motor-driver-for-3d-printer-board-vs-tmc2208) works fine) and a [Seedstudio esp32c3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html).

You will also need:
* step down converter
    (the board is made for an Aliexpress one I had laying around using the MP-1584EN chip, but anything that can convert 9-24v down to 5v will work, given it can fit.)
* 4 2-pin pushbuttons
* 2 generic LEDs, one green one red
* handful of various THT resistors
* 100uF electrolytic cap
* some right-angle pin headers
* 5.5mm barrel jack
* 4 pin JST-XH socket
* power supply capable of supplying somewhere between 9-24v (depending on the torque you need out of your stepper) to a 5.5mm barrel plug
* either a reed switch and magnet or a microswitch.
* (optional) some pin header sockets, to socket your esp32c3 and tmc2209 for easy replacement.

The board assembly is straightforward.
The only thing you need to remember is that if your step down converter is adjustable,
adjust its output with a multimeter to 5v **BEFORE** you install the esp32c3.

For the installation onto the window,
it will require a bit of creativity on your part,
as every brand of blinds is different.
I use [this](https://www.printables.com/model/465889-minimalistic-motorized-roller-blinds-nema-17-stepp) mount to drive the blinds bead chain,
but depending on your exact model, you may be able to attach the motor shaft directly to the blinds.
Then, arrange for pins one and two of the right angle pin header (the one with the filled silkscreen is pin 1) to be shorted by a microswitch or a reed switch when the blinds are at their topmost position.
(pin 3 is 3.3v, for future hall effect sensor usage)

## Building

You will need:
* [rustup](https://rust-lang.org/tools/install/)
* [probe-rs](https://probe.rs/)

Then,

1. Install cargo with rustup (as described in the rustup link).
    You don't need to worry about having specific cross-compilation toolchains,
    the included `rust-toolchain.toml` file will take care of that for you.
    Also, if you use Nix flakes, I include a flake.nix with a devshell, so you can just run `nix shell`.
2. (optional) Edit `./crabroll/.cargo/config.toml`, editing the environment variables within to configure Crabroll.
3. Still in the `crabroll` directory, and with your board plugged in via USB, run `cargo embed --release`.
    This will build Crabroll and flash it to the board.
    If you don't want to edit `config.toml` to avoid accidentally committing secrets to git,
    you can prepend environment variables to your command invocation, like this: `SSID=test PASSWORD=password cargo embed --release`.

## Home Assistant:

Add the following to your Home Assistant's `config.yaml`,
changing the `identifier` and `unique_id` as desired,
and changing the `position_topic` and `set_position_topic` to match your build.

```yaml
mqtt:
- cover:
    device:
      identifiers:
        crabroll
      manufacturer: Gabe
      model: v1
      name: crabroll
    device_class: blind
    qos: 0
    position_closed: 100 
    position_open: 0
    position_topic: crabroll/test/pos
    set_position_topic: crabroll/test/command
    unique_id: 01KEHE0KF2K00XCSSD2NK8PAS7_c3bfba9a3af04e1a9bbbece23a366ee8
```

## The physical interface:

Due to the stepper motor,
it is very important to *never pull on the blind chain or otherwise override the electronics*,
as that will cause Crabroll to lose track of its position.

Crabroll does come with a 4 button physical interface for cases when Home Assistant is unavailable or inconvenient.
The topmost button (with the barrel jack side being the bottom) will open the blinds when short pressed,
and will initiate the homing procedure when pressed for more than a second.
The green LED will flash when homing is finished.
The middle-top button and middle-bottom button will jog the blinds up and down, respectively.
The bottommost button will close the blinds fully when short pressed,
and when long pressed will set the current position as the bottom position and if storage is sucsessful, the green LED will flash.

On boot, the green LED will flash once after confirming flash storage is working.

In the case of an error, the red LED will flash, and depending on the error, Crabroll may reboot.
If caught in a bootloop, either the esp32 is dying, or there is a bug, and you should file an issue.
