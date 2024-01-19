# probe

WIP probe firmware using [`dap-rs`](https://github.com/korken89/dap-rs/).

Can be flashed on STLink v2 boards, including the clones.

## Installation

- Connect another probe to GND, SWDIO, SWCLK to the probe where you want to install `eprobe` on. See [boards](https://github.com/embassy-rs/eprobe/tree/main/boards) for SWD pinouts for different stlink boards.
- Run `build.sh`.
- Run `cd flasher; cargo run`
