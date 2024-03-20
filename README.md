# lap-sim-rs

This project should be a much rustier version than my previous lap-sim-cpp, hilarious I know.
I will take infinite inspiration from [Purdue Electric Racing](https://github.com/PurdueElectricRacing/LapSim), where they have a full run sim in matlab.
The goal is to allow for fast, precise and "easy" future modifications to setup, allowing for rapid prototyping and validation.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)

## Features

- Lap sim with static parameters
- Lap sim with dynamic interactions
- GUI for visualization and ease-of-use ([tauri](https://tauri.app/))

## Installation

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/GoncaloOlas/lap-sim-rs.git
    ```

2. Navigate to the project directory:

    ```bash
    cd lap-sim-rs
    ```

3. Compile the source code:

    ```bash
    cargo build
    ```

## Usage

1. Upload or change the desired parameters (vehicle.toml)

2. Run the executable:

    ```bash
    cargo run
    ```

## Dependencies

- [Rust](https://www.rust-lang.org/tools/install)

Make sure to have these dependencies installed on your system before compiling the application.

## Contributing

Contributions are welcome! If you have any suggestions, improvements, or feature requests, feel free to open an issue or create a pull request.

## License

This project is licensed under the MIT license.
