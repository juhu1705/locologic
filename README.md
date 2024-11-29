# LocoLogic ![https://github.com/juhu1705/locologic/blob/main/LICENSE-APACHE](https://img.shields.io/badge/license-Apache-blue.svg)![https://github.com/juhu1705/locologic/blob/main/LICENSE-MIT](https://img.shields.io/badge/license-MIT-blue.svg)[![Tests](https://github.com/juhu1705/locologic/actions/workflows/test.yml/badge.svg?branch=main)](https://github.com/juhu1705/locologic/actions/workflows/test.yml)[![Docs](https://github.com/juhu1705/locologic/actions/workflows/doc.yml/badge.svg?branch=main)](https://github.com/juhu1705/locologic/actions/workflows/doc.yml)[![rust-clippy analyze](https://github.com/juhu1705/locologic/actions/workflows/rust-clippy.yml/badge.svg)](https://github.com/juhu1705/locologic/actions/workflows/rust-clippy.yml)

A rust library for controlling a model train railroad system.

## Features

| Feature                             | Description | Status     |
| ----------------------------------- | ----------- | ---------- |
| Automatic driving                   |             | IN PROCESS |
| Save and load railway configuration |             | PLANNING   |

## Importing the LocoLogic

As rust is able to use GitHub repositories directly as dependencies you can simply add
`locologic = { git = "https://github.com/juhu1705/locologic.git" }` to your `Cargo.toml`

## Using the LocoLogic

Not yet available

## Documentation

The documentation is published [here](https://juhu1705.github.io/locologic/doc/locologic)

## Committing to the LocoLogic

### Setting up the project

To set up the project yourself please make sure to have rust installed.

### Commitment rules

To commit to this repository please consider the [Contribution rules](./CONTRIBUTING.md).

Please note: Always add me to your pull request to test your changes with an active model railroad connection
or add some test logs to your commitment.

## Used Dependencies

### Rust

| Dependency | License |
| ---------- | ------- |
| tokio-util | MIT     |
| bytes      | MIT     |
| tokio      | MIT     |
| locodrive  | MIT     |
