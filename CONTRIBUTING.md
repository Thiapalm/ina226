# Contributing to `ina226-tp`

First off, thanks for taking the time to contribute!

## How to Contribute

1.  **Fork the repository** on GitHub.
2.  **Clone the fork** to your local machine.
3.  **Create a new branch** for your feature or bug fix: `git checkout -b feature/your-feature-name`.
4.  **Make your changes** and ensure they follow the project's style and conventions.
5.  **Write and run tests** to verify your changes:
    *   `cargo test`
    *   `cargo test --features no_float`
    *   `cargo test --features async`
    *   `cargo test --features "async no_float"`
6.  **Check linting and formatting**:
    *   `cargo fmt -- --check`
    *   `cargo clippy -- -D warnings`
7.  **Commit your changes**: `git commit -am 'Add some feature'`.
8.  **Push to the branch**: `git push origin feature/your-feature-name`.
9.  **Submit a pull request** to the `main` branch of the original repository.

## Licensing

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as MIT and Apache 2.0, without any additional terms or conditions.
