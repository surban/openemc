# defmt-ringbuf

This crate stores [`defmt`] log messages in a simple ring buffer that is persisted across resets.
You still need to read the messages from the buffer and transfer them to a host for formatting.

[`defmt`]: https://github.com/knurling-rs/defmt

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT license](LICENSE-MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in defmt-ringbuf by you shall be licensed as above, 
without any additional terms or conditions.
