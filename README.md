This will be my next revision of https://github.com/jonlamb-gh/air-gradient-pro-rs

Switching from RTICv1 to embassy

hw:
* STM32F407ZGTx ([olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) for now)

TODO
* get renode stuff working
* update my driver forks to use defmt
* remove unused deps in Cargo.toml, copied from example
* doc `DEF_LOG`, currently in .cargo/config
* fix the display async driver, occasional I2C NACK errors and timeouts
* error recovery

```
838.011230 ERROR Panic occured!
838.011260 ERROR panicked at src/tasks/sht40.rs:50:57:
called `Result::unwrap()` on an `Err` value: I2c(Nack)
```
