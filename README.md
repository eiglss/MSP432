# MSP432
---

This repository is a peripheral driver library for the MSP432P401R micro-controller.

* The `libraries` folder contain all the header files. The sub-folder `src` contain the sources of these header files.

* The `platform` folder contain the system files.

* The `src` folder contain a `main.c` file with a template drivers.

## Usage

### Develop
To use this library include `<driver_lib>.h` to your file in `src` folder, where `<driver_lib>.h` is the name of the file in `libraries` folder that correspond to the peripheral that you want to use.

See `src/main.c` if you need a template.

### Compile
To compile execute:
```
Make
```
in the root of this repository.

---
*Note that file versioned `0` haven't been fully tested.*
