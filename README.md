# Proof of concept GPS with maps on ATtiny3224

Just a simple PoC to investigate the feasibility of a GPS with maps on a 8-bit MCU.
The speed is not great, but might be enough for hiking.

## Building

At least 14.1.0 `avr-gcc` is required. Full setup can be found in
[build.yml](.github/workflows/build.yml).Then the usual `CMake` routine can commence:

```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DAVR_MCU=attiny3224 \
      -DAVR_PROGRAMMER=serialupdi -DAVR_PROGRAMMER_PORT=/dev/ttyUSB0 \
      -DMCU_FREQ=10000000 -DCMAKE_TOOLCHAIN_FILE=../toolchain-avr-gcc.make ..
make -j
```

