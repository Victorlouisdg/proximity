# Promixity sensor model

![img](https://i.imgur.com/A3NGztG.png)

This repository contains a small proof of concept model of a proximity sensor.
It uses libigl with its interface to Embree for raycasting.

## Compile

Compile this project using the standard cmake routine:

```
mkdir build
cd build
cmake ..
make
```

This should find and build the dependencies and create a `example` binary.

## Run

From within the `build` directory just issue:
```
./example
```