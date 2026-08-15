# SimpleCAR

## Build

```bash
./setup.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
```

## Run

`<model-file>` may be an AIGER (`.aig`) or BTOR2 (`.btor2`) model.

```bash
./build/simpleCAR <model-file>
```

Run `./build/simpleCAR -h` to see all available options.
