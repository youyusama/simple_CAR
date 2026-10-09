# Carat

Carat is a bit- and word-level model checker for AIGER and BTOR2 models.

## Build

```bash
./setup.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
```

## Run

`<model-file>` may be an AIGER (`.aig`, `.aag`) or BTOR2 (`.btor2`) model.

```bash
./build/carat <model-file>
```

Run `./build/carat -h` to see all available options.
