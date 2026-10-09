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

## Portfolio runs (Linux, Python 3.8+)

Run `python3 scripts/portfolio.py --type <safety|array|liveness> <model-file>` for parallel portfolio verification, or use `--help` for options.
