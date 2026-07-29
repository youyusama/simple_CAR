# SimpleCAR

## build

`./setup.sh`

`mkdir build`

`cd build`

`cmake .. -DCMAKE_BUILD_TYPE=Release -DKISSAT=1`

`make -j`

## BTOR2

BTOR2 is supported directly by `simpleCAR`:

```bash
./build/simpleCAR -a ic3 -s minicore --sd model.btor2
```

For an unsafe model, `-w <output_dir>` writes
`<input-name>.cexb`. The witness can be checked with:

```bash
btor2/deps/btor2tools/build/bin/btorsim -c \
  model.btor2 <output_dir>/<input-name>.cexb
```

The frontend applies conservative selective bitblasting and package resizing
to bit-vector models. Remodellable array states are handled with selected-slot
memory abstraction and a CEGAR loop. Abstract counterexamples are replayed on
the original BTOR2 model by a word-level simulator; spurious traces refine the
tracked memory/address/delay pairs. For safe abstract results, the frontend
compares the algorithm-provided safe depth against `MaxDelay` and returns
`Unknown` if the depth is insufficient.

The first implementation supports array states whose next-state expression is
formed from the state, `write`, and array-valued `ite`. Array equality, nested
arrays, and general array inputs are rejected with a source line diagnostic.
