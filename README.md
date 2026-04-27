# build

`./setup.sh`

`mkdir build`

`cd build`

`cmake .. -DCMAKE_BUILD_TYPE=Release -DKISSAT=1`

`make -j`

### [optional] btor2

`cd btor2`

`setup_btor2.sh`

usage:

`python btorasaig.py <simpleCAR> [OPTIONS] -w <output_dir> <btor2_file>`

The `.cexb` file in `output_dir\` is the witness file for btor2sim.

'btor2asaig.py' would create a `.aig` file in the same directory as the input btor2 file.
