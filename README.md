# build

`git submodule update --init --recursive`

### build cadical

`cd simple_CAR/src/sat/cadical/`

`./configure --competition && make`

`cd ../../../..`

### build kissat

`cd simple_CAR/src/sat/kissat/`

`./configure --competition && make`

`cd ../../../..`

### build simple_CAR

`mkdir build`

`cd build`

`cmake .. -DCMAKE_BUILD_TYPE=Release -DCADICAL=1 -DKISSAT=1`

`make -j`

### [optional] btor2

`./setup_btor2.sh`

usage:

`python btorasaig.py <simpleCAR> [OPTIONS] -w <output_dir> <btor2_file>`

The `.cexb` file in `output_dir\` is the witness file for btor2sim.
