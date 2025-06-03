# build

`git submodule update --init --recursive`

### build cadical

`cd simple_CAR/src/sat/cadical/`

`./configure && make`

`cd ../../../..`

### build simple_CAR

`mkdir release`

`cd release`

`cmake .. -DCMAKE_BUILD_TYPE=Release -DCADICAL=1`

`make`
