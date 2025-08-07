# build

`git submodule update --init --recursive`

### build cadical

`cd simple_CAR/src/sat/cadical/`

`./configure --competition && make`

`cd ../../../..`

### build simple_CAR

`mkdir build`

`cd build`

`cmake .. -DCMAKE_BUILD_TYPE=Release -DCADICAL=1`

`make -j`
