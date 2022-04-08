#!/bin/bash

#slow
arr=("6s320rb0"
"oski2b4i"
"beemfrogs4f1"
"6s350rb46"
"6s350rb35"
"6s207rb28"
"beemlmprt8f1"
"prodcellp1"
"prodcellp2neg"
"oski15a08b01s"
"oski15a08b15s"
"intel040"
"oski15a01b13s"
"oski15a08b16s"
"intel036"
"oski2b5i"
"intel009"
"oski15a08b17s"
"oski15a01b19s"
"oski15a01b05s")
for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_unsafe_slow/ -timeout 1 -vis
done


#quick
arr=("bob9234spec5neg"
"beemprdcell2f1"
"bob9234spec6neg"
"irstdme5"
"bob9234spec4neg"
"oski3b3i"
"oski15a14b29s"
"csmacdp2"
"bobsynthetic2"
"irstdme6"
"oski15a14b33s"
"beemadd4b1"
"bj08amba2g4f3"
"bc57sensorsp1neg"
"mentorbm1and"
"oski15a01b39s")
for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_unsafe_quick/ -timeout 1 -vis
done