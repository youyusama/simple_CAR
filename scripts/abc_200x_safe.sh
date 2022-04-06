#!/bin/bash

arr=("6s209b0"
"6s275rb318"
"6s276rb342"
"6s277rb292"
"6s391rb379"
"beemelev1f1"
"bobsynth09neg"
"eijks1196"
"eijks1238"
"eijks208o"
"eijks713"
"nusmvreactorp3"
"pdtviscoherence4"
"pdtvisns2p3"
"pdtvisvending00"
"pdtvisvsa16a27"
"pdtvisvsar22"
"pdtvsar8multip13"
"pj2007"
"viscoherencep2"
"viscoherencep3")

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/abc_200x_safe/ -timeout 1 -vis
done
