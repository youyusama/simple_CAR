#!/bin/bash


#1-1
arr1_1=("pdtvisretherrtf1"
"eijks208o"
"texasifetch1p1"
"6s275rb318"
"pdtvisvsa16a27"
"pdtvisns2p3"
"pdtvisvending07"
"6s276rb342"
"pj2010"
"6s277rb292"
"beemelev2f1"
"viscoherencep2"
"viscoherencep3"
"6s221rb14"
"pdtviscoherence4"
"pdtvisminmax0"
"pdtpmsns2"
"pj2007"
"beemelev1f1")
for var in ${arr1_1[@]}
do
 echo "solve $var in 1-1"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/1-1/ -timeout 60 -vis
done


#1-2
arr1_2=("6s159"
"6s307rb09"
"6s317b14"
"6s421rb083"
"beemskbn3f1"
"bobtuint16neg"
"bobtuint20neg"
"pdtpmsrotate32"
"pdtvisvending00"
"pj2003")
for var in ${arr1_2[@]}
do
 echo "solve $var in 1-2"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/1-2/ -timeout 60 -vis
done


#1-3
arr1_3=("6s0"
"6s269r"
"beembrptwo3b2"
"bob12s09"
"bob12s10"
"bobsmminiuart"
"intel024"
"intel034"
"neclaftp1002"
"neclaftp2002"
"oc8051gm9ddata"
"oc8051gmb8addr"
"pdtfifo1to0"
"pdtswvtma6x6p3"
"pdtvsarmultip29")
for var in ${arr1_3[@]}
do
 echo "solve $var in 1-3"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/1-3/ -timeout 60 -vis
done


#2
arr2=("oski15a10b03s"
"beemlmprt8f1"
"6s350rb35"
"6s350rb46"
"intel017")
for var in ${arr2[@]}
do
 echo "solve $var in 2"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/2/ -timeout 60 -vis
done


#3
arr3=("6s372rb26"
"cmuperiodic"
"oski15a14b08s"
"nusmvtcasp3"
"nusmvreactorp2"
"nusmvtcastp3"
"nusmvreactorp6"
"oski2b2i"
"oski15a01b04s"
"oski1rub03i"
"intel026")
for var in ${arr3[@]}
do
 echo "solve $var in 3"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/3/ -timeout 60 -vis
done


#4-1
arr4_1=("beemprdcell2f1"
"bobsynthetic2"
"irstdme6"
"bc57sensorsp1neg"
"oski15a14b29s"
"oski3b3i"
"bc57sensorsp1"
"oski15a14b33s"
"bc57sensorsp2"
"oski15a01b39s"
"oski15a01b79s"
"oski15a08b01s"
"6s320rb0"
"6s320rb1"
"oski2b4i")
for var in ${arr4_1[@]}
do
 echo "solve $var in 4-1"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/4-1/ -timeout 60 -vis
done


#4-2
arr4_2=("oski15a08b11s"
"oski15a14b25s"
"oski15a14b03s"
"6s374b029"
"intel035"
"6s151"
"pdtpmsretherrtf"
"6s339rb19"
"6s204b16"
"6s309b046")
for var in ${arr4_2[@]}
do
 echo "solve $var in 4-2"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/4-2/ -timeout 60 -vis
done


#5
arr5=("6s160"
"6s185"
"6s198"
"6s268r"
"6s301rb106"
"6s33"
"6s343b31"
"6s38"
"6s42"
"beemcmbrdg1f1"
"beempgsol4f1"
"bobsmhdlc1"
"intel012"
"intel020"
"intel032"
"oc8051gm49acc"
"oc8051gm63iram"
"oc8051gmadacc"
"oski15a01b34s"
"oski15a07b2s")
for var in ${arr5[@]}
do
 echo "solve $var in 5"
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_vis_comparison/5/ -timeout 60 -vis
done