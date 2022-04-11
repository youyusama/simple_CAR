#!/bin/bash

#slow
arr=("intel007"
"beemelev1f1"
"pdtvisvending00"
"pdtpmscoherence"
"neclaftp1002"
"beemszmsk1f1"
"eijks444"
"power2eq32768"
"cmudme2"
"pdtvistimeout0"
"pdtvsarmultip29"
"intel024"
"oc8051gmcadata"
"6s0"
"oc8051gm9ddata"
"oc8051gm3dpc"
"intel066"
"eijkbs3271"
"intel067"
"oc8051gm68addr"
"oc8051gm58addr"
"oc8051gm6dpc")
for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_safe_slow/ -timeout 30 -vis
done


# #quick
# arr=("nusmvtcasp2"
# "bj08amba5g62"
# "oski15a14b18s"
# "6s384rb024"
# "pdtvisretherrtf1"
# "139443p0"
# "pdtvsar8multip23"
# "6s325rb072"
# "neclaftp4001"
# "vis4arbitp1"
# "neclaftp4002"
# "bobtuint21neg"
# "pdtvsarmultip08"
# "bobtuint17neg"
# "6s206rb025"
# "pdtvsarmultip13")
# for var in ${arr[@]}
# do
#  echo $var
#   ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/car_safe_quick/ -timeout 1 -vis
# done