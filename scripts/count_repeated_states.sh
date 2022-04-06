#!/bin/bash

arr=("oski15a01b01s"
"oski15a08b15s"
"pdtvsarmultip29"
"intel040"
"oski15a01b13s"
"6s320rb0"
"pdtvistimeout0"
"cmudme2"
"power2eq32768"
"oski15a08b16s"
"intel036"
"oski2b5i"
"intel009"
"eijks444"
"oski15a08b17s"
"beemszmsk1f1"
"oski2b4i"
"neclaftp1002"
"beemfrogs4f1"
"oski15a01b19s"
"oski15a01b05s"
"6s362rb0"
"6s350rb46"
"oski15a10b14s"
"beemextnc3f1"
"6s350rb35"
"pdtpmscoherence"
"pdtvisvending00"
"beemelev1f1"
"intel007"
"intel035"
"boblivea"
"6s207rb28"
"pj2007"
"beemlann2f1"
"6s339rb19"
"boblivear"
"beemlmprt8f1"
"6s277rb292"
"pdtpmsns2"
"oski1rub03i"
"pdtvisvsa16a27"
"6s221rb14"
"6s276rb342"
"beemptrsn4b1"
"intel037"
"bjrb07amba5andenv"
"prodcellp1"
"6s284rb1"
"6s275rb318"
"intel043"
"oski15a14b03s"
"prodcellp2neg"
"oski2ub0i"
"oski15a14b27s"
"oski2b2i"
"oski15a14b23s"
"oski15a14b21s"
"6s374b029"
"oski15a01b49s"
"6s339rb22"
"oski15a08b01s"
"nusmvreactorp2"
"bjrb07amba4andenv"
"oski15a08b05s"
"bobtuttt"
"oski15a14b05s"
"pdtpmsretherrtf"
"nusmvreactorp3"
"oski15a01b09s"
"pdtvisminmax0"
"viscoherencep2"
"pdtviscoherence4"
"intel026"
"viscoherencep3"
"neclabakery001"
"6s391rb379"
"bobcohdoptdcd4"
"beemmsmie1f1"
"oski15a14b25s"
"nusmvreactorp6"
"oski15a10b00s"
"139444p22"
"bc57sensorsp2"
"6s120"
"oski15a01b04s"
"6s273b37"
"beemtrngt4b1"
"productioncellp0"
"prodcellp4"
"oski15a08b13s"
"bobsynth09neg"
"prodcellp0neg"
"nusmvtcasp3"
"bc57sensorsp1"
"oski15a08b03s"
"oski2b3i"
"oski15a10b05s"
"6s421rb083"
"intel017"
"pj2016"
"nusmvtcastp3"
"nusmvbrp"
"oski15a08b11s"
"6s209b0"
"power2eq2048"
"oski2b1i"
"eijks1196"
"pdtvsar8multip13"
"eijks1238"
"139443p5"
"nusmvreactorp4"
"eijks208o"
"cmuperiodic"
"bc57sensorsp0"
"beemptrsn1b1"
"eijks344"
"oski15a01b79s"
"eijks208c"
"oski15a01b39s"
"nusmvguidancep8"
"mentorbm1or"
"139464p0"
"nusmvguidancep6")

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/count_re_s_1/ -timeout 1
done

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/count_re_s_3/ -timeout 3
done

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/count_re_s_3/ -timeout 5
done

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/count_re_s_10/ -timeout 10
done

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/count_re_s_30/ -timeout 30
done