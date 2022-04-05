#!/bin/bash

arr=("6s350rb35"
"6s350rb46"
"beemlmprt8f1")

for var in ${arr[@]}
do
 echo $var
  ../abc-master/abc -c "pdr;write_cex -a 'log.cex'" ../cases/$var.aig
  mv ./log.cex ../output/abc_100x_unsafe/$var.cex
done

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/abc_100x_unsafe/ -timeout 5 -vis ../output/abc_100x_unsafe/$var.cex
done
