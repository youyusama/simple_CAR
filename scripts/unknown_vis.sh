arr=("6s367r"
"6s37"
"6s376r"
"6s377r"
"6s382r"
"beemandrsn6b1"
"beembkry8b1"
"beemcmbrdg7f2"
"beemextnc1f1"
"beemrwtrs2b1"
"beemskbn2b1"
"bobsmcodic"
"bobsmfpu"
"bobsmhdlc1"
"intel027"
"intel028"
"intel032"
"oc8051gm3bacc"
"oc8051gm3facc"
"oc8051gm43acc"
"oski15a08b08s"
"oski15a08b09s"
"oski15a08b10s"
"oski15a08b12s"
"pdtswvqis10x6p2"
"power2eq262144"
"shift1add262144"
"6s320rb0")

for var in ${arr[@]}
do
 echo $var
  ./simplecar_minisat -b -end -inter -rotation ../cases/$var.aig ../output/unknown_vis/ -timeout 30 -vis
done