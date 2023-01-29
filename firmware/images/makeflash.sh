#!/bin/sh

rm flash.bin

for i in *.png
do
  convert $i -flop -colorspace gray +matte -colors 2 -depth 1 temp.gray
  truncate -s +80 temp.gray
  cat temp.gray >> flash.bin
done

rm temp.gray

truncate -s 1M flash.bin
