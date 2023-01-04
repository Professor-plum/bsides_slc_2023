#!/bin/sh

rm flash.bin

for i in *.bmp
do
  convert $i -colorspace gray +matte -colors 2 -depth 1 temp.gray
  truncate -s +80 temp.gray
  cat temp.gray >> flash.bin
done

truncate -s 1M flash.bin