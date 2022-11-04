#!/bin/bash

filename=$1
filename="${filename%.*}"


if [ -f "${filename}.binvox" ]
then
rm "${filename}.binvox"
fi

./binvox -c -d 512 $1 #Converting to binvox by  z-buffer based carving method only



binvox2bt "${filename}.binvox" --mark-free  -o "${filename}.bt" #converting to .bt
