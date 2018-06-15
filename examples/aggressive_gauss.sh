#!/bin/bash

# Using a combination of different 2D filters on the disparity maps, we try to reconstruct the 
# the entire data set as "smoothly" as possible. To get rid of discretization artifacts, we apply a gaussian filter.
#
# $1: source directory with pngs
# $2: target directory for output data
# $3: first frame, default: 0
# $4: last frame, default: 100000
#
# The script assumes to be run in the same directory where also the mousetrack binary is


APP=./mousetrack

SRC=$1
OUT=$2
FRAMES="--first-frame=${3-0} --last-frame=${4-100000}"

$APP -c -s $SRC -l trace --pipeline-timer -o $OUT ${FRAMES} \
	--pipeline-frame-window-filtering \
		disparity-median \
		disparity-morph-open\
		disparity-morph-close\
		disparity-morph-close\
		disparity-morph-open\
		disparity-gauss\
		disparity-morph-open\
	--disparity-gauss-k=10 \
	--disparity-morph-open-diameter=10 \
	--disparity-morph-close-diameter=10 \
	--disparity-median-diameter=5 \
	--pipeline-clustering=single-cluster 


