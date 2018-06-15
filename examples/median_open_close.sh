#!/bin/bash

# Demonstrates the effects of 2D filters on the disparity map.
#
# $1: source directory with pngs
# $2: target directory for output data
# $3: frame to choose for background subtraction
# $4: first frame, default: 0
# $5: last frame, default: 100000
#
# The script assumes to be run in the same directory where also the mousetrack binary is


APP=./mousetrack

SRC=$1
OUT=$2
BG_SUB=${3-1}
FRAMES="--first-frame=${4-0} --last-frame=${5-100000}"

$APP -c -s $SRC -l trace --pipeline-timer -o $OUT ${FRAMES} \
	--pipeline-frame-window-filtering \
		disparity-median \
		disparity-morph-open\
		disparity-morph-close\
		disparity-morph-close\
		disparity-morph-open\
		background-subtraction \
		disparity-median \
	--disparity-morph-open-diameter=10 \
	--disparity-morph-close-diameter=10 \
	--disparity-median-diameter=5 \
	--background-subtraction-cage-directory=$SRC \
	--background-subtraction-cage-frame=$BG_SUB \
	--pipeline-clustering=single-cluster 


