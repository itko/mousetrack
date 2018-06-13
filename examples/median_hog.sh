#!/bin/bash

# Demonstrates the application of different 2D filters on the disparity map.
# Uses opening, closing and a median filter.
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
TRAIN=$3
FRAMES="--first-frame=${4-0} --last-frame=${5-100000}"

$APP -c -s $SRC -l trace --pipeline-timer -o $OUT ${FRAMES} \
	--pipeline-frame-window-filtering \
		hog-labeling \
		disparity-median \
		disparity-morph-open\
		disparity-morph-close\
		disparity-morph-close\
		disparity-morph-open\
		background-subtraction \
		disparity-median \
	--hog-labeling-train=$TRAIN\
	--disparity-morph-open-diameter=10 \
	--disparity-morph-close-diameter=10 \
	--disparity-median-diameter=5 \
	--background-subtraction-cage-directory=$SRC \
	--pipeline-clustering=label-clustering 

