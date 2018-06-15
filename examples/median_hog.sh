#!/bin/bash

# Demonstrates the application of different 2D filters on the disparity map.
# Uses opening, closing and a median filter.
#
# $1: source directory with pngs
# $2: target directory for output data
# $3: path to csv file with hog feature vectors
# $4: frame to choose for background subtraction
# $5: first frame, default: 0
# $6: last frame, default: 100000
#
# The script assumes to be run in the same directory where also the mousetrack binary is


APP=./mousetrack

SRC=$1
OUT=$2
TRAIN=$3
BG_SUB=${4-1}
FRAMES="--first-frame=${5-0} --last-frame=${6-100000}"

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
	--background-subtraction-cage-directory=$BG_SUB \
	--pipeline-clustering=label-clustering 


