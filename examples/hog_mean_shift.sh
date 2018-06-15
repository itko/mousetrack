#!/bin/bash

# Hog labeling and background subtraction on which mean shift clustering is performed.
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
# CSV with HOG feature vectors

$APP -c -s $SRC -l trace --pipeline-timer -o $OUT ${FRAMES} \
	--pipeline-frame-window-filtering \
	hog-labeling \
	background-subtraction \
	--hog-labeling-train=$TRAIN \
	--background-subtraction-cage-directory=$SRC \
	--background-subtraction-cage-frame=$BG_SUB \
	--pipeline-clustering=mean-shift-cpu-optimized \
	--mean-shift-sigma=0.01 \
	--mean-shift-oracle=brute-force


