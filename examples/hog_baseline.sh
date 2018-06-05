#!/bin/bash

# Performs hog labeling and background subtraction to extract the positions of the mouse body parts.
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
# CSV with HOG feature vectors

$APP -c -s $SRC -l trace --pipeline-timer -o $OUT ${FRAMES} \
	--pipeline-frame-window-filtering hog-labeling background-subtraction \
	--hog-labeling-train=$TRAIN \
	--background-subtraction-cage-directory=$SRC \
	--pipeline-clustering=label-clustering


