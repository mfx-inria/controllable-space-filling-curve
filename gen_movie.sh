#!/bin/bash

ffmpeg -y -framerate 30 -i Tinit_%d.png -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=1 init.mp4
ffmpeg -y -framerate 30 -pattern_type glob -i 'Tcomb_*.png' -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=2 comb.mp4
ffmpeg -y -framerate 30 -pattern_type glob -i 'Tgeom_*.png' -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=2 geom.mp4
echo "file 'init.mp4'
file 'comb.mp4'
file 'geom.mp4'" > tmp.txt
ffmpeg -y -f concat -safe 0 -i tmp.txt -c copy fast_forward.mp4
rm tmp.txt
vlc fast_forward.mp4
