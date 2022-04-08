#!/bin/bash

ffmpeg -y -framerate 30 -i Tinit_%d.png  -pix_fmt yuv420p -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=1 init.mp4
ffmpeg -y -framerate 30 -pattern_type glob -i 'Tcomb_*.png' -pix_fmt yuv420p -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=1.5 comb.mp4
ffmpeg -y -framerate 30 -pattern_type glob -i 'Tgeom_*.png' -pix_fmt yuv420p -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=1.5 geom.mp4
ffmpeg -y -framerate 30 -i 'ani.png'     -pix_fmt yuv420p -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=2.5 ani.mp4
ffmpeg -y -framerate 30 -i 'iso.png'     -pix_fmt yuv420p -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=2.5 iso.mp4
ffmpeg -y -framerate 30 -i 'ali.png'     -pix_fmt yuv420p -c:v libx264 -vf tpad=stop_mode=clone:stop_duration=3 ali.mp4
echo "file 'init.mp4'
file 'comb.mp4'
file 'geom.mp4'
file 'ani.mp4'
file 'iso.mp4'
file 'ali.mp4'" > tmp.txt
ffmpeg -y -f concat -safe 0 -i tmp.txt -c copy fast_forward.mp4
rm tmp.txt
vlc fast_forward.mp4
