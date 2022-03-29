#!/bin/bash

fun() {
	gimp -i -b $'
	(let*
		(
			(image (car (gimp-file-load RUN-NONINTERACTIVE "'$1$'" "'$1$'")))
			(drawable (car (gimp-image-get-active-drawable image)))
			(merged)
		)
		(gimp-context-set-foreground \x27(175 49 39))
		(gimp-text-fontname image drawable 1345 28 "'"$2"$'" 0 TRUE 68 0 "Courier Bold")
		(set! merged (car (gimp-image-merge-visible-layers image 2)))
		(gimp-file-save RUN-NONINTERACTIVE image merged "'"T$1"$'" "'"T$1"$'")
		(gimp-image-delete image)
	)' -b '(gimp-quit 0)'
}
export -f fun

ls init_*.png | xargs -I {} bash -c 'fun "$@"' _ {} "Cycle\nInitialization"
ls comb_*.png | xargs -I {} bash -c 'fun "$@"' _ {} "Combinatorial\nOptimization"
ls geom_*.png | xargs -I {} bash -c 'fun "$@"' _ {} "Geometrical\nOptimization"