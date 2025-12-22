#include <math.h>
#include "render.h"

void draw_line(struct posf pos1, struct posf pos2, bool set, struct display_screen screen) {
	struct posf delta = posf_sub(pos1, pos2);

	bool swap = fabsf(delta.y) > fabsf(delta.x);
	if (swap) { // swap x and y if gradient > 1 (45° from horizontal), otherwise there will be gaps since it loops over x-values
		SWAP_POSF(pos2);
		SWAP_POSF(pos1);
		SWAP_POSF(delta);
	}
	float gradient = delta.x != 0 ? delta.y / delta.x : 0;
	if (delta.x < 0) SWAP(struct posf, pos2, pos1); // swap from/to values to make it easier to loop
	for (size_t x = floorf(pos2.x); x <= ceilf(pos1.x); ++x) {
		struct poss cell = POSS(x, roundf(gradient * (cell.x - pos2.x) + pos2.y)); // y=m*(x-x1)+y1
		if (swap) SWAP_POSS(cell);
		DISPLAY_SET_CELL(cell, set, screen);
	}
}
