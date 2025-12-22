#ifndef DISPLAY_H
#define DISPLAY_H
#include "sim.h"
#include "util.h"
#include <stdbool.h>
#include <termios.h>
struct display_screen {
	union {
		struct poss size;
		struct {
			size_t w, h;
		};
	};
	size_t buf_size;
	void *buf, *buf_damage;
};

struct display_data {
	const char *info;
	bool debug;
	struct termios old_termios;
	struct display_screen screen, term;
};

#define DISPLAY_INDEX(pos, screen) ((size_t) (pos.y) * (size_t) (screen.w) + (size_t) (pos.x))

#define DISPLAY_CELL_IN_BOUNDS(pos, screen) ((pos).x >= 0 && (pos).y >= 0 && (pos).x < (screen).w && (pos).y < (screen).h)

#define DISPLAY_GET_CELL_TYPE(pos, screen, type) (DISPLAY_CELL_IN_BOUNDS(pos, screen) ? ((type *) screen.buf)[DISPLAY_INDEX(pos, screen)] : 0)
#define DISPLAY_SET_CELL_TYPE(pos, set, screen, type) \
	if (DISPLAY_CELL_IN_BOUNDS(pos, screen))          \
	(((type *) screen.buf)[DISPLAY_INDEX(pos, screen)] = set)

#define DISPLAY_GET_CELL(pos, screen) DISPLAY_GET_CELL_TYPE(pos, screen, char)
#define DISPLAY_SET_CELL(pos, set, screen) DISPLAY_SET_CELL_TYPE(pos, set, screen, char)

bool display_enable(struct display_data *display);
bool display_disable(struct display_data *display);
bool display_render(struct display_data *display, bool (*render)(struct display_screen screen, void *render_data), void *render_data);
#endif
