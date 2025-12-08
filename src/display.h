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

bool display_enable(struct display_data *display);
bool display_disable(struct display_data *display);
bool display_render(struct display_data *display, struct pendulum_system *system);
#endif
