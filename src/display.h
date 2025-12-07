#ifndef DISPLAY_H
#define DISPLAY_H
#include "sim.h"
#include <stdbool.h>
struct display_params {
	const char *info;
	bool debug;
};

bool display_enable(struct display_params params);
bool display_disable(struct display_params params);
bool display_render(struct display_params params, struct pendulum_system *system);
#endif
