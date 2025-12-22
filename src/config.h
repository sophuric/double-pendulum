#include <stdbool.h>
#include <stdint.h>

#include "sim.h"
#include "display.h"

typedef uintmax_t nsec_t;
#define SEC ((nsec_t) 1000000000)

struct timing_info {
	nsec_t time, sim_time, render_time, frame_time;
	bool show_lag, lag, first;
	nsec_t last_lag_time;
};

struct display_data init_display(void);
struct sim_simulation *init_simulation(void);
void free_simulation(struct sim_simulation *sim);
bool render_func(struct display_screen screen, struct sim_simulation *sim);
bool update_display_func(struct sim_simulation *sim, struct display_data *display, const struct timing_info *timing);

extern nsec_t max_fps;
extern double simulation_speed;
extern int steps_per_frame;
extern bool frame_skip; // frame skipping is non-deterministic, TODO: check if unsetting this is actually deterministic
