#include "../src/config.h"

nsec_t max_fps = 240;
double simulation_speed = 1;
int steps_per_frame = 100;
bool frame_skip = true;

#include "../src/render.h"
#include "../src/linked_list.h"
#include "../src/util.h"
#include <math.h>
#include <inttypes.h>

struct display_data init_display(void) {
	return (struct display_data) {
	        .debug = false, // disable tcsetattr and terminal ANSI codes when entering/exiting display mode
	};
}

struct sim_simulation *init_simulation(void) {
	struct sim_simulation *sim;
	CWRAPPER_OUTPUT_TYPE sym_error = 0;
	bool res = false;

	basic_struct *temp = NULL, *vx = NULL, *vy = NULL, *vlx = NULL, *vly = NULL, *height = NULL, *half = NULL, *one = NULL;

	ASSERT(sim = sim_new(NULL, 1));
	sim->in_variables[0] = 9.81; // gravity
	struct sim_body *pend;

	ASSERT(pend = sim_new_body(NULL, sim, 1, 2, NULL));
	pend->coordinates[0].position = M_PI * 2 / 3; // angle
	pend->coordinates[0].velocity = 0;
	pend->in_variables[0] = 1.5; // mass
	pend->in_variables[1] = 1;   // length

	ASSERT(pend = sim_new_body(NULL, sim, 1, 2, NULL));
	pend->coordinates[0].position = M_PI / 2; // angle
	pend->coordinates[0].velocity = 0;
	pend->in_variables[0] = 1; // mass
	pend->in_variables[1] = 1; // length

	// using heap allocation to avoid freeing uninitialised data if it fails abruptly
	BASIC_NEW(temp);
	BASIC_NEW(vx);
	BASIC_NEW(vy);
	BASIC_NEW(vlx);
	BASIC_NEW(vly);
	BASIC_NEW(height);
	BASIC_NEW(half);
	BASIC_NEW(one);

	basic_const_zero(vx);
	basic_const_zero(vy);
	basic_const_zero(height);
	ASSERT_SYM(rational_set_ui(half, 1, 2));
	basic_const_one(one);

	// define energy for each of the pendulum bodies
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		// define the kinetic energy

		ASSERT_SYM(basic_sin(vlx, body->sym_coordinates[0].position));
		ASSERT_SYM(basic_cos(vly, body->sym_coordinates[0].position));

		ASSERT_SYM(basic_mul(temp, body->sym_variables[1], body->sym_coordinates[0].velocity)); // v = rω
		ASSERT_SYM(basic_mul(vlx, vlx, temp));
		ASSERT_SYM(basic_mul(vly, vly, temp));

		// add velocity vector
		ASSERT_SYM(basic_sub(vx, vx, vlx));
		ASSERT_SYM(basic_add(vy, vy, vly));

		// compute magnitude^2
		ASSERT_SYM(basic_mul(vlx, vx, vx));
		ASSERT_SYM(basic_mul(vly, vy, vy));
		ASSERT_SYM(basic_add(temp, vlx, vly));

		// multiply by mass and halve
		ASSERT_SYM(basic_mul(temp, temp, body->sym_variables[0]));
		ASSERT_SYM(basic_mul(temp, temp, half));

		// set kinetic energy, KE=0.5mv^2
		ASSERT_SYM(basic_assign(body->sym_kinetic, temp));

		// define the potential energy

		ASSERT_SYM(basic_cos(vly, body->sym_coordinates[0].position)); // cos is in the vertical axis, unlike the unit circle
		ASSERT_SYM(basic_sub(vly, one, vly));                          // flip vertically
		ASSERT_SYM(basic_mul(vly, vly, body->sym_variables[1]));       // multiply by length
		ASSERT_SYM(basic_add(height, height, vly));                    // add height
		ASSERT_SYM(basic_mul(temp, height, sim->sym_variables[0]));    // multiply by gravity
		ASSERT_SYM(basic_mul(temp, temp, body->sym_variables[0]));     // multiply by mass

		// set potential energy, GPE=mgh
		ASSERT_SYM(basic_assign(body->sym_potential, temp));
	}

	ASSERT(sim_compile(NULL, sim));

	res = true;
fail:
	BASIC_FREE(temp);
	BASIC_FREE(vx);
	BASIC_FREE(vy);
	BASIC_FREE(vlx);
	BASIC_FREE(vly);
	BASIC_FREE(height);
	BASIC_FREE(half);
	BASIC_FREE(one);
	if (res) return sim;
	sim_remove(sim);
	return NULL;
}

void free_simulation(struct sim_simulation *sim) {
	sim_remove(sim);
}

bool render_func(struct display_screen screen, struct sim_simulation *sim) {
	static const struct posf stretch = {2, 1};

	float total_length = 0;
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		total_length += body->in_variables[1];
	}

	struct rectf
	        rect_from = RECTF(-total_length, -total_length, total_length * 2, total_length * 2), // max distance the pendulum can reach
	        rect_stretched = RECTF2(POSF2(0), stretch),
	        rect_to = get_fit_rectf(rect_stretched.size, RECTF2(POSF2(0), poss2f(screen.size))); // letter-box rect to the display screen size

	struct posf pend_t = POSF(0, 0);
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		double angle = body->coordinates[0].position, length = body->in_variables[1];
		struct posf pend_f = pend_t;
		pend_t.x += sin(angle) * length;
		pend_t.y += cos(angle) * length;

		// draw line
		struct posf cell_f = map_rectf(pend_f, rect_from, rect_to),
		            cell_t = map_rectf(pend_t, rect_from, rect_to);

		draw_line(cell_f, cell_t, 1, screen);
	}

	return true;
}

bool update_display_func(struct sim_simulation *sim, struct display_data *display, const struct timing_info *timing) {
	static char str[2048];
	display->info = str;

	double kinetic = 0, potential = 0;
	for (struct sim_body *body = sim->bodies; body; body = body->next) {
		kinetic += body->out_kinetic;
		potential += body->out_potential;
	}
	double total = kinetic + potential;

	int printf_res = snprintf(str, LENGTHOF(str),
	                          "             FPS: %10.3f Hz%s%s%s\n"
	                          " Simulation time: %10" PRIuMAX " ns\n"
	                          "     Render time: %10" PRIuMAX " ns\n"
	                          "  Kinetic energy: %10.3f J\n"
	                          "Potential energy: %10.3f J\n"
	                          "    Total energy: %10.3f J\n",
	                          SEC / (double) timing->frame_time,
	                          timing->show_lag ? " (" : "",
	                          timing->show_lag ? (frame_skip ? "frame skipping" : "lagging") : "",
	                          timing->show_lag ? ")" : "",
	                          timing->sim_time, timing->render_time,
	                          kinetic, potential, total);

	return printf_res > 0 && printf_res <= LENGTHOF(str);
}
