#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <limits.h>
#include <stdbool.h>
#include <time.h>
#include <stdint.h>
#include <math.h>

#define eprintf(...) fprintf(stderr, __VA_ARGS__)

#include "display.h"
#include "sim.h"
#include "util.h"

static struct sim_simulation *simulation = NULL;
static struct display_data display;

#include "config.h"

static bool running = false;

#undef ASSERT
#define ASSERT(func, ...)     \
	if (!(func)) {            \
		eprintf(__VA_ARGS__); \
		res = false;          \
	}

static bool stop(bool final) {
	if (!running) return true;
	running = false;

	bool res = true;
	ASSERT(display_disable(&display), "Failed to deinitialise display\n");
	if (final) {
		free_simulation(simulation);
		simulation = NULL;
	}

	display.info = NULL;

	return res;
}

static bool start(bool first) {
	if (running) return true;
	running = true;

	bool res = true;
	if (first) {
		if (!simulation) ASSERT(simulation = init_simulation(), "Failed to initialise simulation\n");
	}
	ASSERT(display_enable(&display), "Failed to initialise display\n");

	if (!res) stop(true);
	return res;
}
#undef ASSERT

static void signal_func(int signal) {
	if (signal == SIGWINCH) {
		return;
	}

	if (signal == SIGCONT) {
		if (!start(false)) exit(3);
		return;
	}

	bool is_stop_signal = false;
	switch (signal) {
		case SIGTSTP:
		case SIGTTIN:
		case SIGTTOU:
			is_stop_signal = true;
	}

	bool did_stop = stop(!is_stop_signal);
	eprintf("Caught signal %02i: %s\n", signal, strsignal(signal));
	if (!did_stop) exit(3);
	if (is_stop_signal) {
		raise(SIGSTOP);
		return;
	}
	exit(signal == SIGINT ? 0 : 1);
}

nsec_t get_time(void) {
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (nsec_t) tp.tv_sec * SEC + tp.tv_nsec;
}

bool nsleep(nsec_t time) {
	struct timespec tp = {.tv_sec = time / SEC, .tv_nsec = time % SEC};
	return !nanosleep(&tp, NULL);
}

static bool main_render_func(struct display_screen screen, void *render_data) {
	return render_func(screen, (struct sim_simulation *) render_data);
}

int main(void) {
	struct sigaction sa;
	if (sigemptyset(&sa.sa_mask)) return 2;
	sa.sa_handler = signal_func;
	sa.sa_flags = 0;
	for (int signal = 1; signal < NSIG; ++signal) {
		if (signal == SIGTRAP) continue;
		if (signal == SIGCHLD || signal == SIGURG) continue;  // signals that are ignored by default (excl. SIGCONT, SIGWINCH)
		if (signal == SIGKILL || signal == SIGSTOP) continue; // can't handle these
		sigaction(signal, &sa, NULL);
	}

	display = init_display();
	if (!start(true)) return 3;

	const nsec_t wait_time = SEC / max_fps;
	nsec_t dest = get_time(), dest_last = dest;
	struct timing_info timing = {.first = true};

	while (1) {
		timing.time = get_time();

		if (timing.time > dest) dest = timing.time + wait_time; // if more than one second has elapsed, reset the offset and wait until 1 second has passed since now
		nsec_t delay = dest - timing.time;                      // wait until destination time

		if (dest == timing.time + wait_time || nsleep(delay)) {
			timing.frame_time = dest - dest_last;
			double time_advance = simulation_speed * (((frame_skip ? timing.frame_time : wait_time) / (double) SEC));

			timing.time = get_time();
			if (!timing.first) {
				if (!sim_step(simulation, steps_per_frame, time_advance)) goto fail;

				if (timing.frame_time != wait_time) {
					timing.lag = true;
					timing.last_lag_time = timing.time;
				}
			}

			timing.show_lag = timing.lag && timing.time < timing.last_lag_time + SEC;
			timing.sim_time = get_time() - timing.time;

			if (!update_display_func(simulation, &display,&timing)) goto fail;

			dest_last = dest;
			dest += wait_time; // add delay amount to destination time so we can precisely run the code on that interval
			timing.first = false;
		}
		timing.render_time = get_time();
		if (!display_render(&display, main_render_func, simulation)) goto fail;
		timing.render_time = get_time() - timing.render_time;
	}

	return 0;
fail:
	if (!stop(true)) return 3;
	return 1;
}
