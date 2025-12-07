#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <limits.h>
#include <stdbool.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#define eprintf(...) fprintf(stderr, __VA_ARGS__)

#include "display.h"
#include "sim.h"
#include "util.h"

#include "config.h"

static bool running = false;

#define ASSERT(func, ...)     \
	if (!(func)) {            \
		eprintf(__VA_ARGS__); \
		res = false;          \
	}

static char *info_str = NULL;

static bool stop(bool final) {
	if (!running) return true;
	running = false;

	bool res = true;
	ASSERT(display_disable(display_params), "Failed to deinitialise display\n");
	ASSERT(sim_free(&pendulum_system), "Failed to deinitialise simulation\n");

	if (final) FREE(info_str);
	display_params.info = NULL;

	return res;
}

static bool start(bool first) {
	if (running) return true;
	running = true;

	bool res = true;
	ASSERT(sim_init(&pendulum_system), "Failed to initialise simulation\n");
	ASSERT(display_enable(display_params), "Failed to initialise display\n");

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

#define SEC 1000000000

typedef uintmax_t nsec_t;

nsec_t get_time(void) {
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (nsec_t) tp.tv_sec * SEC + tp.tv_nsec;
}

bool nsleep(nsec_t time) {
	struct timespec tp = {.tv_sec = time / SEC, .tv_nsec = time % SEC};
	return !nanosleep(&tp, NULL);
};

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

	if (!start(true)) return 3;

	const static size_t info_str_size = 1024;

	if (SHOW_INFO) {
		info_str = malloc(info_str_size);
		if (!info_str) goto fail;
	}
	display_params.info = info_str;

	const nsec_t wait_time = SEC / (MAX_FPS);
	nsec_t dest = get_time(), dest_last = dest;
	bool frame_skip = FRAME_SKIP;
	bool lag = false, first = true;
	nsec_t last_lag = 0;

	while (1) {
		nsec_t time = get_time();

		if (time > dest) dest = time + wait_time; // if more than one second has elapsed, reset the offset and wait until 1 second has passed since now
		nsec_t delay = dest - time;               // wait until destination time

		if (dest == time + wait_time || nsleep(delay)) {
			nsec_t frame_time = dest - dest_last;
			double time_advance = (SIMULATION_SPEED) * ((frame_skip ? frame_time : wait_time) / (double) SEC);

			time = get_time();
			if (!first) {
				if (!sim_step(&pendulum_system, (STEPS_PER_FRAME), time_advance)) goto fail;

				if (frame_time != wait_time) {
					lag = true;
					last_lag = time;
				}
			}
			bool show_lag = lag && time < last_lag + SEC;

			double ke, gpe, total;
			if (!sim_substitute(&ke, pendulum_system.ke, &pendulum_system)) goto fail;
			if (!sim_substitute(&gpe, pendulum_system.gpe, &pendulum_system)) goto fail;
			total = ke + gpe;

			if (info_str) {
				nsec_t sim_time = get_time() - time;
				int printf_res = snprintf(info_str, info_str_size,
				                          "             FPS: %10.3f Hz%s%s%s\n"
				                          " Simulation time: %10" PRIuMAX " ns\n"
				                          "  Kinetic energy: %10.3f J\n"
				                          "Potential energy: %10.3f J\n"
				                          "    Total energy: %10.3f J\n",
				                          SEC / (double) frame_time,
				                          show_lag ? " (" : "",
				                          show_lag ? (frame_skip ? "frame skipping" : "lagging") : "",
				                          show_lag ? ")" : "",
				                          sim_time, ke, gpe, total);
				if (printf_res < 0 || printf_res >= info_str_size) goto fail;
			}

			dest_last = dest;
			dest += wait_time; // add delay amount to destination time so we can precisely run the code on that interval
			first = false;
		}
		if (!display_render(display_params, &pendulum_system)) goto fail;
	}

	return 0;
fail:
	if (!stop(true)) return 3;
	return 1;
}
