#define MAX_FPS 240
#define SIMULATION_SPEED 1
#define STEPS_PER_FRAME 1
#define FRAME_SKIP true // frame skipping is non-deterministic
#define SHOW_INFO true

static struct display_params display_params = {
        .debug = false, // disable tcsetattr and terminal ANSI codes when entering/exiting display mode
};
static struct pendulum_system pendulum_system = {
        .gravity = 9.81,
        .count = 2,
        .chain = (struct pendulum[]) {
                                      {.mass = 1.5, .length = 1, .angvel = 0, .angle = M_PI * 2 / 3},
                                      {.mass = 1, .length = 1, .angvel = 0, .angle = M_PI / 2}}
};
