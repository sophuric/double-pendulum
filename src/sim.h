#ifndef SIM_H
#define SIM_H
#include <stddef.h>
struct pendulum {
	float mass, length, angular_velocity, angle;
};
struct pendulum_chain {
	float gravity;
	size_t count;
	struct pendulum *chain;
};
void step(struct pendulum_chain *chain);
#endif
