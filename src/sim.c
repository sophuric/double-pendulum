#include "sim.h"

static unsigned log10i(size_t x) {
	unsigned i;
	for (i = 1; x >= 10; x /= 10) ++i;
	return i;
}

#define ASSERT(x) \
	if ((res = (x))) goto fail
#define HEAP_ALLOC(x) \
	if (!(x = basic_new_heap())) goto fail
#define HEAP_FREE(x) \
	if (x) x = (basic_free_heap(x), NULL)

bool sim_init(struct pendulum_system *system) {
	bool ret = false;
	CWRAPPER_OUTPUT_TYPE res = 0;

	// initialise temp variables
	basic temp, vx, vy, vlx, vly, half, one, t_angvel, t_angle;
	CVecBasic *time_args = NULL;
	CMapBasicBasic *to_func_subs = NULL;
	basic_new_stack(temp);
	basic_new_stack(vx);
	basic_new_stack(vy);
	basic_new_stack(vlx);
	basic_new_stack(vly);
	basic_new_stack(half);
	basic_new_stack(one);
	basic_new_stack(t_angvel);
	basic_new_stack(t_angle);

	// initialise system symbols
	HEAP_ALLOC(system->sym_gravity);
	HEAP_ALLOC(system->ke);
	HEAP_ALLOC(system->gpe);
	HEAP_ALLOC(system->lagrangian);
	HEAP_ALLOC(system->time);

	ASSERT(symbol_set(system->sym_gravity, "g"));
	ASSERT(symbol_set(system->time, "t"));
	basic_const_zero(system->ke);
	basic_const_zero(system->gpe);
	basic_const_zero(vx);
	basic_const_zero(vy);
	ASSERT(rational_set_ui(half, 1, 2));
	basic_const_one(one);

	time_args = vecbasic_new();
	if (!time_args) goto fail;
	vecbasic_push_back(time_args, system->time);

	to_func_subs = mapbasicbasic_new();
	if (!to_func_subs) goto fail;

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// initialise symbols for pendulum
		HEAP_ALLOC(p->sym_mass);
		HEAP_ALLOC(p->sym_length);
		HEAP_ALLOC(p->sym_angle);
		HEAP_ALLOC(p->sym_angvel);
		HEAP_ALLOC(p->func_angle);
		HEAP_ALLOC(p->func_angvel);

		// create unique name for the variable
		unsigned str_size = 16 + log10i(i);
		char str[str_size];
		int printf_res = snprintf(str, str_size, ".%u", i);
		if (printf_res < 0 || printf_res >= str_size) goto fail;

		// define symbols
		str[0] = 'm';
		ASSERT(symbol_set(p->sym_mass, str));
		str[0] = 'l';
		ASSERT(symbol_set(p->sym_length, str));
		str[0] = 'x';
		ASSERT(symbol_set(p->sym_angle, str));
		str[0] = 'v';
		ASSERT(symbol_set(p->sym_angvel, str));

		// define angle and angular velocity functions
		str[0] = 'f';
		ASSERT(function_symbol_set(p->func_angle, str, time_args));
		ASSERT(basic_diff(p->func_angvel, p->func_angle, system->time));

		// define the kinetic energy

		ASSERT(basic_sin(vlx, p->sym_angle));
		ASSERT(basic_cos(vly, p->sym_angle));

		ASSERT(basic_mul(temp, p->sym_length, p->sym_angvel)); // v = rω
		ASSERT(basic_mul(vlx, vlx, temp));
		ASSERT(basic_mul(vly, vly, temp));

		// add velocity vector
		ASSERT(basic_sub(vx, vx, vlx));
		ASSERT(basic_add(vy, vy, vly));

		// compute magnitude^2
		ASSERT(basic_mul(vlx, vx, vx));
		ASSERT(basic_mul(vly, vy, vy));
		ASSERT(basic_add(temp, vlx, vly));

		// multiply by mass and halve
		ASSERT(basic_mul(temp, temp, p->sym_mass));
		ASSERT(basic_mul(temp, temp, half));

		// add value, KE=0.5mv^2
		ASSERT(basic_add(system->ke, system->ke, temp));

		// define the gravitational potential energy

		ASSERT(basic_cos(vly, p->sym_angle)); // cos is in the vertical axis, unlike the unit circle
		ASSERT(basic_sub(vly, one, vly));     // flip vertically
		ASSERT(basic_mul(vly, vly, p->sym_length));
		ASSERT(basic_mul(temp, vly, system->sym_gravity)); // multiply by gravity
		ASSERT(basic_mul(temp, temp, p->sym_mass));        // multiply by mass

		// add value, GPE=mgh
		ASSERT(basic_add(system->gpe, system->gpe, temp));
	}

	// define the Lagrangian function
	ASSERT(basic_sub(system->lagrangian, system->ke, system->gpe)); // L = T (kinetic) - V (potential)

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		mapbasicbasic_insert(to_func_subs, p->sym_angle, p->func_angle);
		mapbasicbasic_insert(to_func_subs, p->sym_angvel, p->func_angvel);
	}

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// partially differentiate Lagrangian function
		ASSERT(basic_diff(t_angle, system->lagrangian, p->sym_angle));
		// note that angular velocity is treated as a separate variable to angle when finding this partial derivative,
		// instead of as the derivative of the angle w.r.t. time
		// see https://math.stackexchange.com/a/2085001
		ASSERT(basic_diff(t_angvel, system->lagrangian, p->sym_angvel));

		// implement Lagrange's equations

		// convert angle and angular velocity into functions of time, so SymEngine doesn't think they're constants and differentiates them to zero
		basic_subs(t_angle, t_angle, to_func_subs);
		basic_subs(t_angvel, t_angvel, to_func_subs);

		// differentiate t_angvel w.r.t. time
		ASSERT(basic_diff(t_angvel, t_angvel, system->time));

		// TODO
		char *str;
		str = basic_str(t_angle);
		printf("∂L/∂q = %s\n\n", str);
		basic_str_free(str);

		str = basic_str(t_angvel);
		printf("d/dt (∂L/∂q̇) = %s\n\n", str);
		basic_str_free(str);
	}

	ret = true;

fail:
	basic_free_stack(temp);
	basic_free_stack(vx);
	basic_free_stack(vy);
	basic_free_stack(vlx);
	basic_free_stack(vly);
	basic_free_stack(half);
	basic_free_stack(one);
	basic_free_stack(t_angvel);
	basic_free_stack(t_angle);

	vecbasic_free(time_args);
	mapbasicbasic_free(to_func_subs);

	if (!ret) {
		if (res) fprintf(stderr, "SymEngine exception %d\n", res);
		sim_free(system);
	}

	return ret;
}

bool sim_free(struct pendulum_system *system) {
	HEAP_FREE(system->sym_gravity);
	HEAP_FREE(system->ke);
	HEAP_FREE(system->gpe);
	HEAP_FREE(system->lagrangian);
	HEAP_FREE(system->time);
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		HEAP_FREE(p->sym_mass);
		HEAP_FREE(p->sym_length);
		HEAP_FREE(p->sym_angle);
		HEAP_FREE(p->sym_angvel);
		HEAP_FREE(p->func_angle);
		HEAP_FREE(p->func_angvel);
	}
	return true;
}

bool sim_step(struct pendulum_system *system) {
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		// testing
		p->angle += 0.05 * (i + 1);
	}
	return true;
}
