#include "sim.h"
#include "rk4.h"

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
	basic temp, vx, vy, vlx, vly, height, half, one, t_angvel, t_angle, lagrangian, temp_solution;
	CVecBasic *time_args = NULL,
	          *angacc_system = NULL, *angacc_solution = NULL, *angacc_symbol = NULL,
	          *system_args = NULL, *system_expr = NULL;
	CMapBasicBasic *to_func_subs = NULL, *to_sym_subs = NULL;
	basic_new_stack(temp);
	basic_new_stack(vx);
	basic_new_stack(vy);
	basic_new_stack(vlx);
	basic_new_stack(vly);
	basic_new_stack(height);
	basic_new_stack(half);
	basic_new_stack(one);

	// initialise system symbols
	HEAP_ALLOC(system->sym_gravity);
	HEAP_ALLOC(system->sym_ke);
	HEAP_ALLOC(system->sym_gpe);
	HEAP_ALLOC(system->sym_time);

	ASSERT(symbol_set(system->sym_gravity, "g"));
	ASSERT(symbol_set(system->sym_time, "t"));
	basic_const_zero(system->sym_ke);
	basic_const_zero(system->sym_gpe);
	basic_const_zero(vx);
	basic_const_zero(vy);
	basic_const_zero(height);
	ASSERT(rational_set_ui(half, 1, 2));
	basic_const_one(one);

	time_args = vecbasic_new();
	if (!time_args) goto fail;
	ASSERT(vecbasic_push_back(time_args, system->sym_time));

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// initialise symbols for pendulum
		HEAP_ALLOC(p->sym_mass);
		HEAP_ALLOC(p->sym_length);
		HEAP_ALLOC(p->sym_angle);
		HEAP_ALLOC(p->sym_angvel);
		HEAP_ALLOC(p->sym_angacc);
		HEAP_ALLOC(p->func_angle);
		HEAP_ALLOC(p->func_angvel);
		HEAP_ALLOC(p->func_angacc);

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
		str[0] = 'a';
		ASSERT(symbol_set(p->sym_angacc, str));

		// define angle and angular velocity functions
		str[0] = 'f';
		ASSERT(function_symbol_set(p->func_angle, str, time_args));
		ASSERT(basic_diff(p->func_angvel, p->func_angle, system->sym_time));
		ASSERT(basic_diff(p->func_angacc, p->func_angvel, system->sym_time));

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
		ASSERT(basic_add(system->sym_ke, system->sym_ke, temp));

		// define the gravitational potential energy

		ASSERT(basic_cos(vly, p->sym_angle));                 // cos is in the vertical axis, unlike the unit circle
		ASSERT(basic_sub(vly, one, vly));                     // flip vertically
		ASSERT(basic_mul(vly, vly, p->sym_length));           // multiply by length
		ASSERT(basic_add(height, height, vly));               // add height
		ASSERT(basic_mul(temp, height, system->sym_gravity)); // multiply by gravity
		ASSERT(basic_mul(temp, temp, p->sym_mass));           // multiply by mass

		// add value, GPE=mgh
		ASSERT(basic_add(system->sym_gpe, system->sym_gpe, temp));
	}

	to_func_subs = mapbasicbasic_new();
	if (!to_func_subs) goto fail;

	to_sym_subs = mapbasicbasic_new();
	if (!to_sym_subs) goto fail;

	// define the Lagrangian function
	basic_new_stack(lagrangian);
	ASSERT(basic_sub(lagrangian, system->sym_ke, system->sym_gpe)); // L = T (kinetic) - V (potential)

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		mapbasicbasic_insert(to_func_subs, p->sym_angacc, p->func_angacc);
		mapbasicbasic_insert(to_func_subs, p->sym_angvel, p->func_angvel);
		mapbasicbasic_insert(to_func_subs, p->sym_angle, p->func_angle);

		mapbasicbasic_insert(to_sym_subs, p->func_angacc, p->sym_angacc);
		mapbasicbasic_insert(to_sym_subs, p->func_angvel, p->sym_angvel);
		mapbasicbasic_insert(to_sym_subs, p->func_angle, p->sym_angle);
	}

	basic_new_stack(t_angvel);
	basic_new_stack(t_angle);

	angacc_system = vecbasic_new();
	if (!angacc_system) goto fail;
	angacc_solution = vecbasic_new();
	if (!angacc_solution) goto fail;
	angacc_symbol = vecbasic_new();
	if (!angacc_symbol) goto fail;

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		HEAP_ALLOC(p->equation_of_motion);

		// https://en.wikipedia.org/wiki/Lagrangian_mechanics#Equations_of_motion

		// partially differentiate Lagrangian function
		// these are taken separately for each axis
		ASSERT(basic_diff(t_angle, lagrangian, p->sym_angle));
		// note that angular velocity is treated as a separate variable to angle when finding this partial derivative,
		// instead of as the derivative of the angle w.r.t. time
		// see https://math.stackexchange.com/a/2085001
		ASSERT(basic_diff(t_angvel, lagrangian, p->sym_angvel));

		// implement Lagrange's equations

		// convert into a function of time, so SymEngine doesn't think angle and angular velocity are constants and differentiates them to zero
		basic_subs(t_angvel, t_angvel, to_func_subs);

		// differentiate t_angvel w.r.t. time
		ASSERT(basic_diff(t_angvel, t_angvel, system->sym_time));

		// substitute symbols back in
		basic_subs(t_angvel, t_angvel, to_sym_subs);

		// final equation
		ASSERT(basic_sub(p->equation_of_motion, t_angvel, t_angle));

		// add equation in system of equations to solve for angular acceleration
		ASSERT(vecbasic_push_back(angacc_system, p->equation_of_motion));
		ASSERT(vecbasic_push_back(angacc_symbol, p->sym_angacc));
	}

	// solve system of equations for angular acceleration
	ASSERT(vecbasic_linsolve(angacc_solution, angacc_system, angacc_symbol));

	system_args = vecbasic_new();
	if (!system_args) goto fail;
	ASSERT(vecbasic_push_back(system_args, system->sym_gravity));
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		ASSERT(vecbasic_push_back(system_args, p->sym_mass));
		ASSERT(vecbasic_push_back(system_args, p->sym_length));
		ASSERT(vecbasic_push_back(system_args, p->sym_angle));
		ASSERT(vecbasic_push_back(system_args, p->sym_angvel));
	}

	system_expr = vecbasic_new();
	if (!system_expr) goto fail;
	ASSERT(vecbasic_push_back(system_expr, system->sym_ke));
	ASSERT(vecbasic_push_back(system_expr, system->sym_gpe));

	// JIT compile functions for numerically evaluating energy and each angular acceleration

	// 1= common subexpression elimination
	system->jit_energy = jit_visitor_new();
	if (!system->jit_energy) goto fail;
	jit_visitor_init(system->jit_energy, system_args, system_expr, 1);

	vecbasic_free(system_expr);
	system_expr = vecbasic_new();
	if (!system_expr) goto fail;

	basic_new_stack(temp_solution);
	for (unsigned i = 0; i < system->count; ++i) {
		ASSERT(vecbasic_get(angacc_solution, i, temp_solution));
		ASSERT(vecbasic_push_back(system_expr, temp_solution));
	}

	system->jit_angacc_solutions = jit_visitor_new();
	if (!system->jit_angacc_solutions) goto fail;
	jit_visitor_init(system->jit_angacc_solutions, system_args, system_expr, 1); // 1= common subexpression elimination, 2= compile with -O2

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
	basic_free_stack(lagrangian);
	basic_free_stack(temp_solution);

	vecbasic_free(time_args);
	vecbasic_free(angacc_system);
	vecbasic_free(angacc_solution);
	vecbasic_free(angacc_symbol);
	vecbasic_free(system_args);
	vecbasic_free(system_expr);

	mapbasicbasic_free(to_func_subs);
	mapbasicbasic_free(to_sym_subs);

	if (!ret) {
		if (res) fprintf(stderr, "SymEngine exception %d\n", res);
		sim_free(system);
	}

	return ret;
}

bool sim_free(struct pendulum_system *system) {
	HEAP_FREE(system->sym_gravity);
	HEAP_FREE(system->sym_ke);
	HEAP_FREE(system->sym_gpe);
	HEAP_FREE(system->sym_time);
	jit_visitor_free(system->jit_energy);
	jit_visitor_free(system->jit_angacc_solutions);
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		HEAP_FREE(p->sym_mass);
		HEAP_FREE(p->sym_length);
		HEAP_FREE(p->sym_angle);
		HEAP_FREE(p->sym_angvel);
		HEAP_FREE(p->sym_angacc);
		HEAP_FREE(p->func_angle);
		HEAP_FREE(p->func_angvel);
		HEAP_FREE(p->func_angacc);
		HEAP_FREE(p->equation_of_motion);
	}
	return true;
}

#define JIT_OFFSET (1) // how many variables before first pendulum
#define JIT_VARS (4)   // number of variables per pendulum

static void substitute_jit_args(double *input, struct pendulum_system *system) {
	size_t count = 0;
	input[count++] = system->gravity;
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		input[count++] = p->mass;
		input[count++] = p->length;
		input[count++] = p->angle;
		input[count++] = p->angvel;
	}
}

struct energy sim_get_energy(struct pendulum_system *system) {
	double input[JIT_OFFSET + system->count * JIT_VARS];
	substitute_jit_args(input, system);

	double output[2];
	jit_visitor_call(system->jit_energy, output, input);

	return (struct energy) {.ke = output[0], .gpe = output[1]};
}

#define DYDT_VARS (2) // number of variables per pendulum

static struct pendulum_system *dydt_system;
// static bool dydt_success;
static double *dydt_inputs;

static void dydt(double t, double y[], double out[]) {
	struct pendulum_system *system = dydt_system;

	// substitute variables in
	for (unsigned i = 0; i < system->count; ++i) {
		dydt_inputs[JIT_OFFSET + JIT_VARS * i + 2] = y[i * DYDT_VARS + 0]; // angle
		dydt_inputs[JIT_OFFSET + JIT_VARS * i + 3] = y[i * DYDT_VARS + 1]; // angvel
	}

	double angacc[system->count];
	jit_visitor_call(system->jit_angacc_solutions, angacc, dydt_inputs);

	for (int i = 0; i < system->count; ++i) {
		out[i * DYDT_VARS] = y[i * DYDT_VARS + 1]; // angle changes by angular velocity
		out[i * DYDT_VARS + 1] = angacc[i];        // angular velocity changes by angular acceleration
	}

	// dydt_success = true;
}

bool sim_step(struct pendulum_system *system, int steps, double time_span) {
	if (steps < 1) return false;
	if (time_span <= 0) return false;

	dydt_system = system;
	int variables = system->count * DYDT_VARS;
	double tspan[2] = {0, time_span};
	double y[variables * (steps + 1)];
	double t[steps + 1];

	// copy pendulum data into rk4 input
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		y[i * DYDT_VARS] = p->angle;
		y[i * DYDT_VARS + 1] = p->angvel;
	}

	double inputs[JIT_OFFSET + system->count * JIT_VARS];
	dydt_inputs = inputs; // this is safe because this is only accessed in dydt which is only called here

	// copy pendulum constants into JIT function input
	inputs[0] = system->gravity;
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		inputs[JIT_OFFSET + JIT_VARS * i + 0] = p->mass;
		inputs[JIT_OFFSET + JIT_VARS * i + 1] = p->length;
	}

	// perform Runge-Kutta order 4
	// dydt_success = false;
	rk4(dydt, tspan, y, steps, variables, t, y);
	// if (!dydt_success) return false;

	// copy rk4 output back into pendulum data
	double *final_y = &y[variables * steps];
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		p->angle = final_y[i * DYDT_VARS];
		p->angvel = final_y[i * DYDT_VARS + 1];
	}

	return true;
}
