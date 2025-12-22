#include "sim.h"
#include "rk4.h"
#include "util.h"
#include "linked_list.h"
#include <stdint.h>

static unsigned log10i(size_t x) {
	unsigned i;
	for (i = 1; x >= 10; x /= 10) ++i;
	return i;
}

static void sim_remove_unlinked_body(struct sim_body *body) {
	if (!body) return;
	// removes body without modifying prev/next

	BASIC_FREE(body->sym_kinetic);
	BASIC_FREE(body->sym_potential);

	if (body->sym_variables)
		for (size_t i = 0; i < body->variables_len; ++i)
			BASIC_FREE(body->sym_variables[i]);

	if (body->sym_coordinates)
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			BASIC_FREE(body->sym_coordinates[i].position);
			BASIC_FREE(body->sym_coordinates[i].velocity);
		}

	free(body->sym_variables);
	free(body->sym_coordinates);
	free(body->in_variables);
	free(body->coordinates);
	free(body);
}

// helper macro to assert that snprintf succeeds
#define SNPRINTF(str, size, ...)                                    \
	{                                                               \
		int snprintf_temp_var = snprintf(str, size, __VA_ARGS__);   \
		ASSERT(snprintf_temp_var >= 0 && snprintf_temp_var < size); \
	}

struct sim_simulation *sim_new(CWRAPPER_OUTPUT_TYPE *error, size_t variables_len) {
	// buffer string for defining symbols
	const size_t str_length = 16 + log10i(SIZE_MAX);
	char str[str_length];

	CWRAPPER_OUTPUT_TYPE sym_error = 0;

	struct sim_simulation *sim = calloc(1, sizeof(struct sim_simulation));
	ASSERT(sim);

	// allocate arrays

	sim->variables_len = variables_len;
	ASSERT(sim->sym_variables = calloc(variables_len, sizeof(*sim->sym_variables)));
	ASSERT(sim->in_variables = calloc(variables_len, sizeof(*sim->in_variables)));

	for (size_t i = 0; i < variables_len; ++i) {
		sim_basic *c = &sim->sym_variables[i];
		BASIC_NEW(*c);
		SNPRINTF(str, str_length, "sim_var_%p", (void *) *c);
		ASSERT_SYM(symbol_set(*c, str));
	}
	for (size_t i = 0; i < variables_len; ++i) sim->in_variables[i] = 0.0;

	BASIC_NEW(sim->sym_time);
	ASSERT_SYM(symbol_set(sim->sym_time, "t"));

	BASIC_NEW(sim->sym_lagrangian);
	ASSERT_SYM(symbol_set(sim->sym_lagrangian, "L"));

	return sim;

fail:
	if (sym_error) *error = sym_error;
	sim_remove(sim);
	return NULL;
}

static void sim_remove_unlinked_constraint(struct sim_basic_list *constraint) {
	if (!constraint) return;
	// removes constraint without modifying prev/next
	BASIC_FREE(constraint->basic);
	free(constraint);
}

void sim_remove(struct sim_simulation *sim) {
	if (!sim) return;

	struct sim_body *remove_body;
	LL_REMOVE_ALL(sim->bodies, remove_body, sim_remove_unlinked_body(remove_body));

	struct sim_basic_list *remove_constraint;
	LL_REMOVE_ALL(sim->constraints, remove_constraint, sim_remove_unlinked_constraint(remove_constraint));

	if (sim->sym_variables)
		for (size_t i = 0; i < sim->variables_len; ++i)
			BASIC_FREE(sim->sym_variables[i]);

	// free visitor functions
	if (sim->internal_dydt_func) sim_visitor_free(sim->internal_dydt_func);
	if (sim->internal_energy_func) sim_visitor_free(sim->internal_energy_func);

	BASIC_FREE(sim->sym_time);
	BASIC_FREE(sim->sym_lagrangian);

	free(sim->in_variables);
	free(sim->sym_variables);
	free(sim->internal_func_args);
	free(sim);
}

struct sim_basic_list *sim_new_constraint(CWRAPPER_OUTPUT_TYPE *error, struct sim_simulation *sim, sim_basic constraint, struct sim_basic_list *insert_before) {
	CWRAPPER_OUTPUT_TYPE sym_error = 0;

	struct sim_basic_list *con = calloc(1, sizeof(*con));
	ASSERT(con);

	BASIC_NEW(con->basic);
	ASSERT_SYM(basic_assign(con->basic, constraint));

	// add to linked list
	LL_INSERT_BEFORE(con, sim->constraints, sim->constraints_last, insert_before);

	return con;

fail:
	sim_remove_unlinked_constraint(con);
	return NULL;
}

void sim_remove_constraint(struct sim_simulation *sim, struct sim_basic_list *constraint) {
	if (!constraint) return;
	LL_REMOVE(constraint, sim->constraints, sim->constraints_last);
	sim_remove_unlinked_constraint(constraint);
}

struct sim_body *sim_new_body(CWRAPPER_OUTPUT_TYPE *error, struct sim_simulation *sim, size_t coordinates_len, size_t variables_len, struct sim_body *insert_before) {
	// buffer string for defining symbols
	const size_t str_length = 16 + log10i(SIZE_MAX);
	char str[str_length];

	CWRAPPER_OUTPUT_TYPE sym_error = 0;

	struct sim_body *body = calloc(1, sizeof(*body));
	ASSERT(body);

	body->out_kinetic = 0.0, body->out_potential = 0.0;
	body->coordinates_len = coordinates_len;
	body->variables_len = variables_len;
	body->variables_len = variables_len;

	BASIC_NEW(body->sym_kinetic);
	BASIC_NEW(body->sym_potential);

	// allocate arrays

	ASSERT(body->sym_variables = calloc(variables_len, sizeof(body->sym_variables)));
	for (size_t i = 0; i < variables_len; ++i) {
		sim_basic *c = &body->sym_variables[i];
		BASIC_NEW(*c);
		SNPRINTF(str, str_length, "body_var_%p", (void *) *c);
		ASSERT_SYM(symbol_set(*c, str));
	}

	ASSERT(body->in_variables = calloc(variables_len, sizeof(body->in_variables)));
	for (size_t i = 0; i < variables_len; ++i) body->in_variables[i] = 0.0;

	ASSERT(body->sym_coordinates = calloc(coordinates_len, sizeof(body->sym_coordinates)));
	for (size_t i = 0; i < coordinates_len; ++i) {
		struct sim_sym_body_coordinate *c = &body->sym_coordinates[i];

		// define symbols

		BASIC_NEW(c->position);
		SNPRINTF(str, str_length, "pos_%p", (void *) c);
		ASSERT_SYM(symbol_set(c->position, str));

		BASIC_NEW(c->velocity);
		SNPRINTF(str, str_length, "vel_%p", (void *) c);
		ASSERT_SYM(symbol_set(c->velocity, str));
	}

	ASSERT(body->coordinates = calloc(coordinates_len, sizeof(struct sim_num_body_coordinate)));
	for (size_t i = 0; i < coordinates_len; ++i) body->coordinates[i] = (struct sim_num_body_coordinate) {.position = 0.0, .velocity = 0.0};

	// add to linked list
	LL_INSERT_BEFORE(body, sim->bodies, sim->bodies_last, insert_before);
	return body;
fail:
	if (sym_error) *error = sym_error;
	sim_remove_unlinked_body(body);
	return NULL;
}

void sim_remove_body(struct sim_simulation *sim, struct sim_body *body) {
	if (!body) return;
	LL_REMOVE(body, sim->bodies, sim->bodies_last);
	sim_remove_unlinked_body(body);
}

bool sim_compile(CWRAPPER_OUTPUT_TYPE *error, struct sim_simulation *sim) {
	// buffer string for defining symbols
	const size_t str_length = 16 + log10i(SIZE_MAX);
	char str[str_length];
	bool res = false;

	CWRAPPER_OUTPUT_TYPE sym_error = 0;

	free(sim->internal_func_args);
	sim->internal_func_args = NULL;

	// free visitor functions
	sim_visitor_free(sim->internal_dydt_func);
	sim_visitor_free(sim->internal_energy_func);
	sim->internal_dydt_func = NULL;
	sim->internal_energy_func = NULL;

	// initialise variables

	CVecBasic *visitor_args = NULL, *system_equations = NULL, *acc_solutions = NULL, *acc_vars = NULL, *dydt_output = NULL, *energy_output = NULL, *time_args = NULL;
	CMapBasicBasic *to_func_subs = NULL, *to_sym_subs = NULL;
	sim_basic lagrangian = NULL, temp = NULL, temp2 = NULL;
	size_t coordinates_len = 0;

	BASIC_NEW(temp);
	BASIC_NEW(temp2);

	// initialise time arguments
	ASSERT(time_args = vecbasic_new());
	ASSERT_SYM(vecbasic_push_back(time_args, sim->sym_time));

	// initialise args for visitor functions
	ASSERT(visitor_args = vecbasic_new());

	// add simulation variables
	for (size_t i = 0; i < sim->variables_len; ++i)
		ASSERT_SYM(vecbasic_push_back(visitor_args, sim->sym_variables[i]));

	LL_LOOP(struct sim_body *, body, sim->bodies) {
		// add body other variables
		for (size_t i = 0; i < body->variables_len; ++i) ASSERT_SYM(vecbasic_push_back(visitor_args, body->sym_variables[i]));
	}

	LL_LOOP(struct sim_body *, body, sim->bodies) {
		// add body coordinates
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			++coordinates_len;
			ASSERT_SYM(vecbasic_push_back(visitor_args, body->sym_coordinates[i].position));
			ASSERT_SYM(vecbasic_push_back(visitor_args, body->sym_coordinates[i].velocity));
		}
	}

	// initialise args array for calling visitor functions
	ASSERT(sim->internal_func_args = calloc(vecbasic_size(visitor_args), sizeof(*sim->internal_func_args)));

	// initialise map to substitute variables with their function of time variables
	ASSERT(to_func_subs = mapbasicbasic_new());
	ASSERT(to_sym_subs = mapbasicbasic_new());
	ASSERT(acc_vars = vecbasic_new());
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			struct sim_sym_body_coordinate *coordinate = &body->sym_coordinates[i];

			SNPRINTF(str, str_length, "func_%p", (void *) coordinate);
			ASSERT_SYM(function_symbol_set(temp, str, time_args)); // define position as a function of time
			mapbasicbasic_insert(to_func_subs, coordinate->position, temp);
			mapbasicbasic_insert(to_sym_subs, temp, coordinate->position);

			ASSERT_SYM(basic_diff(temp, temp, sim->sym_time)); // velocity
			mapbasicbasic_insert(to_func_subs, coordinate->velocity, temp);
			mapbasicbasic_insert(to_sym_subs, temp, coordinate->velocity);

			SNPRINTF(str, str_length, "acc_%p", (void *) coordinate);
			ASSERT_SYM(symbol_set(temp2, str));
			ASSERT_SYM(vecbasic_push_back(acc_vars, temp2));

			ASSERT_SYM(basic_diff(temp, temp, sim->sym_time)); // acceleration
			mapbasicbasic_insert(to_func_subs, temp2, temp);
			mapbasicbasic_insert(to_sym_subs, temp, temp2);
		}
	}

	BASIC_NEW(lagrangian);
	basic_const_zero(lagrangian);
	// L = T - V
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		ASSERT_SYM(basic_add(lagrangian, lagrangian, body->sym_kinetic));
		ASSERT_SYM(basic_sub(lagrangian, lagrangian, body->sym_potential));
	}

	// TODO: constraint variables

	// create equations of motion
	ASSERT(system_equations = vecbasic_new());
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			struct sim_sym_body_coordinate *coordinate = &body->sym_coordinates[i];

			// https://en.wikipedia.org/wiki/Lagrangian_mechanics#Equations_of_motion

			// partially differentiate Lagrangian function
			// these are taken separately for each coordinate
			ASSERT_SYM(basic_diff(temp, lagrangian, coordinate->position)); // ∂L/∂q

			// note that velocity is treated as a separate variable to position when finding this partial derivative,
			// instead of as the derivative of the position w.r.t. time
			// see https://math.stackexchange.com/a/2085001
			ASSERT_SYM(basic_diff(temp2, lagrangian, coordinate->velocity)); // ∂L/∂q̇

			// convert into a function of time, so SymEngine doesn't think angle and angular velocity are constants and differentiates them to zero
			ASSERT_SYM(basic_subs(temp2, temp2, to_func_subs));
			// differentiate w.r.t. time
			ASSERT_SYM(basic_diff(temp2, temp2, sim->sym_time)); // d/dt (∂L/∂q̇)
			ASSERT_SYM(basic_subs(temp2, temp2, to_sym_subs));

			ASSERT_SYM(basic_sub(temp, temp, temp2));

			// TODO: constraint variables

			// add equation of motion as an equation to solve
			ASSERT_SYM(vecbasic_push_back(system_equations, temp));
		}
	}

	// solve for acceleration
	ASSERT(acc_solutions = vecbasic_new());
	ASSERT_SYM(vecbasic_linsolve(acc_solutions, system_equations, acc_vars));

	// initialise output for time derivative visitor function
	ASSERT(dydt_output = vecbasic_new());
	size_t j = 0;
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			struct sim_sym_body_coordinate *coordinate = &body->sym_coordinates[i];
			ASSERT_SYM(vecbasic_push_back(dydt_output, coordinate->velocity)); // dposition/dtime = velocity
			ASSERT_SYM(vecbasic_get(acc_solutions, j++, temp));                // get acceleration
			ASSERT_SYM(vecbasic_push_back(dydt_output, temp));                 // dvelocity/dtime = acceleration
		}
	}

	// compile time derivative visitor function
	ASSERT(sim->internal_dydt_func = sim_visitor_new());
	sim_visitor_init(sim->internal_dydt_func, visitor_args, dydt_output, 1);

	// initialise output for energy visitor function
	ASSERT(energy_output = vecbasic_new());
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		ASSERT_SYM(vecbasic_push_back(energy_output, body->sym_kinetic));
		ASSERT_SYM(vecbasic_push_back(energy_output, body->sym_potential));
	}

	// compile energy visitor function
	ASSERT(sim->internal_energy_func = sim_visitor_new());
	sim_visitor_init(sim->internal_energy_func, visitor_args, energy_output, 1);

	res = true;
fail:
	// free everything
	BASIC_FREE(lagrangian);
	BASIC_FREE(temp);
	BASIC_FREE(temp2);
	vecbasic_free(visitor_args);
	vecbasic_free(system_equations);
	vecbasic_free(acc_solutions);
	vecbasic_free(acc_vars);
	vecbasic_free(dydt_output);
	vecbasic_free(energy_output);
	vecbasic_free(time_args);
	mapbasicbasic_free(to_func_subs);
	mapbasicbasic_free(to_sym_subs);

	if (res) return true;
	sim_visitor_free(sim->internal_dydt_func);
	sim_visitor_free(sim->internal_energy_func);
	sim->internal_dydt_func = NULL;
	sim->internal_energy_func = NULL;
	if (sym_error) *error = sym_error;
	return false;
}

struct dydt_data {
	struct sim_simulation *simulation;
	size_t coordinates_start_index;
};

static void dydt(double t, double y[], double out[], void *custom) {
	struct dydt_data *data = custom;
	struct sim_simulation *sim = data->simulation;

	size_t rk4_i = 0, arg_i = data->coordinates_start_index;

	// copy body coordinates to visitor arguments
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		// copy body coordinates to rk4 variables
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			sim->internal_func_args[arg_i++] = y[rk4_i++]; // position
			sim->internal_func_args[arg_i++] = y[rk4_i++]; // velocity
		}
	}

	// run ODE function
	sim_visitor_call(sim->internal_dydt_func, out, sim->internal_func_args);
}

bool sim_step(struct sim_simulation *sim, int steps, double time_span) {
	if (steps < 1) return false;
	if (time_span <= 0) return false;

	size_t arg_i = 0;

	size_t rk4_len = 0; // number of coordinates to iterate through

	size_t body_len = 0;
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		++body_len;
		rk4_len += body->coordinates_len * 2;
		if (rk4_len < 0) return false; // overflow
	}

	// initialise rk4 variables

	double tspan[2] = {0, time_span};

	double rk4_coordinates[rk4_len * (steps + 1)];
	double time_out[steps + 1];

	// initialise visitor variables

	for (size_t i = 0; i < sim->variables_len; ++i)
		sim->internal_func_args[arg_i++] = sim->in_variables[i];

	size_t rk4_i = 0;
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		for (size_t i = 0; i < body->variables_len; ++i)
			sim->internal_func_args[arg_i++] = body->in_variables[i];
		// skip setting position and velocity coordinates for internal_func_args, that is done in the dydt function

		// copy body coordinates to rk4 variables
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			rk4_coordinates[rk4_i++] = body->coordinates[i].position;
			rk4_coordinates[rk4_i++] = body->coordinates[i].velocity;
		}
	}

	// perform Runge-Kutta order 4
	struct dydt_data data = {
	        .simulation = sim,
	        .coordinates_start_index = arg_i};
	rk4(dydt, tspan, rk4_coordinates, steps, rk4_len, time_out, rk4_coordinates, &data);

	rk4_i = rk4_len * steps; // index of last set of coordinates
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		for (size_t i = 0; i < body->coordinates_len; ++i) {
			// copy coordinates into visitor arguments again to calculate energy values
			sim->internal_func_args[arg_i++] = rk4_coordinates[rk4_i];     // position
			sim->internal_func_args[arg_i++] = rk4_coordinates[rk4_i + 1]; // velocity
			// and copy back into bodies
			body->coordinates[i].position = rk4_coordinates[rk4_i++]; // position
			body->coordinates[i].velocity = rk4_coordinates[rk4_i++]; // velocity
		}
	}

	// perform energy calculations
	double energy[2 * body_len];
	sim_visitor_call(sim->internal_energy_func, energy, sim->internal_func_args);

	// copy energy numbers into bodies
	arg_i = 0;
	LL_LOOP(struct sim_body *, body, sim->bodies) {
		body->out_kinetic = energy[arg_i++];
		body->out_potential = energy[arg_i++];
	}

	return true;
}
