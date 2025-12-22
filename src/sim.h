#ifndef SIM_H
#define SIM_H
#include <stddef.h>
#include <symengine/cwrapper.h>
#include <stdbool.h>

#ifdef HAVE_SYMENGINE_LLVM
#ifndef SIM_NO_USE_LLVM
#define SIM_USE_LLVM // use LLVM if supported, around 10x faster on my machine
#endif
#endif

#ifdef SIM_USE_LLVM
#define SIM_JIT_TYPE(x) llvm_double_##x
#define SIM_VISITOR_TYPE CLLVMDoubleVisitor
#define sim_visitor_init(...) SIM_JIT_TYPE(visitor_init)(__VA_ARGS__, 2) // compile with -O2 optimisation flag
#else
#define SIM_JIT_TYPE(x) lambda_real_double_##x
#define SIM_VISITOR_TYPE CLambdaRealDoubleVisitor
#define sim_visitor_init SIM_JIT_TYPE(visitor_init)
#endif

#define sim_visitor_new SIM_JIT_TYPE(visitor_new)
#define sim_visitor_call SIM_JIT_TYPE(visitor_call)
#define sim_visitor_free SIM_JIT_TYPE(visitor_free)

typedef basic_struct *sim_basic;

struct sim_sym_body_coordinate {
	sim_basic position, velocity;
};

struct sim_num_body_coordinate {
	double position, velocity;
};

struct sim_body {
	size_t coordinates_len, variables_len;

	struct sim_sym_body_coordinate *sym_coordinates;
	sim_basic *sym_variables;

	// these need to be defined (in terms of sym_coordinates, sym_variables, sym_time) before calling sim_compile
	sim_basic sym_kinetic, sym_potential;

	struct sim_num_body_coordinate *coordinates;
	double *in_variables, out_kinetic, out_potential;

	void *custom;

	struct sim_body *prev, *next;
};

struct sim_basic_list {
	sim_basic basic;
	struct sim_basic_list *prev, *next;
};

struct sim_simulation {
	size_t variables_len;

	struct sim_body *bodies, *bodies_last;
	struct sim_basic_list *constraints, *constraints_last;

	sim_basic *sym_variables;
	double *in_variables;

	void *custom;

	// sym_time is used for kinetic/potential energy expressions that depend on time
	sim_basic sym_time;
	// sym_lagrangian is used for constraints, using it for kinetic/potential energy is undefined
	sim_basic sym_lagrangian;

	double *internal_func_args;
	SIM_VISITOR_TYPE *internal_dydt_func, *internal_energy_func;
};

struct sim_simulation *sim_new(CWRAPPER_OUTPUT_TYPE *error, size_t variables_len);
void sim_remove(struct sim_simulation *sim);

// insert_before = NULL to add to end of list
struct sim_basic_list *sim_new_constraint(CWRAPPER_OUTPUT_TYPE *error, struct sim_simulation *sim, sim_basic constraint, struct sim_basic_list *insert_before);
void sim_remove_constraint(struct sim_simulation *sim, struct sim_basic_list *constraint);

struct sim_body *sim_new_body(CWRAPPER_OUTPUT_TYPE *error, struct sim_simulation *sim, size_t coordinates_len, size_t variables_len, struct sim_body *insert_before);
void sim_remove_body(struct sim_simulation *sim, struct sim_body *body);

// must be called before sim_step and after the last sim_[new/remove]_[body/constraint] call
// sym_kinetic and sym_potential must be defined for all bodies prior to calling this
bool sim_compile(CWRAPPER_OUTPUT_TYPE *error, struct sim_simulation *sim);

bool sim_step(struct sim_simulation *system, int steps, double time_span);
#endif
