#ifndef SIM_H
#define SIM_H
#include <symengine/cwrapper.h>
#include <stdbool.h>

#ifdef HAVE_SYMENGINE_LLVM
#define USE_LLVM // use LLVM if supported, around 10x faster on my machine
#endif

#ifdef USE_LLVM
#define JIT_TYPE(x) llvm_double_##x
#define VISITOR_TYPE CLLVMDoubleVisitor
#define jit_visitor_init(...) JIT_TYPE(visitor_init)(__VA_ARGS__, 2) // compile with -O2 optimisation flag
#else
#define JIT_TYPE(x) lambda_real_double_##x
#define VISITOR_TYPE CLambdaRealDoubleVisitor
#define jit_visitor_init JIT_TYPE(visitor_init)
#endif

#define jit_visitor_new JIT_TYPE(visitor_new)
#define jit_visitor_call JIT_TYPE(visitor_call)
#define jit_visitor_free JIT_TYPE(visitor_free)

struct pendulum {
	double mass, length, angle, angvel;
	basic_struct *sym_mass, *sym_length, *sym_angle, *sym_angvel, *sym_angacc,
	        *equation_of_motion,
	        *func_angle, *func_angvel, *func_angacc;
};

struct pendulum_system {
	double gravity;
	basic_struct *sym_gravity, *sym_time, *sym_ke, *sym_gpe;
	VISITOR_TYPE *jit_energy,
	        *jit_angacc_solutions;
	unsigned count;
	struct pendulum *chain;
};

struct energy {
	double ke, gpe;
} sim_get_energy(struct pendulum_system *system);

bool sim_init(struct pendulum_system *system);
bool sim_step(struct pendulum_system *system, int steps, double time_span);
bool sim_free(struct pendulum_system *system);
#endif
