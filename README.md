<h2 align="center">
<a href="https://en.wikipedia.org/wiki/Lagrangian_mechanics">Lagrangian mechanics</a> system simulation in C<br/><br/>
</h2>

### Examples:
- [Double Pendulum](examples/double-pendulum.c)
- TODO: Add more examples
- TODO: Implement constraints
- TODO: Add GIF here

### Dependencies:
- [SymEngine](https://symengine.org/)
  - may depend on [GMP](https://gmplib.org/), [MPFR](https://www.mpfr.org/)
- [LLVM](https://llvm.org/) for SymEngine's Just-in-Time compilation for numerical evaluation
  - ensure SymEngine is compiled against LLVM, seems to perform ~10x on my machine with this
- `libm`/`<math.h>`
- A terminal that supports ANSI escape codes and [`tcsetattr`](https://linux.die.net/man/3/tcsetattr)
