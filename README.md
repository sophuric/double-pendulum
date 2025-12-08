<h2 align="center">
double pendulum simulation in C<br/><br/>
</h2>

### Dependencies:
- [SymEngine](https://symengine.org/)
  - may depend on [GMP](https://gmplib.org/), [MPFR](https://www.mpfr.org/)
- [LLVM](https://llvm.org/) for SymEngine's Just-in-Time compilation for numerical evaluation 
  - ensure SymEngine is compiled against LLVM, seems to perform ~10x on my machine with this
- `libm`/`<math.h>`
- A terminal that supports ANSI escape codes and [`tcsetattr`](https://linux.die.net/man/3/tcsetattr)
