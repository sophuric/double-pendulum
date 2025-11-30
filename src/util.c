#include "util.h"
struct pos pos_add(struct pos a, struct pos b) { return POS(a.x + b.x, a.y + b.y); }
struct pos pos_sub(struct pos a, struct pos b) { return POS(a.x - b.x, a.y - b.y); }
struct pos pos_mul(struct pos a, struct pos b) { return POS(a.x * b.x, a.y * b.y); }
struct pos pos_div(struct pos a, struct pos b) { return POS(a.x / b.x, a.y / b.y); }
