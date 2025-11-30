#ifndef UTIL_H
#define UTIL_H
struct pos {
	float x, y;
};
#define POS(x_, y_) ((struct pos) {.x = x_, .y = y_})
#define POS2(x_) POS(x_, x_)
struct pos pos_add(struct pos a, struct pos b);
struct pos pos_sub(struct pos a, struct pos b);
struct pos pos_mul(struct pos a, struct pos b);
struct pos pos_div(struct pos a, struct pos b);

struct rect {
	union {
		struct pos pos;
		struct {
			float x, y;
		};
	};
	union {
		struct pos size;
		struct {
			float w, h;
		};
	};
};
#define RECT(x_, y_, w_, h_) ((struct rect) {.pos = POS(x_, y_), .size = POS(w_, h_)})
#endif
