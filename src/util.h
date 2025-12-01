#ifndef UTIL_H
#define UTIL_H
#include <stddef.h>
#include <stdbool.h>

struct posf {
	float x, y;
};
struct poss {
	size_t x, y;
};

#define POSF(x_, y_) ((struct posf) {.x = x_, .y = y_})
#define POSF2(x_) POSF(x_, x_)
#define POSS(x_, y_) ((struct poss) {.x = x_, .y = y_})
#define POSS2(x_) POSS(x_, x_)

struct posf posf_add(struct posf a, struct posf b);
struct posf posf_sub(struct posf a, struct posf b);
struct posf posf_mul(struct posf a, struct posf b);
struct posf posf_div(struct posf a, struct posf b);
bool posf_eq(struct posf a, struct posf b);

struct poss poss_add(struct poss a, struct poss b);
struct poss poss_sub(struct poss a, struct poss b);
struct poss poss_mul(struct poss a, struct poss b);
struct poss poss_div(struct poss a, struct poss b);
bool poss_eq(struct poss a, struct poss b);

struct posf poss2f(struct poss a);
struct poss posf2s(struct posf a);

struct rectf {
	union {
		struct posf pos;
		struct {
			float x, y;
		};
	};
	union {
		struct posf size;
		struct {
			float w, h;
		};
	};
};

#define RECTF(x_, y_, w_, h_) ((struct rectf) {.x = x_, .y = y_, .w = w_, .h = h_})
#define RECTF2(pos_, size_) ((struct rectf) {.pos = pos_, .size = size_})

#define SWAP(type, a, b)         \
	{                            \
		type temp##__LINE__ = b; \
		b = a;                   \
		a = temp##__LINE__;      \
	}
#define SWAP_POSF(a) SWAP(float, a.x, a.y)
#define SWAP_POSS(a) SWAP(size_t, a.x, a.y)

#define FREE(x) (free(x), x = NULL)

struct rectf get_fit_rectf(struct posf inner_size, struct rectf frame_rect);
struct posf map_rectf(struct posf pos, struct rectf from, struct rectf to);
#endif
