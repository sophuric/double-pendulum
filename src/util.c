#include "util.h"
struct posf posf_add(struct posf a, struct posf b) { return POSF(a.x + b.x, a.y + b.y); }
struct posf posf_sub(struct posf a, struct posf b) { return POSF(a.x - b.x, a.y - b.y); }
struct posf posf_mul(struct posf a, struct posf b) { return POSF(a.x * b.x, a.y * b.y); }
struct posf posf_div(struct posf a, struct posf b) { return POSF(a.x / b.x, a.y / b.y); }
bool posf_eq(struct posf a, struct posf b) { return a.x == b.x && a.y == b.y; }

struct poss poss_add(struct poss a, struct poss b) { return POSS(a.x + b.x, a.y + b.y); }
struct poss poss_sub(struct poss a, struct poss b) { return POSS(a.x - b.x, a.y - b.y); }
struct poss poss_mul(struct poss a, struct poss b) { return POSS(a.x * b.x, a.y * b.y); }
struct poss poss_div(struct poss a, struct poss b) { return POSS(a.x / b.x, a.y / b.y); }
bool poss_eq(struct poss a, struct poss b) { return a.x == b.x && a.y == b.y; }

struct posf poss2f(struct poss a) { return POSF(a.x, a.y); }
struct poss posf2s(struct posf a) { return POSS(a.x, a.y); }

struct rectf get_fit_rectf(struct posf inner_size, struct rectf frame_rect) {
	// adapted from https://github.com/sophuric/Foto/blob/51413b512ad7f/src/util.c#L12-L37

	// fits the inner rect inside of the frame rect in the center, while maintaining its aspect ratio

	float scale;

	// avoid division by zero
	if (inner_size.x == 0.0f || inner_size.y == 0.0f || frame_rect.w == 0.0f || frame_rect.h == 0.0f)
		return frame_rect;

	if (inner_size.x / inner_size.y > frame_rect.w / frame_rect.h) {
		scale = frame_rect.w / inner_size.x;
		return RECTF(frame_rect.x, frame_rect.y + ((frame_rect.h * 0.5f) - (inner_size.y * scale * 0.5f)), frame_rect.w, (scale * inner_size.y));
	} else {
		scale = frame_rect.h / inner_size.y;
		return RECTF(frame_rect.x + ((frame_rect.w * 0.5f) - (inner_size.x * scale * 0.5f)), frame_rect.y, (scale * inner_size.x), frame_rect.h);
	}
}

struct posf map_rectf(struct posf pos, struct rectf from, struct rectf to) {
	return posf_add(posf_mul(posf_div(posf_sub(pos, from.pos), from.size), to.size), to.pos);
};
