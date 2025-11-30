#include "display.h"
#include "util.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define DISPLAY_FD (STDOUT_FILENO)
#define eprintf(...) \
	if (dprintf(DISPLAY_FD, __VA_ARGS__) < 0) return false

static struct termios termios;
bool display_enable() {
	if (tcgetattr(DISPLAY_FD, &termios)) return false;

	struct termios new_termios = termios;
	cfmakeraw(&new_termios);     // stty raw
	new_termios.c_lflag |= ISIG; // allow ^C = SIGINT, etc.

	if (tcsetattr(DISPLAY_FD, TCSANOW, &new_termios)) return false;

	eprintf("\x1b[?1049h"); // move to separate buffer
	eprintf("\x1b[?7l");    // disable newline at end of line
	eprintf("\x1b[?25l");   // hide cursor
	return true;
}

bool display_disable() {
	eprintf("\x1b[?25h");                                       // show cursor
	eprintf("\x1b[?7h");                                        // re-enable newline at end of line
	eprintf("\x1b[?1049l");                                     // restore buffer
	if (tcsetattr(DISPLAY_FD, TCSANOW, &termios)) return false; // restore terminal settings
	return true;
}

static struct rect get_fit_rect(struct pos inner_size, struct pos frame_size) {
	// adapted from https://github.com/sophuric/Foto/blob/51413b512ad7f/src/util.c#L12-L37

	// gets the fit mode rect for an inner and frame size

	float scale;

	// avoid division by zero
	if (inner_size.x == 0.0f || inner_size.y == 0.0f || frame_size.x == 0.0f || frame_size.y == 0.0f)
		return RECT(0, 0, 0, 0);

	// get fit mode rect from inner and frame size
	if (inner_size.x / inner_size.y > frame_size.x / frame_size.y) {
		scale = frame_size.x / inner_size.x;
		return RECT(0, (int) ((frame_size.y * 0.5f) - (inner_size.y * scale * 0.5f)), frame_size.x, (int) (scale * inner_size.y));
	} else {
		scale = frame_size.y / inner_size.y;
		return RECT((int) ((frame_size.x * 0.5f) - (inner_size.x * scale * 0.5f)), 0, (int) (scale * inner_size.x), frame_size.y);
	}
}

static struct pos get_fit_pos(struct pos inner_pos, struct pos inner_size, struct pos frame_size) {
	struct rect rect = get_fit_rect(inner_size, frame_size);
	return pos_add(pos_mul(pos_div(inner_pos, inner_size), rect.size), rect.pos);
};

bool display_render(struct pendulum_chain *chain) {
	eprintf("\x1b[H\x1b[2J"); // clear

	struct winsize term_size;
	if (ioctl(DISPLAY_FD, TIOCGWINSZ, &term_size)) return false; // get terminal size

	struct pos skew = POS(2, 1);

	const static unsigned scale = 2; // adjust code for producing block character if changing this

	size_t term_size_x = term_size.ws_col,
	       term_size_y = term_size.ws_row,
	       size_x = term_size_x * scale,
	       size_y = term_size_y * scale;
	struct pos sizef = POS(size_x, size_y);

	size_t buf_size = ((size_x * size_y) + CHAR_BIT - 1) / CHAR_BIT;

#define INDEX(x, y) ((size_t) (y) * (size_t) (size_x) + (size_t) (x))
#define INDEX_CHAR(x, y) (INDEX(x, y) / CHAR_BIT)
#define INDEX_BIT(x, y) (1 << (INDEX(x, y) % CHAR_BIT)) // returns bit mask

	char *buf = calloc(1, buf_size);

#define SET_BIT(char, mask, set) (set ? (char) | (mask) : (char) &~(mask))
#define GET_CELL(x, y) ((buf[INDEX_CHAR(x, y)] & INDEX_BIT(x, y)) != 0)
#define SET_CELL_NO_CHECK(x, y, set) (buf[INDEX_CHAR(x, y)] = SET_BIT(buf[INDEX_CHAR(x, y)], INDEX_BIT(x, y), set))
#define SET_CELL(x, y, set) \
	if ((x) >= 0 && (y) >= 0 && (x) < (size_x) && (y) < (size_y)) SET_CELL_NO_CHECK(x, y, set)

	float total_length = 0;
	for (size_t i = 0; i < chain->count; ++i) total_length += chain->chain[i].length;

	struct pos pend_t = POS(0, 0);
	float pend_angle = 0;
	for (size_t i = 0; i < chain->count; ++i) {
		struct pendulum *p = &chain->chain[i];
		struct pos pend_f = pend_t;
		pend_angle += p->angle;
		pend_t.x += cos(pend_angle) * p->length;
		pend_t.y += sin(pend_angle) * p->length;

		// draw line
#define CELL_POS(x)                                                                                  \
	get_fit_pos(/* map from [-total_length, +total_length] to [0, skew] */                           \
	            pos_mul(pos_mul(pos_add(pos_div(x, POS2(total_length)), POS2(1)), POS2(0.5)), skew), \
	            skew, sizef) // then letter-box that to the cells size
		struct pos cell_f = CELL_POS(pend_f),
		           cell_t = CELL_POS(pend_t),
		           delta = pos_sub(cell_t, cell_f);
		if (delta.x == 0 && delta.y == 0) break;
		bool swap = fabsf(delta.y) > fabsf(delta.x);
		if (swap) {
#define SWAP(type, a, b) \
	{                    \
		type temp = b;   \
		b = a;           \
		a = temp;        \
	}
#define SWAP_POS(a) SWAP(float, a.x, a.y)
			SWAP_POS(cell_f);
			SWAP_POS(cell_t);
			SWAP_POS(delta);
		}
		float slope = delta.y / delta.x;
		signed sign_x = delta.x < 0 ? -1 : 1;
		for (signed x = floorf(cell_f.x); sign_x * x <= sign_x * ceilf(cell_t.x); x += sign_x) {
			signed y = roundf(slope * (x - cell_f.x) + cell_f.y); // y-y1=m*(x-x1)
			signed px = x;
			if (swap) SWAP(signed, px, y)
			SET_CELL(px, y, 1);
		}
	}

	size_t cursor_x = 0, cursor_y = 0;

	for (size_t term_y = 0; term_y < term_size_y; ++term_y)
		for (size_t term_x = 0; term_x < term_size_x; ++term_x) {
			size_t cell_x = (size_t) term_x * scale,
			       cell_y = (size_t) term_y * scale;

			static char *chars[] = {" ", "▖", "▗", "▄", "▘", "▌", "▚", "▙", "▝", "▞", "▐", "▟", "▀", "▛", "▜", "█"};
			unsigned char index = 0;
			index |= GET_CELL(cell_x + 0, cell_y + 1) ? 1 : 0;
			index |= GET_CELL(cell_x + 1, cell_y + 1) ? 2 : 0;
			index |= GET_CELL(cell_x + 0, cell_y + 0) ? 4 : 0;
			index |= GET_CELL(cell_x + 1, cell_y + 0) ? 8 : 0;
			if (index == 0) continue;
			char *c = chars[index];
			if (term_x != cursor_x || term_y != cursor_y) {
				eprintf("\x1b[%zu;%zuH", term_y + 1, term_x + 1);
				cursor_x = term_x, cursor_y = term_y;
			}
			eprintf("%s", c);
			++cursor_x;
		}

	free(buf);
	fsync(DISPLAY_FD);

	return true;
}
