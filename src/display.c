#include "display.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define DISPLAY_FD (STDOUT_FILENO)
#define eprintf(...) \
	if (dprintf(DISPLAY_FD, __VA_ARGS__) < 0) goto fail

#define INDEX(pos, screen) ((size_t) (pos.y) * (size_t) (screen.w) + (size_t) (pos.x))

#define CELL_IN_BOUNDS(pos, screen) ((pos).x >= 0 && (pos).y >= 0 && (pos).x < (screen).w && (pos).y < (screen).h)

#define BIT_SET(char, mask, set) ((set) ? (char) | (mask) : (char) &~(mask))

#define GET_CELL(pos, screen, type) (CELL_IN_BOUNDS(pos, screen) ? ((type *) screen.buf)[INDEX(pos, screen)] : 0)
#define SET_CELL(pos, set, screen, type) \
	if (CELL_IN_BOUNDS(pos, screen))     \
	(((type *) screen.buf)[INDEX(pos, screen)] = set)

bool display_enable(struct display_data *display) {
	if (tcgetattr(DISPLAY_FD, &display->old_termios)) return false;

	if (!display->debug) {
		struct termios new_termios = display->old_termios;
		cfmakeraw(&new_termios);     // stty raw
		new_termios.c_lflag |= ISIG; // allow ^C = SIGINT, etc.

		if (tcsetattr(DISPLAY_FD, TCSANOW, &new_termios)) return false;

		eprintf(
		        "\x1b[?1049h" // move to separate buffer
		        "\x1b[?7l"    // disable newline at end of line
		        "\x1b[?25l"   // hide cursor
		);
	}

	display->screen.buf = NULL;
	display->term.buf = NULL;
	return true;
fail:
	return false;
}

bool display_disable(struct display_data *display) {
	FREE(display->screen.buf);
	FREE(display->term.buf);

	if (!display->debug) {
		eprintf(
		        "\x1b[H"      // move to start
		        "\x1b[2J"     // clear
		        "\x1b[?25h"   // show cursor
		        "\x1b[?7h"    // re-enable newline at end of line
		        "\x1b[?1049l" // restore buffer
		);

		if (tcsetattr(DISPLAY_FD, TCSANOW, &display->old_termios)) return false; // restore terminal settings
	}
	return true;
fail:
	return false;
}

static void draw_line(struct posf pos1, struct posf pos2, bool set, struct display_screen screen) {
	struct posf delta = posf_sub(pos1, pos2);

	bool swap = fabsf(delta.y) > fabsf(delta.x);
	if (swap) { // swap x and y if gradient > 1 (45° from horizontal), otherwise there will be gaps since it loops over x-values
		SWAP_POSF(pos2);
		SWAP_POSF(pos1);
		SWAP_POSF(delta);
	}
	float gradient = delta.x != 0 ? delta.y / delta.x : 0;
	if (delta.x < 0) SWAP(struct posf, pos2, pos1); // swap from/to values to make it easier to loop
	for (size_t x = floorf(pos2.x); x <= ceilf(pos1.x); ++x) {
		struct poss cell = POSS(x, roundf(gradient * (cell.x - pos2.x) + pos2.y)); // y=m*(x-x1)+y1
		if (swap) SWAP_POSS(cell);
		SET_CELL(cell, set, screen, char);
	}
}

static bool init_screen(struct display_screen *screen, size_t item_size, struct poss size, bool *resize, bool *clear) {
	if (resize && (!screen->buf || !poss_eq(screen->size, size))) *resize = true;
	screen->size = size;

	size_t buf_size = screen->w * screen->h * item_size;

	// see https://stackoverflow.com/a/39562813
	if (screen->buf == NULL || buf_size > screen->buf_size) {
		// create new buffer if it doesn't exist yet or we are increasing pixel count
		FREE(screen->buf);
		screen->buf_size = buf_size;
		return (screen->buf = calloc(1, buf_size));
	}
	if (buf_size < screen->buf_size) {
		// realloc if we are decreasing pixel count
		char *buf = realloc(screen->buf, buf_size);
		if (!buf) return false;
		screen->buf = buf;
		screen->buf_size = buf_size;
		memset(screen->buf, 0x00, buf_size);
		return true;
	}
	if (clear) *clear = true;
	return true;
}

bool display_render(struct display_data *display, struct pendulum_system *system) {
	bool res = false;

	eprintf("\x1b[H"); // move to start

	struct winsize ioctl_term_size;
	if (ioctl(DISPLAY_FD, TIOCGWINSZ, &ioctl_term_size)) return false; // get terminal size
	struct poss term_size = POSS(ioctl_term_size.ws_col, ioctl_term_size.ws_row);

	struct posf stretch = POSF(2, 1);

	const static struct poss block_size = POSS(2, 2); // adjust code for producing block character if changing this

	// initialise terminal screen
	bool resize = false, clear = false;
	if (!init_screen(&display->term, sizeof(int), term_size, &resize, NULL)) goto fail;

	// initialise screen on which cells are drawn
	display->screen.size = poss_mul(term_size, block_size);
	if (!init_screen(&display->screen, 1, display->screen.size, &resize, &clear)) goto fail;
	if (clear) memset(display->screen.buf, 0x00, display->screen.buf_size);

	float total_length = 0;
	for (unsigned i = 0; i < system->count; ++i) total_length += system->chain[i].length;

	struct rectf
	        rect_from = RECTF(-total_length, -total_length, total_length * 2, total_length * 2), // max distance the pendulum can reach
	        rect_stretched = RECTF2(POSF2(0), stretch),
	        rect_to = get_fit_rectf(rect_stretched.size, RECTF2(POSF2(0), poss2f(display->screen.size))); // letter-box rect to the display screen size

	struct posf pend_t = POSF(0, 0);
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		struct posf pend_f = pend_t;
		pend_t.x += sin(p->angle) * p->length;
		pend_t.y += cos(p->angle) * p->length;

		// draw line
		struct posf cell_f = map_rectf(pend_f, rect_from, rect_to),
		            cell_t = map_rectf(pend_t, rect_from, rect_to);

		draw_line(cell_f, cell_t, 1, display->screen);
	}

	if (resize) eprintf("\x1b[2J"); // clear on resize
	if (display->info) {
		eprintf("\x1b[H");
		const char *info = display->info;
		const char *newline;
		do {
			newline = strchr(info, '\n');
			size_t nbyte = newline ? newline - info : strlen(info);
			eprintf("\x1b[2K");                                     // clear line
			if (nbyte != write(DISPLAY_FD, info, nbyte)) goto fail; // write up until first newline
			eprintf("\x1b[E");                                      // next line
			info = newline + 1;
		} while (newline);
	}

	struct poss cursor = POSS(-1, 0), term_cell;
	for (term_cell.x = 0; term_cell.x < display->term.w; ++term_cell.x)
		for (term_cell.y = 0; term_cell.y < display->term.h; ++term_cell.y) {
			unsigned char term_char = 0;
			struct poss rel_block, screen_cell = poss_mul(term_cell, block_size);
			for (rel_block.y = 0; rel_block.y < block_size.y; ++rel_block.y)
				for (rel_block.x = 0; rel_block.x < block_size.x; ++rel_block.x) {
					struct poss block = poss_add(screen_cell, rel_block);
					if (!GET_CELL(block, display->screen, char)) continue;
					term_char |= 1 << (rel_block.y * block_size.x + rel_block.x); // set corresponding bit for the block character
				}
			if (!resize && GET_CELL(term_cell, display->term, int) != term_char) {
				SET_CELL(term_cell, term_char, display->term, int);

				static const char *chars[] = {" ", "▘", "▝", "▀", "▖", "▌", "▞", "▛", "▗", "▚", "▐", "▜", "▄", "▙", "▟", "█"};
				const char *c = chars[term_char];
				if (!poss_eq(term_cell, cursor)) {
					eprintf("\x1b[%zu;%zuH%s", term_cell.y + 1, term_cell.x + 1, c);
					cursor = term_cell;
				} else
					eprintf("%s", c);
				++cursor.x;
			}
		}

	res = true;
fail:
	fsync(DISPLAY_FD);
	return res;
}
