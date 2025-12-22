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

bool display_render(struct display_data *display, bool (*render_func)(struct display_screen screen, void *render_data), void *render_data) {
	bool res = false;

	eprintf("\x1b[H"); // move to start

	struct winsize ioctl_term_size;
	if (ioctl(DISPLAY_FD, TIOCGWINSZ, &ioctl_term_size)) return false; // get terminal size
	struct poss term_size = POSS(ioctl_term_size.ws_col, ioctl_term_size.ws_row);

	const static struct poss block_size = {2, 2}; // adjust code for producing block character if changing this

	// initialise terminal screen
	bool resize = false, clear = false;
	if (!init_screen(&display->term, sizeof(int), term_size, &resize, NULL)) goto fail;

	// initialise screen on which cells are drawn
	display->screen.size = poss_mul(term_size, block_size);
	if (!init_screen(&display->screen, 1, display->screen.size, &resize, &clear)) goto fail;
	if (clear) memset(display->screen.buf, 0x00, display->screen.buf_size);

	if (!render_func(display->screen, render_data)) goto fail;

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
					if (!DISPLAY_GET_CELL(block, display->screen)) continue;
					term_char |= 1 << (rel_block.y * block_size.x + rel_block.x); // set corresponding bit for the block character
				}
			if (!resize && DISPLAY_GET_CELL_TYPE(term_cell, display->term, int) != term_char) {
				DISPLAY_SET_CELL_TYPE(term_cell, term_char, display->term, int);

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
