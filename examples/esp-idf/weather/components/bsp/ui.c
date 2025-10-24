#include "bsp_board.h"
#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Simple UI log area placed at bottom third of the screen. */
static lv_obj_t *ui_log_container = NULL;
static lv_obj_t *ui_log_label = NULL;
static char ui_log_buf[1024]; /* circular/simple buffer for display */

void ui_init(void)
{
	if (ui_log_container) return; /* already initialized */

	/* Create container occupying bottom third */
	bsp_display_lock(0);
	lv_obj_t *scr = lv_scr_act();
	int hor = lv_disp_get_hor_res(NULL);
	int ver = lv_disp_get_ver_res(NULL);

	int height = ver / 3; /* bottom third */
	ui_log_container = lv_obj_create(scr);
	lv_obj_set_size(ui_log_container, hor, height);
	lv_obj_align(ui_log_container, LV_ALIGN_BOTTOM_MID, 0, 0);
	/* use default background (no explicit color) */
	lv_obj_set_style_border_width(ui_log_container, 0, 0);

	ui_log_label = lv_label_create(ui_log_container);
	/* rely on setting width for wrapping; avoid lv_label_set_long_mode to be compatible across LVGL versions */
	lv_obj_set_width(ui_log_label, hor - 8);
	lv_obj_align(ui_log_label, LV_ALIGN_TOP_LEFT, 4, 4);
	/* use default text color */
	lv_label_set_text(ui_log_label, "");
	bsp_display_unlock();
}

void ui_log_set(const char *txt)
{
	if (!ui_log_label) return;
	bsp_display_lock(0);
	lv_label_set_text(ui_log_label, txt ? txt : "");
	bsp_display_unlock();
}

void ui_log_clear(void)
{
	ui_log_set("");
}

void ui_log_append(const char *fmt, ...)
{
	if (!ui_log_label) return;

	va_list ap;
	va_start(ap, fmt);
	/* append to local buffer, keep it null-terminated */
	int used = 0;
	while (used < (int)sizeof(ui_log_buf) && ui_log_buf[used]) used++;
	int remain = (int)sizeof(ui_log_buf) - used - 1;
	if (remain <= 0) {
		/* buffer full: simple strategy - drop oldest by shifting half */
		int half = (int)sizeof(ui_log_buf) / 2;
		int new_used = used - half;
		if (new_used < 0) new_used = 0;
		for (int i = 0; i < new_used; ++i) ui_log_buf[i] = ui_log_buf[half + i];
		ui_log_buf[new_used] = '\0';
		used = new_used;
		remain = (int)sizeof(ui_log_buf) - used - 1;
	}
	vsnprintf(ui_log_buf + used, remain + 1, fmt, ap);
	va_end(ap);

	bsp_display_lock(0);
	lv_label_set_text(ui_log_label, ui_log_buf);
	bsp_display_unlock();
}
#else
void ui_init(void) {}
void ui_log_set(const char *txt) {(void)txt;}
void ui_log_append(const char *fmt, ...) {(void)fmt;}
void ui_log_clear(void) {}
#endif
