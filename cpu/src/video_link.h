// The tape video's window in framebuffer pixels (0 size hides it), and the overlay's reset palette and columns.
#ifndef VIDEO_LINK_H
#define VIDEO_LINK_H

void video_window(int x, int y, int w, int h);
void video_fullscreen(void);
void video_palette_reset(void);
void video_overlay_default(void);

#endif
