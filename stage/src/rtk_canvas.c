/*
 *  STK2 : A GUI toolkit for robotics
 *  Copyright (C) 2001  Andrew Howard  ahoward@usc.edu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Desc: Stk canvas functions
 * Author: Andrew Howard, Richard Vaughan
 */

#if HAVE_CONFIG_H
#include <config.h>
#endif

#define _GNU_SOURCE

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <gdk/gdkkeysyms.h>

#if HAVE_JPEGLIB_H
#include <jpeglib.h>
#endif

//#define DEBUG

#include "rtk.h"
#include "rtkprivate.h"
#include "blinker.h"
#include "gui.h"

// Defined in stage.h/stage.c:
void stg_print_warning(char* m, ...);

// Declare some local functions
static gboolean stg_rtk_on_destroy(GtkWidget *widget, stg_rtk_canvas_t *canvas);
static void stg_rtk_on_configure(GtkWidget *widget, GdkEventConfigure *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_on_expose(GtkWidget *widget, GdkEventExpose *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_on_press(GtkWidget *widget, GdkEventButton *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_on_motion(GtkWidget *widget, GdkEventMotion *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_on_release(GtkWidget *widget, GdkEventButton *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_on_key_press(GtkWidget *widget, GdkEventKey *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_canvas_on_scroll(GtkWidget *widget, GdkEventScroll *event, stg_rtk_canvas_t *canvas);
static void stg_rtk_canvas_mouse(stg_rtk_canvas_t *canvas, int event, int button, int x, int y, unsigned int modmask);

// Mouse modes 
enum {MOUSE_NONE, MOUSE_PAN, MOUSE_ZOOM, MOUSE_TRANS, MOUSE_ROT, MOUSE_SCALE};

// Mouse events
enum {EVENT_PRESS, EVENT_MOTION, EVENT_RELEASE, EVENT_SCROLL};

// Mouse button identifiers. The first three match GDK's button identifiers
enum { MBUTTON_LEFT = 1, MBUTTON_MID = 2, MBUTTON_RIGHT = 3, MBUTTON_SCROLLDOWN, MBUTTON_SCROLLUP, MBUTTON_SCROLLLEFT, MBUTTON_SCROLLRIGHT };

// Mouse event modifiers (just a few from GDK)
enum { MOD_SHIFT = GDK_SHIFT_MASK, MOD_CNTL = GDK_CONTROL_MASK, MOD_ALT1 = GDK_MOD1_MASK };

// Convert from device to logical coords
#define LX(x) (canvas->ox + (+(x) - canvas->sizex / 2) * canvas->sx)
#define LY(y) (canvas->oy + (-(y) + canvas->sizey / 2) * canvas->sy)

// Convert from logical to device coords
#define DX(x) (canvas->sizex / 2 + ((x) - canvas->ox) / canvas->sx)
#define DY(y) (canvas->sizey / 2 - ((y) - canvas->oy) / canvas->sy)


/* Fonts to try */
//#define NUMFONTS 16 
#define NUMFONTS 13 
static const char* fontnames[NUMFONTS] = {
  "-*-monospace-medium-r-*-*-*-140-*-*-*-*-*-*",
  "-*-freemono-medium-r-*-*-*-140-*-*-*-*-*-*",
  "-*-monospace-medium-r-*-*-*-120-*-*-*-*-*-*",
  "-*-freemono-medium-r-*-*-*-120-*-*-*-*-*-*",
  "-*-monospace-medium-r-*-*-*-100-*-*-*-*-*-*",
  "-*-monospace-*-r-*-*-*-120-*-*-*-*-*-*",
  "-*-monospace-*-r-*-*-*-100-*-*-*-*-*-*",
  "-*-freemono-medium-r-*-*-*-100-*-*-*-*-*-*",
  "-*-latin modern sans-*-r-*-*-*-120-*-*-*-*-*-*",
  "-*-monospace-*-r-*-*-*-100-*-*-*-*-*-*",
  "-*-latin modern sans-*-r-*-*-*-100-*-*-*-*-*-*",
  "-*-*-*-r-*-sans-*-120-*-*-*-*-*-*",
  "-*-*-*-r-*-sans-*-100-*-*-*-*-*-*"
/*
  ,
  "-*-*-*-r-*-*-*-120-*-*-*-*-*-*",
  "-*-*-*-r-*-*-*-100-*-*-*-*-*-*",
  "-*-*-*-r-*-*-*-90-*-*-*-*-*-*",
  "-*-*-*-*-*-*-*-120-*-*-*-*-*-*",
  "-*-*-*-*-*-*-*-100-*-*-*-*-*-*",
  "-*-*-*-*-*-*-*-90-*-*-*-*-*-*"
*/
};

gboolean    stest                  (GtkWidget *widget,
				   GdkEventConfigure *event,
				   gpointer user_data)
{
  printf( "EVENT\n" );
  return FALSE;
} 


// Create a canvas
stg_rtk_canvas_t *stg_rtk_canvas_create(stg_rtk_app_t *app)
{
  RTK_F
  stg_rtk_canvas_t *canvas;
  GtkHBox* hbox;
  int i;

  // Create canvas
  canvas = calloc(1, sizeof(stg_rtk_canvas_t));

  // Append canvas to linked list
  STK_LIST_APPEND(app->canvas, canvas);
    
  canvas->app = app;
  canvas->sizex = 0;
  canvas->sizey = 0;
  canvas->ox = 0.0;
  canvas->oy = 0.0;
  canvas->sx = 0.01;
  canvas->sy = 0.01;
  canvas->destroyed = FALSE;
  canvas->bg_dirty = TRUE;
  canvas->fg_dirty = TRUE;
  canvas->fg_dirty_region = stg_rtk_region_create();
  canvas->calc_deferred = 0;
  canvas->movemask = STK_MOVE_PAN | STK_MOVE_ZOOM;

  canvas->fig = NULL;
  canvas->layer_fig = NULL;

  canvas->draw_figs = TRUE;

  // Initialise mouse handling
  canvas->zoom_fig = NULL;
  canvas->mouse_mode = MOUSE_NONE;
  canvas->mouse_over_fig = NULL;  
  canvas->mouse_selected_fig = NULL;
  canvas->mouse_selected_fig_last = NULL;

  // Create a top-level window
  canvas->frame = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  canvas->layout = gtk_vbox_new(FALSE, 0);
  
  // scrolling doesn't work yet, but the widgets are in place - need
  // to change the canvas behaviour quite considerably. May as well
  // wait until a move to a canvas library like gnomecanvas or similar.
  //GtkWidget* scrolled_win = gtk_scrolled_window_new ( NULL, NULL );
  //gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_win),
  //			  GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
  
  //gtk_widget_show( scrolled_win );

 
  hbox = GTK_HBOX(gtk_hbox_new( FALSE, 3 ));
  
  // Create gtk drawing area
  canvas->canvas = gtk_drawing_area_new();
  //gtk_widget_set_size_request( canvas->canvas, 1000,1000 );

  canvas->status_bar = GTK_STATUSBAR(gtk_statusbar_new());
  gtk_statusbar_set_has_resize_grip( canvas->status_bar, TRUE );

  canvas->clock_label = GTK_LABEL(gtk_label_new( "clock" ));
  canvas->timeavg_label = GTK_LABEL(gtk_label_new( "avgtime" ));
  canvas->status_label = GTK_LABEL(gtk_label_new( "stat" ));
  stg_blinker_init(&canvas->status_bar_blinker, 500, 2);
  gdk_color_alloc(canvas->colormap, &canvas->status_bar_alert_color);
  canvas->status_bar_alert_color.red = 0xFFFF;
  canvas->status_bar_alert_color.green = 0x0000;
  canvas->status_bar_alert_color.blue = 0x0000;

  //gtk_scrolled_window_add_with_viewport( scrolled_win, canvas->canvas );
  gtk_widget_show( canvas->canvas );

  
  canvas->perf_bar = GTK_PROGRESS_BAR(gtk_progress_bar_new());
  gtk_progress_bar_set_text( canvas->perf_bar, "speed" );

  canvas->rt_bar = GTK_PROGRESS_BAR(gtk_progress_bar_new());
  gtk_progress_bar_set_text( canvas->rt_bar, "realtime" );

/*   GtkVBox* vbox = gtk_vbox_new( FALSE, 0 ); */
/*   gtk_box_pack_start(GTK_BOX(vbox),  */
/* 		   GTK_WIDGET(canvas->perf_bar), FALSE, FALSE,0); */
/*   gtk_box_pack_start(GTK_BOX(vbox),  */
/* 		   GTK_WIDGET(canvas->rt_bar), FALSE, FALSE,0); */


  // messages window
  if(stg_show_messages_view)
  {
    canvas->messages_text = gtk_text_view_new();
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(canvas->messages_text), GTK_WRAP_WORD);
    gtk_text_view_set_editable(GTK_TEXT_VIEW(canvas->messages_text), 0);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(canvas->messages_text), 0);
    canvas->messages_view = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(canvas->messages_view), GTK_POLICY_AUTOMATIC, GTK_POLICY_ALWAYS);
    gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(canvas->messages_view), canvas->messages_text);
//    gtk_widget_set_size_request(canvas->messages_text,  10, 300);
    gtk_widget_set_size_request(canvas->messages_view,  10, 150);
//    gtk_window_set_default_size(canvas->messages_text, 10, 300);
//    gtk_window_set_default_size(canvas->messages_view, 10, 300);
    gtk_widget_show( canvas->messages_text );
    gtk_widget_show( canvas->messages_view );
  }
  else
  {
    canvas->messages_text = NULL;
    canvas->messages_view = NULL;
  }

  // Put it all together
  gtk_container_add(GTK_CONTAINER(canvas->frame), canvas->layout);

  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(canvas->clock_label), FALSE, FALSE, 5);
  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(canvas->timeavg_label), FALSE, FALSE, 5);
  gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(canvas->status_label), FALSE, FALSE, 5);

/*   gtk_box_pack_start(GTK_BOX(hbox),  */
/* 		   GTK_WIDGET(vbox), FALSE, FALSE, 5); */

  gtk_box_pack_start(GTK_BOX(hbox), 
		   GTK_WIDGET(canvas->status_bar), TRUE, TRUE, 5);

 
  // we'll add these backwards so we can stick the menu in later
  // HBox containing the status bar etc.:
  gtk_box_pack_end(GTK_BOX(canvas->layout), GTK_WIDGET(hbox), FALSE, TRUE, 0);
  // Messages window:
  if(canvas->messages_view)
  {
    gtk_box_pack_end(GTK_BOX(canvas->layout), GTK_WIDGET(canvas->messages_view), FALSE, FALSE, 0);
  }
  // Simulation enviornment view canvas:
  //gtk_box_pack_end(GTK_BOX(canvas->layout), scrolled_win, TRUE, TRUE, 0);
  gtk_box_pack_end(GTK_BOX(canvas->layout), GTK_WIDGET(canvas->canvas), TRUE, TRUE, 0);

  canvas->bg_pixmap = NULL;
  canvas->fg_pixmap = NULL;
  canvas->gc = NULL;
  canvas->colormap = NULL;

  // Set default drawing attributes

  for(i = 0; i < NUMFONTS; ++i)
  {
    const char *fontname = fontnames[i];
    //canvas->fontname = fontnames[i];
    //cont char *fontname = canvas->fontname;
    canvas->font = gdk_font_load(fontname);
    if(canvas->font)
    {
      stg_print_message("Using font: %s", fontname);
      gdk_font_ref(canvas->font);
      break;
    }
  }
  if(!canvas->font)
    stg_print_warning("Could not load any font. Text display may not work.");

  canvas->bgcolor.red = 0xFFFF;
  canvas->bgcolor.green = 0xFFFF;
  canvas->bgcolor.blue = 0xFFFF;
  canvas->linewidth = 0;

  // Movie stuff
  canvas->movie_context = NULL;

  // Connect gtk signal handlers
  g_signal_connect(GTK_OBJECT(canvas->frame), "destroy",
		   GTK_SIGNAL_FUNC(stg_rtk_on_destroy), canvas);
  g_signal_connect(GTK_OBJECT(canvas->canvas), "configure_event", 
		   GTK_SIGNAL_FUNC(stg_rtk_on_configure), canvas);
  g_signal_connect(GTK_OBJECT(canvas->canvas), "expose_event", 
		   GTK_SIGNAL_FUNC(stg_rtk_on_expose), canvas);
  g_signal_connect(GTK_OBJECT(canvas->canvas), "button_press_event", 
		   GTK_SIGNAL_FUNC(stg_rtk_on_press), canvas);
  g_signal_connect(GTK_OBJECT(canvas->canvas), "motion_notify_event", 
		   GTK_SIGNAL_FUNC(stg_rtk_on_motion), canvas);
  g_signal_connect(GTK_OBJECT(canvas->canvas), "button_release_event", 
		   GTK_SIGNAL_FUNC(stg_rtk_on_release), canvas);
  g_signal_connect(GTK_OBJECT(canvas->canvas), "scroll-event", 
       GTK_SIGNAL_FUNC(stg_rtk_canvas_on_scroll), canvas);

  // this seems not to receive the arrow keys...
  g_signal_connect_after(GTK_OBJECT(canvas->frame), "key-press-event", 
			 GTK_SIGNAL_FUNC(stg_rtk_on_key_press), canvas);
  
  // Set the event mask
  gtk_widget_set_events(canvas->canvas,
                        GDK_EXPOSURE_MASK |
                        GDK_BUTTON_PRESS_MASK |
                        GDK_BUTTON_RELEASE_MASK |
			GDK_KEY_PRESS_MASK |
                        GDK_POINTER_MOTION_MASK );

  // rtv - support for motion hints would be handy - todo?
  //GDK_POINTER_MOTION_HINT_MASK);

  /* canvas->gcanvas = gnome_canvas_new_aa(); */
/*   gnome_canvas_set_pixels_per_unit( canvas->gcanvas, 100.0 ); */
/*   gnome_canvas_set_center_scroll_region( canvas->gcanvas, TRUE ); */
/*   gnome_canvas_set_scroll_region ( canvas->gcanvas, 0,0, 10,10); */

  /* gtk_box_pack_end(GTK_BOX(canvas->layout), canvas->gcanvas, TRUE, TRUE, 0); */

 /* white background */
/*   gnome_canvas_item_new(gnome_canvas_root(canvas->gcanvas), */
/*                         gnome_canvas_rect_get_type(), */
/*                         "x1",1.0, */
/*                         "y1",1.0, */
/*                         "x2",100.0, */
/*                         "y2",100.0, */
/*                         "fill_color", "white", */
/*                         NULL); */

  // show all layers by default
  stg_rtk_canvas_all_layers_show(canvas, 1);


  

  // default cursor
  canvas->original_cursor = NULL;

  return canvas;
}

// Delete the canvas
void stg_rtk_canvas_destroy(stg_rtk_canvas_t *canvas)
{
  RTK_F
  int count;

  // Get rid of any figures we still have
  count = 0;
  while (canvas->fig)
  {
    stg_rtk_fig_destroy(canvas->fig);
    count++;
  }
  if (count > 0)
    RTK_PRINT_WARN1("garbage collected %d figures", count);

  // Clear the dirty regions
  stg_rtk_region_destroy(canvas->fg_dirty_region);
  
  // Remove ourself from the linked list in the app
  STK_LIST_REMOVE(canvas->app->canvas, canvas);

  // Destroy the frame
  gtk_widget_hide(GTK_WIDGET(canvas->frame));
  gtk_widget_destroy(GTK_WIDGET(canvas->frame));
  
  // fontname was strdup()ed  (was it? check in init)
  //if (canvas->fontname)
  //  free(canvas->fontname);

  if(canvas->font)
    gdk_font_unref(canvas->font);

  // Free ourself
  free(canvas);

  return;
}

// See if the canvas has been closed
int stg_rtk_canvas_isclosed(stg_rtk_canvas_t *canvas)
{
  RTK_F
  return canvas->destroyed;
}


// Set the canvas title
void stg_rtk_canvas_title(stg_rtk_canvas_t *canvas, const char *title)
{
  RTK_F
  gtk_window_set_title(GTK_WINDOW(canvas->frame), title);
}


// Set the size of a canvas
// (sizex, sizey) is the width and height of the canvas, in pixels.
void stg_rtk_canvas_size(stg_rtk_canvas_t *canvas, int sizex, int sizey)
{
  RTK_F
  gtk_window_set_default_size(GTK_WINDOW(canvas->frame), sizex, sizey);
  return;
}

// Get the canvas size.
// (sizex, sizey) is the width and height of the canvas, in pixels.
void stg_rtk_canvas_get_size(stg_rtk_canvas_t *canvas, int *sizex, int *sizey)
{
  *sizex = canvas->sizex;
  *sizey = canvas->sizey;
  return;
}

// Set the origin of a canvas
// (ox, oy) specifies the logical point that maps to the center of the
// canvas.
void stg_rtk_canvas_origin(stg_rtk_canvas_t *canvas, double ox, double oy)
{
  canvas->ox = ox;
  canvas->oy = oy;

  // Re-calculate all the figures.
  stg_rtk_canvas_calc(canvas);
}


// Get the origin of a canvas
// (ox, oy) specifies the logical point that maps to the center of the
// canvas.
void stg_rtk_canvas_get_origin(stg_rtk_canvas_t *canvas, double *ox, double *oy)
{
  *ox = canvas->ox;
  *oy = canvas->oy;
}


// Scale a canvas
// Sets the pixel width and height in logical units
void stg_rtk_canvas_scale(stg_rtk_canvas_t *canvas, double sx, double sy)
{
  canvas->sx = sx;
  canvas->sy = sy;
  
  // Re-calculate all the figures.
  stg_rtk_canvas_calc(canvas);
}


// Get the scale of the canvas
// (sx, sy) are the pixel with and height in logical units
void stg_rtk_canvas_get_scale(stg_rtk_canvas_t *canvas, double *sx, double *sy)
{
  *sx = canvas->sx;
  *sy = canvas->sy;
}


// Set the movement mask
// Set the mask to a bitwise combination of STK_MOVE_TRANS, STK_MOVE_SCALE.
// to enable user manipulation of the canvas.
void stg_rtk_canvas_movemask(stg_rtk_canvas_t *canvas, int mask)
{
  canvas->movemask = mask;
}


// Set the default font for text strokes
void stg_rtk_canvas_font(stg_rtk_canvas_t *canvas, const char *fontname)
{
  RTK_F
  if (canvas->font)
  {
    gdk_font_unref(canvas->font);
    canvas->font = NULL;
  }

  //if (canvas->fontname)
  //{
  //  free(canvas->fontname);
  //  canvas->fontname = strdup(fontname);
 // }

  // Load the default font
  if (canvas->font == NULL)
    canvas->font = gdk_font_load(fontname);

  // Text extents will have changed, so recalc everything.
  stg_rtk_canvas_calc(canvas);
}


// Set the canvas backround color
void stg_rtk_canvas_bgcolor(stg_rtk_canvas_t *canvas, double r, double g, double b)
{
  canvas->bgcolor.red = (int) (r * 0xFFFF);
  canvas->bgcolor.green = (int) (g * 0xFFFF);
  canvas->bgcolor.blue = (int) (b * 0xFFFF);
  return;
}


// Set the default line width.
void stg_rtk_canvas_linewidth(stg_rtk_canvas_t *canvas, int width)
{
  canvas->linewidth = width;
}

///
// rtv experimental feature
//

// the figure will be shown until stg_rtk_canvas_flash_update() is called
// [duration] times. if [kill] is non-zero, the fig will
// also be destroyed when its counter expires.
void stg_rtk_canvas_flash( stg_rtk_canvas_t* canvas, stg_rtk_fig_t* fig, int duration, 
		       int kill )
{
  RTK_F
  stg_rtk_flasher_t* flasher = malloc( sizeof(stg_rtk_flasher_t) );
  
  // force the fig visible
  stg_rtk_fig_show( fig, 1 );
  
  flasher->fig = fig;
  flasher->duration = duration;
  flasher->kill = kill;
  
  STK_LIST_APPEND( canvas->flashers, flasher );
}

void stg_rtk_canvas_flash_update( stg_rtk_canvas_t* canvas )
{
  RTK_F
  stg_rtk_flasher_t* flasher = canvas->flashers; 
  
  while( flasher != NULL )
    {
      //stg_rtk_fig_t* fig = flasher->fig;      
      flasher->duration--;
      
      // if it's time to flip, flip
      if( flasher->duration < 1 )
	{
	  stg_rtk_flasher_t* doomed = flasher;
	  
	  // force the fig invisible
	  if( doomed->kill )
	    stg_rtk_fig_and_descendents_destroy( doomed->fig );
	  else
	    stg_rtk_fig_show( doomed->fig, 0);
	  
	  flasher = flasher->next;	  
	  STK_LIST_REMOVE( canvas->flashers, doomed );
	  continue;
	}
      
      flasher = flasher->next;
    }
}

void stg_rtk_canvas_layer_show( stg_rtk_canvas_t* canvas, int layer, char show )
{
  canvas->layer_show[layer] = show;
  
  // invalidate the whole window 
  stg_rtk_canvas_calc( canvas );
} 

void stg_rtk_canvas_all_layers_show(stg_rtk_canvas_t* canvas, char show)
{
  int l;
  for(l=0; l<STK_CANVAS_LAYERS; l++ )
    canvas->layer_show[l] = show;
}


// end rtv experimental

// Re-calculate all the figures (private).
void stg_rtk_canvas_calc(stg_rtk_canvas_t *canvas)
{
  RTK_F
  stg_rtk_fig_t *fig;
  
  // The whole window is dirty
  canvas->bg_dirty = TRUE;
  canvas->fg_dirty = TRUE;

  // Add the whole window to the dirty region
  stg_rtk_region_set_union_rect(canvas->fg_dirty_region, 0, 0, canvas->sizex, canvas->sizey);

  // Update all the figures
  for (fig = canvas->fig; fig != NULL; fig = fig->sibling_next)
    stg_rtk_fig_calc(fig);
 
  return;
}

void stg_rtk_canvas_enable(stg_rtk_canvas_t* canvas, gboolean enable)
{
  canvas->draw_figs = enable;
  stg_rtk_canvas_calc(canvas);
  stg_rtk_canvas_render(canvas);
}

// Render the figures in the canvas
void stg_rtk_canvas_render(stg_rtk_canvas_t *canvas)
{
  RTK_F
  //static int c=0;  
  //if( c++ % 20 != 0 )
  //return;

  int bg_count;
  stg_rtk_fig_t *fig;
  //GdkColor color;

  int rcount;
  GdkRectangle clipbox;

  if (canvas->destroyed)
    return;

  // If there were deferred computations, do them now.
  if (canvas->calc_deferred)
  {
    stg_rtk_canvas_calc(canvas);
    canvas->calc_deferred = 0;
  }
  
  // Set the canvas color
  gdk_color_alloc(canvas->colormap, &canvas->bgcolor);
  gdk_gc_set_foreground(canvas->gc, &canvas->bgcolor);
  
  // See if there is anything in the background.
  // TODO: optimize
  if(canvas->draw_figs)
  {
    bg_count = 0;
    for (fig = canvas->layer_fig; fig != NULL; fig = fig->layer_next)
    {
      if (fig->layer <= STK_BACKGROUND_LAYER && fig->show)
      {
        bg_count++;
        break;
      }
    }
  }

  // Render the background
  if (canvas->bg_dirty)
  {
    //printf("rtk: canvas has dirty background, redrawing all.\n");

    // Draw background color pixmap
    gdk_draw_rectangle(canvas->bg_pixmap, canvas->gc, TRUE,
                       0, 0, canvas->sizex, canvas->sizey);


    // Render all background layer shapes:
    if(canvas->draw_figs)
    {
      for (fig = canvas->layer_fig; fig != NULL; fig = fig->layer_next)
      {
        if (fig->layer <= STK_BACKGROUND_LAYER)
          stg_rtk_fig_render(fig);
      }
    }

/*
    static GdkPixbuf *bg_image_pixbuf = NULL;
    // XXX TODO
    // Draw a background image
    // TODO move loading of the file to menu event with user giving position
    // offset (or centered) plus desired size in meters (with option to use map
    // extents)
    // TODO if map has georeference point and image is a geotiff, automatically
    // calucalate size and position.
    // TODO draw with alpha
    // TODO draw before we do the background layer figs above, but make the map fig have a clear
    // background instead of white.
    puts("XXX loading test.png XXX");
    GError *error = NULL;
//    if(bg_image_pixbuf)
//      gdk_pixbuf_delete(bg_image_pixbuf);
    bg_image_pixbuf = gdk_pixbuf_new_from_file_at_scale("/tmp/test.png", canvas->sizex, canvas->sizey, FALSE, &error);
    if(error)
    {
      printf("error loading /tmp/test.png: %s\n", error->message);
    }
    else
    {
      puts("XXX drawingXXX ");
      // TODO recalculate scale based on canvas scale size (sx, sy) (to handle zoom)
      // TODO recalculate position based on canvas origin (ox, oy) (to handle pan)
      gdk_draw_pixbuf(canvas->bg_pixmap, canvas->gc, bg_image_pixbuf, 0, 0, 0, 0, -1, -1, GDK_RGB_DITHER_NONE, 0, 0);
    }
*/

    // Copy background pixmap to foreground pixmap
    gdk_draw_pixmap(canvas->fg_pixmap, canvas->gc, canvas->bg_pixmap,
                    0, 0, 0, 0, canvas->sizex, canvas->sizey);

    // The entire forground needs redrawing
    stg_rtk_region_set_union_rect(canvas->fg_dirty_region,
                             0, 0, canvas->sizex, canvas->sizey);
  }

  // Render the foreground
  if (canvas->bg_dirty || canvas->fg_dirty)
  {
    //printf("rtk: canvas has dirty bg or fg, redrawing dirty region\n");
    
    // Clip drawing to the dirty region
    stg_rtk_region_get_brect(canvas->fg_dirty_region, &clipbox);
    gdk_gc_set_clip_rectangle(canvas->gc, &clipbox);

    // Copy background pixmap to foreground pixmap
    gdk_draw_pixmap(canvas->fg_pixmap, canvas->gc, canvas->bg_pixmap,
                    clipbox.x, clipbox.y, clipbox.x, clipbox.y,
                    clipbox.width, clipbox.height);
    rcount = 0;
    
    // Render all figures, in order of layer
    if(canvas->draw_figs)
    {
      for (fig = canvas->layer_fig; fig != NULL; fig = fig->layer_next)
      {
        // Make sure the figure is in the foreground
        //if (fig->layer >= 0)
        if (fig->layer > STK_BACKGROUND_LAYER && canvas->layer_show[fig->layer] > 0 )
        {
          // Only draw figures in the dirty region
          if (stg_rtk_region_test_intersect(canvas->fg_dirty_region, fig->region))
          {
            //printf("figure %s/%s is in the dirty region\n", fig->username, fig->name);
            rcount++;
            stg_rtk_fig_render(fig);
          }
        }
      }
    }

    // Now copy foreground pixmap to screen
    gdk_draw_pixmap(canvas->canvas->window, canvas->gc, canvas->fg_pixmap,
                    clipbox.x, clipbox.y, clipbox.x, clipbox.y,
                    clipbox.width, clipbox.height);
    gdk_gc_set_clip_rectangle(canvas->gc, NULL);
  }

  // Reset the dirty regions
  canvas->bg_dirty = FALSE;
  canvas->fg_dirty = FALSE;
  stg_rtk_region_set_empty(canvas->fg_dirty_region);

  gdk_colormap_free_colors(canvas->colormap, &canvas->bgcolor, 1);
}


// Export an image.
// [filename] is the name of the file to save.
// [format] is the image file format (STK_IMAGE_FORMAT_JPEG, STK_IMAGE_FORMAT_PPM).
void stg_rtk_canvas_export_image(stg_rtk_canvas_t *canvas, const char *filename, int format)
{
  GdkPixbuf* buf = gdk_pixbuf_get_from_drawable( NULL, 
						  canvas->fg_pixmap,
						  canvas->colormap,
						  0,0,0,0,
						  canvas->sizex,
						  canvas->sizey );
  
  switch( format )
    {
    case STK_IMAGE_FORMAT_JPEG:
      gdk_pixbuf_save( buf, filename, "jpeg", NULL,
		       "quality", "100", NULL );
      break;

    case STK_IMAGE_FORMAT_PPM:
      gdk_pixbuf_save( buf, filename, "ppm", NULL, NULL );
      break;

    case STK_IMAGE_FORMAT_PNG:
      gdk_pixbuf_save( buf, filename, "png", NULL, NULL );
      break;
    case STK_IMAGE_FORMAT_PNM:
      gdk_pixbuf_save( buf, filename, "pnm", NULL, NULL );
      break;

    default: 
      puts( "unrecognized image format" );
      break;
    }
      
}	
  
// Pixel tolerances for moving stuff
#define TOL_MOVE 15

// See if there is a moveable figure close to the given device point.
stg_rtk_fig_t *stg_rtk_canvas_pick_fig(stg_rtk_canvas_t *canvas, int x, int y)
{
  RTK_F
  int maxlayer;
  stg_rtk_fig_t *fig, *maxfig;

  maxfig = NULL;
  maxlayer = INT_MIN;

  // TODO: optimize
  for (fig = canvas->layer_fig; fig != NULL; fig = fig->layer_next)
  {
    if (!fig->show)
      continue;
    if (fig->movemask == 0)
      continue;
    if (stg_rtk_fig_hittest(fig, x, y))
    {
      if (fig->layer >= maxlayer)
      {
        maxfig = fig;
        maxlayer = fig->layer;
      }
    }
  }
  return maxfig;
}

// Do mouse stuff
void stg_rtk_canvas_mouse(stg_rtk_canvas_t *canvas, int event, int button, int x, int y, unsigned int modmask)
{
  RTK_F
  double px, py, pa, rd, rl;
  stg_rtk_fig_t *fig;

  // Use modifiers to simulate right and middle mouse buttons (helpful if on Mac)
  // TODO detect pinch gesture on trackpads for zoom

  if(button == MBUTTON_LEFT)
  { 
    if(modmask & (MOD_SHIFT|MOD_CNTL))
      button = MBUTTON_RIGHT;
    else if(modmask & (MOD_ALT1))
      button = MBUTTON_MID;
  }
    
  if (event == EVENT_PRESS)
  {        
    // See of there are any moveable figures at this point
    canvas->mouse_selected_fig = stg_rtk_canvas_pick_fig(canvas, x, y);

    // If there are moveable figures...
    fig = canvas->mouse_selected_fig;
    if (fig)
    {
      if (button == MBUTTON_LEFT && (fig->movemask & STK_MOVE_TRANS))
      {
        canvas->mouse_mode = MOUSE_TRANS;
        canvas->mouse_start_x = LX(x) - fig->dox;
        canvas->mouse_start_y = LY(y) - fig->doy;
        stg_rtk_fig_dirty(fig);
        stg_rtk_fig_on_mouse(fig, STK_EVENT_PRESS, canvas->mouse_mode);
      }
      else if (button == MBUTTON_RIGHT && (fig->movemask & STK_MOVE_ROT))
      {
        canvas->mouse_mode = MOUSE_ROT;
        px = LX(x) - fig->dox;
        py = LY(y) - fig->doy;
        canvas->mouse_start_a = atan2(py, px) - fig->doa;
        stg_rtk_fig_dirty(fig);
        stg_rtk_fig_on_mouse(fig, STK_EVENT_PRESS, canvas->mouse_mode);
      }

      // rtv - fixed and reinstated scroll wheel support 1/7/03
      // rtv - handle the mouse scroll wheel for rotating objects 
      else if( button == MBUTTON_SCROLLUP && (fig->movemask & STK_MOVE_ROT))
	{
	  stg_rtk_fig_dirty(fig);
	  stg_rtk_fig_origin_global(fig, fig->dox, fig->doy, fig->doa + 0.2 );
	  stg_rtk_fig_on_mouse(fig, STK_EVENT_PRESS, canvas->mouse_mode);
	  
	  return;
	}
      else if( button == MBUTTON_SCROLLDOWN && (fig->movemask & STK_MOVE_ROT))
	{
	  stg_rtk_fig_dirty(fig);
	  stg_rtk_fig_origin_global(fig, fig->dox, fig->doy, fig->doa - 0.2 );
	  stg_rtk_fig_on_mouse(fig, STK_EVENT_PRESS, canvas->mouse_mode);
	  return;
	}
      
    }

    // Else translate and scale the canvas...
    else
    {
      //printf("mouse button=%d (pan=%d zoom=%d)\n", button, canvas->mouse_button_pan, canvas->mouse_button_zoom);
      if (button == canvas->mouse_button_pan && (canvas->movemask & STK_MOVE_PAN))
      {
        // Store the logical coordinate of the start point
        canvas->mouse_mode = MOUSE_PAN;
        canvas->mouse_start_x = LX(x);
        canvas->mouse_start_y = LY(y);
      }
      else if (button == canvas->mouse_button_zoom && (canvas->movemask & STK_MOVE_ZOOM))
      {
	if( canvas->zoom_fig == NULL )
	  {
	    // Store the logical coordinate of the start point
	    canvas->mouse_mode = MOUSE_ZOOM;
	    canvas->mouse_start_x = LX(x);
	    canvas->mouse_start_y = LY(y);
	    
	    // Create a figure for showing the zoom
	    //assert(canvas->zoom_fig == NULL);
	   	    
	    canvas->zoom_fig = stg_rtk_fig_create(canvas, NULL, STK_CANVAS_LAYERS-1, "<zoomcircle>");
	    px = LX(canvas->sizex / 2);
	    py = LY(canvas->sizey / 2);
	    rl = 2 * sqrt((LX(x) - px) * (LX(x) - px) + (LY(y) - py) * (LY(y) - py));
	    stg_rtk_fig_ellipse(canvas->zoom_fig, px, py, 0, rl, rl, 0);
	  }
      }
    }
  }

  if(event == EVENT_SCROLL)
  {
//printf("mouse scroll=%d (pan=%d zoom=%d)\n", button, canvas->mouse_button_pan, canvas->mouse_button_zoom);
    if(button == MBUTTON_SCROLLDOWN && (canvas->movemask & STK_MOVE_ZOOM))
    {
      // wheel down, zoom out
      canvas->sx *= 1.1;
      canvas->sy *= 1.1;
    }
    else if(button == MBUTTON_SCROLLUP && (canvas->movemask & STK_MOVE_ZOOM))
    {
      // wheel up, zoom in
      canvas->sx *= 0.9;
      canvas->sy *= 0.9;
    }
    canvas->calc_deferred ++;
  }

  if (event == EVENT_MOTION)
  {            
//printf("mouse motion mode=%d (trans=%d, rot=%d, pan=%d, zoom=%d)\n", canvas->mouse_mode, MOUSE_TRANS, MOUSE_ROT, MOUSE_PAN, MOUSE_ZOOM);
    if (canvas->mouse_mode == MOUSE_TRANS)
    {
      // Translate the selected figure
      fig = canvas->mouse_selected_fig;
      if(fig) 
      {
        px = LX(x) - canvas->mouse_start_x;
        py = LY(y) - canvas->mouse_start_y;
        stg_rtk_fig_dirty(fig);
        stg_rtk_fig_origin_global(fig, px, py, fig->doa);
        stg_rtk_fig_on_mouse(fig, STK_EVENT_MOTION, canvas->mouse_mode);
      }
    }
    else if (canvas->mouse_mode == MOUSE_ROT)
    {
      // Rotate the selected figure 
      fig = canvas->mouse_selected_fig;
      if(fig)
      {
        px = LX(x) - fig->dox;
        py = LY(y) - fig->doy;
        pa = atan2(py, px) - canvas->mouse_start_a;
        stg_rtk_fig_dirty(fig);
        stg_rtk_fig_origin_global(fig, fig->dox, fig->doy, pa);
        stg_rtk_fig_on_mouse(fig, STK_EVENT_MOTION, canvas->mouse_mode);
      }
    }
    else if (canvas->mouse_mode == MOUSE_PAN)
    {
      // Infer the translation that will map the current physical mouse
      // point to the original logical mouse point.        
      canvas->ox = canvas->mouse_start_x -
        (+x - canvas->sizex / 2) * canvas->sx;
      canvas->oy = canvas->mouse_start_y -
        (-y + canvas->sizey / 2) * canvas->sy;
    }
    else if (canvas->mouse_mode == MOUSE_ZOOM)
    {
      // Compute scale change that will map original logical point
      // onto circle intersecting current mouse point.
      px = x - canvas->sizex / 2;
      py = y - canvas->sizey / 2;
      rd = sqrt(px * px + py * py) + 1;
      px = canvas->mouse_start_x - canvas->ox;
      py = canvas->mouse_start_y - canvas->oy;
      rl = sqrt(px * px + py * py);
      canvas->sy = rl / rd * canvas->sy / canvas->sx;        
      canvas->sx = rl / rd;
    }
    else if (canvas->mouse_mode == MOUSE_NONE)
    {
      // See of there are any moveable figures at this point
      fig = stg_rtk_canvas_pick_fig(canvas, x, y);
      if (fig)
        stg_rtk_fig_dirty(fig);
      if (canvas->mouse_over_fig)
        stg_rtk_fig_dirty(canvas->mouse_over_fig);
      if (fig != canvas->mouse_over_fig)
      {
        if (canvas->mouse_over_fig)
          stg_rtk_fig_on_mouse(canvas->mouse_over_fig, STK_EVENT_MOUSE_NOT_OVER, canvas->mouse_mode);
        if (fig)
          stg_rtk_fig_on_mouse(fig, STK_EVENT_MOUSE_OVER, canvas->mouse_mode);
      }
      canvas->mouse_over_fig = fig;

      if( fig )
	canvas->mouse_selected_fig_last = fig;
    }

    if (canvas->mouse_mode == MOUSE_PAN || canvas->mouse_mode == MOUSE_ZOOM)
    {
      // Re-compute figures (deferred, will be done on next render).
      canvas->calc_deferred++;
    }
  }

  if (event == EVENT_RELEASE)
  {
    if (canvas->mouse_mode == MOUSE_PAN || canvas->mouse_mode == MOUSE_ZOOM)
    {
      // Re-compute figures (deferred, will be done on next render).
      canvas->calc_deferred++;

      // Delete zoom figure
      if (canvas->zoom_fig != NULL)
      {
        stg_rtk_fig_destroy(canvas->zoom_fig);
        canvas->zoom_fig = NULL;
      }
  
      // Reset mouse mode
      canvas->mouse_mode = MOUSE_NONE;
      canvas->mouse_selected_fig = NULL;
    }
    else
    {
      fig = canvas->mouse_selected_fig;
            
      // Reset mouse mode
      canvas->mouse_mode = MOUSE_NONE;
      canvas->mouse_selected_fig = NULL;
      
      // Do callbacks
      if (fig)
      {
        stg_rtk_fig_dirty(fig);
        stg_rtk_fig_on_mouse(fig, STK_EVENT_RELEASE, canvas->mouse_mode);
      }
    }
  }
}


// Handle destroy events
gboolean stg_rtk_on_destroy(GtkWidget *widget, stg_rtk_canvas_t *canvas)
{
  RTK_F
  canvas->destroyed = TRUE;
  return FALSE;
}


// Process configure events
void stg_rtk_on_configure(GtkWidget *widget, GdkEventConfigure *event, stg_rtk_canvas_t *canvas)
{
  RTK_F
  GdkColor color;
  
  //printf( "event width %d height %d\n", event->width, event->height );

  canvas->sizex = event->width;
  canvas->sizey = event->height;

  //printf( "canvas width %d height %d\n", canvas->sizex, canvas->sizey );

  if (canvas->gc == NULL)
    canvas->gc = gdk_gc_new(canvas->canvas->window);
  if (canvas->colormap == NULL)
    canvas->colormap = gdk_colormap_get_system();
  
  // Create offscreen pixmaps
  if (canvas->bg_pixmap != NULL)
    gdk_pixmap_unref(canvas->bg_pixmap);
  if (canvas->fg_pixmap != NULL)
    gdk_pixmap_unref(canvas->fg_pixmap);
  canvas->bg_pixmap = gdk_pixmap_new(canvas->canvas->window,
                                     canvas->sizex, canvas->sizey, -1);
  canvas->fg_pixmap = gdk_pixmap_new(canvas->canvas->window,
                                     canvas->sizex, canvas->sizey, -1);

  // Make sure we redraw with a white background
  gdk_color_white(canvas->colormap, &color);
  gdk_gc_set_foreground(canvas->gc, &color);

  //Clear pixmaps
  gdk_draw_rectangle(canvas->bg_pixmap, canvas->gc, TRUE,
                  0, 0, canvas->sizex, canvas->sizey);
  gdk_draw_rectangle(canvas->fg_pixmap, canvas->gc, TRUE,
                  0, 0, canvas->sizex, canvas->sizey);
    
  // Re-calculate all the figures since the coord transform has
  // changed.
  canvas->calc_deferred++;

}


// Process expose events
void stg_rtk_on_expose(GtkWidget *widget, GdkEventExpose *event, stg_rtk_canvas_t *canvas)
{
  RTK_F
 if (canvas->fg_pixmap)
  {
    // Copy foreground pixmap to screen
    gdk_draw_pixmap(canvas->canvas->window, canvas->gc, canvas->fg_pixmap,
                    0, 0, 0, 0, canvas->sizex, canvas->sizey);
  }
}


// Process keyboard events
void stg_rtk_on_key_press(GtkWidget *widget, GdkEventKey *event, stg_rtk_canvas_t *canvas)
{
  RTK_F
  double scale, dx, dy;
  stg_rtk_fig_t *fig = NULL;

  //PRINT_DEBUG3("key: %d %d %s", event->keyval, event->state, gdk_keyval_name(event->keyval));

  dx = canvas->sizex * canvas->sx;
  dy = canvas->sizey * canvas->sy;
  scale = 1.5;

  
  switch (event->keyval)
    {
    case GDK_Left:
      if (event->state == 0)
        stg_rtk_canvas_origin(canvas, canvas->ox - 0.05 * dx, canvas->oy);
      else if (event->state == 4)
        stg_rtk_canvas_origin(canvas, canvas->ox - 0.5 * dx, canvas->oy);
      break;
    case GDK_Right:
      if (event->state == 0)
        stg_rtk_canvas_origin(canvas, canvas->ox + 0.05 * dx, canvas->oy);
      else if (event->state == 4)
        stg_rtk_canvas_origin(canvas, canvas->ox + 0.5 * dx, canvas->oy);
      break;
    case GDK_Up:
      if (event->state == 0)
        stg_rtk_canvas_origin(canvas, canvas->ox, canvas->oy + 0.05 * dy);
      else if (event->state == 4)
        stg_rtk_canvas_origin(canvas, canvas->ox, canvas->oy + 0.5 * dy);
      else if (event->state == 1)
        stg_rtk_canvas_scale(canvas, canvas->sx / scale, canvas->sy / scale);
      break;
    case GDK_Down:
      if (event->state == 0)
        stg_rtk_canvas_origin(canvas, canvas->ox, canvas->oy - 0.05 * dy);
      else if (event->state == 4)
        stg_rtk_canvas_origin(canvas, canvas->ox, canvas->oy - 0.5 * dy);
      else if (event->state == 1)
        stg_rtk_canvas_scale(canvas, canvas->sx * scale, canvas->sy * scale);
      break;
      
      // TODO: add support for moving objects with the keyboard
    case GDK_w:
    case GDK_W:
      fig = canvas->mouse_selected_fig_last;

      if( fig )
	{
	  stg_rtk_fig_dirty(fig);
	  stg_rtk_fig_origin_global(fig, 
				    fig->dox + 0.04 * cos(fig->doa),
				    fig->doy + 0.04 * sin(fig->doa), 
				    fig->doa );
	}
      break;
      
    case GDK_s:
    case GDK_S:
      fig = canvas->mouse_selected_fig_last;

      if( fig )
	{
	  stg_rtk_fig_dirty(fig);
	  stg_rtk_fig_origin_global(fig, 
				    fig->dox - 0.04 * cos(fig->doa),
				    fig->doy - 0.04 * sin(fig->doa), 
				    fig->doa );
	}
      break;

    case GDK_a:
    case GDK_A:
      fig = canvas->mouse_selected_fig_last;

      if( fig )
	{
	  stg_rtk_fig_dirty(fig);
	  stg_rtk_fig_origin_global(fig, 
				    fig->dox,
				    fig->doy, 
				    fig->doa + 0.1 );
	}
      break;

    case GDK_d:
    case GDK_D:
      fig = canvas->mouse_selected_fig_last;

      if( fig )
	{
	  stg_rtk_fig_dirty(fig);
	  stg_rtk_fig_origin_global(fig, 
				    fig->dox,
				    fig->doy, 
				    fig->doa - 0.1 );
	}
      break;

      
  }
}


// Process mouse press events
void stg_rtk_on_press(GtkWidget *widget, GdkEventButton *event, stg_rtk_canvas_t *canvas)
{
  stg_rtk_canvas_mouse(canvas, EVENT_PRESS, event->button, event->x, event->y, event->state);
}


// Process mouse motion events
void stg_rtk_on_motion(GtkWidget *widget, GdkEventMotion *event, stg_rtk_canvas_t *canvas)
{
  stg_rtk_canvas_mouse(canvas, EVENT_MOTION, 0, event->x, event->y, event->state);
}


// Process mouse release events
void stg_rtk_on_release(GtkWidget *widget, GdkEventButton *event, stg_rtk_canvas_t *canvas)
{
  stg_rtk_canvas_mouse(canvas, EVENT_RELEASE, event->button, event->x, event->y, event->state);
}


void stg_rtk_canvas_on_scroll(GtkWidget *widget, GdkEventScroll *event, stg_rtk_canvas_t *canvas)
{
    int button;
    switch(event->direction)
    {
        case GDK_SCROLL_DOWN: button = MBUTTON_SCROLLDOWN; break;
        case GDK_SCROLL_UP: button = MBUTTON_SCROLLUP; break;
        case GDK_SCROLL_LEFT: button = MBUTTON_SCROLLLEFT; break;
        case GDK_SCROLL_RIGHT: button = MBUTTON_SCROLLRIGHT; break;
    }
    stg_rtk_canvas_mouse(canvas, EVENT_SCROLL, button, event->x, event->y, event->state);
}


int stg_rtk_canvas_is_layer_shown(stg_rtk_canvas_t* canvas, int layer)
{
    return canvas->layer_show[layer];
}

void stg_rtk_canvas_write_message(stg_rtk_canvas_t* canvas, char* msg, char* style)
{
  RTK_F
   if(!stg_show_messages_view)
     return;

   GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (canvas->messages_text));
   int offset;
   GtkAdjustment* adj;

   // Get end iter. The end iter holds the last char index + 1, so we remember
   // the previous index in order to remember the beginning of this insertion in
   // case we need to apply a tag to the inserted text.
   GtkTextIter end_iter;
   GtkTextIter started_insert, ended_insert;
   //printf("[msg] getting end iter for insert\n");
   gtk_text_buffer_get_end_iter(buffer, &end_iter);
   offset = gtk_text_iter_get_offset(&end_iter) - 1;
   if(offset < 0) offset = 0;

   // Insert text at end iterator. -1 means that msg is 0-terminated
   gtk_text_buffer_insert(buffer, &end_iter, msg, -1);

   // Apply tags
   if(style)
   {
     GtkTextTag* tag;

     // Get the new end iterator (end_iter is now invalid) and create an iterator
     // for the beginning of our inserted text, and an iterator that indicates the
     // last character in the buffer, not the abstract "end".
     gtk_text_buffer_get_end_iter(buffer, &end_iter);
     gtk_text_buffer_get_iter_at_offset(buffer, &started_insert, offset);
     gtk_text_buffer_get_iter_at_offset(buffer, &ended_insert, gtk_text_iter_get_offset(&end_iter) );

     tag = gtk_text_tag_table_lookup(gtk_text_buffer_get_tag_table(buffer), style);
     if(!tag)
     {
       // create or error
       PRINT_DEBUG1("message display: creating GTK text tag for style \"%s\"...\n", style);
       if(strcmp(style, "bold") == 0)
         tag = gtk_text_buffer_create_tag(buffer, "bold", "weight", PANGO_WEIGHT_BOLD, NULL); 
       else if(strcmp(style, "red") == 0)
         tag = gtk_text_buffer_create_tag(buffer, "red", "foreground", "red", NULL);
       else if(strcmp(style, "bold red") == 0)
         tag = gtk_text_buffer_create_tag(buffer, "bold red", "foreground", "red", "weight", PANGO_WEIGHT_BOLD, NULL);
       else if(strcmp(style, "underline") == 0)
         tag = gtk_text_buffer_create_tag(buffer, "underline", "underline", PANGO_UNDERLINE_SINGLE, NULL);
       else
         // abort program, you screwed up by supplying an unrecognized style
         // name:
         assert(tag);
     }
     assert(tag);
     gtk_text_buffer_apply_tag(buffer, tag, &started_insert, &ended_insert);
   }

   // Stupid hack to scroll view to the end. Scrolling to the end iterator
   // doesn't work; scrolling to the insert mark doesn't work. WTF?
   // So we just have to set the adjustment on the scroll window manually
   // to an invalid value past the end.
   adj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(canvas->messages_view));
   gtk_adjustment_set_value(adj, (gtk_adjustment_get_value(adj) + 50));

}

