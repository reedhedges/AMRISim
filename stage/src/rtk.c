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
 * Desc: Stk application functions
 * Author: Andrew Howard
 * CVS: $Id$
 */

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef HAVE_PTHREAD_H // have pthread.h and thread locking enabled
#include <pthread.h>
#endif
#include <signal.h>
#include <gdk/gdkkeysyms.h>

//#define DEBUG
#include "rtk.h"
#include "rtkprivate.h"
#include "stage_internal.h"


// Dummy symbol so that autoconf can verify the library version number
// using AC_CHECK_LIB.
int LIBSTK_VERSION_2_2(void) { return(0); }

char rtk_last_function[128];

// Declare some local functions
//static int stg_rtk_app_on_timer(stg_rtk_app_t *app);

// Initialise the library.

int stg_rtk_initxx(int *argc, char ***argv)
{
  RTK_F_INIT

  // Initialise the gtk lib
  gtk_init(argc, argv);

  // Allow rgb image handling
  gdk_rgb_init();

#ifdef ENABLE_AVCODEC
  // Allow movie capture
  avcodec_init();
  avcodec_register_all();
#endif

  return 0;
}


// Create an app
stg_rtk_app_t *stg_rtk_app_create()
{
  RTK_F

  stg_rtk_app_t *app;

  app = malloc(sizeof(stg_rtk_app_t));
  app->must_quit = FALSE;
  app->has_quit = FALSE;
  app->canvas = NULL;
  app->table = NULL;

  return app;
}


// Destroy the app
void stg_rtk_app_destroy(stg_rtk_app_t *app)
{
  RTK_F

  int count;

  // Get rid of any canvases we still have
  count = 0;
  while (app->canvas)
  {
    stg_rtk_canvas_destroy(app->canvas);
    count++;
  }
  if (count > 0)
    RTK_PRINT_WARN1("garbage collected %d canvases", count);

  // Get rid of any tables  we still have
  //count = 0;
  //while (app->table)
  //{
  //stg_rtk_table_destroy(app->table);
  //count++;
  //}
  //if (count > 0)
  //RTK_PRINT_WARN1("garbage collected %d tables", count);

  // Free ourself
  free(app);
}


// Check to see if its time to quit
int stg_rtk_app_quit(stg_rtk_app_t *app)
{
  RTK_F

  return (app->has_quit);
}


// Main loop -- run in own thread
int stg_rtk_app_main(stg_rtk_app_t *app)
{
  RTK_F

  stg_rtk_app_main_init(app);
	gtk_main();
  stg_rtk_app_main_term(app);
  
  return 0;
}


// Do the initial main loop stuff
void stg_rtk_app_main_init(stg_rtk_app_t *app)
{
  RTK_F

  stg_rtk_canvas_t *canvas;
  //stg_rtk_table_t *table;
  
  // Display everything
  for (canvas = app->canvas; canvas != NULL; canvas = canvas->next)
    gtk_widget_show_all(canvas->frame);
  //for (table = app->table; table != NULL; table = table->next)
  //gtk_widget_show_all(table->frame);

  return;
}


// Do the final main loop stuff
void stg_rtk_app_main_term(stg_rtk_app_t *app)
{
  RTK_F

  // Process remaining events
  while (gtk_events_pending())
    gtk_main_iteration();

  // Note that the app has quit
  app->has_quit = TRUE;

  return;
}


// Event processing function.  Returns non-zero if the app should quit.
int stg_rtk_app_main_loop(stg_rtk_app_t *app)
{
  RTK_F
  STG_F()
  int ret;
  stg_rtk_canvas_t *canvas;
  //stg_rtk_table_t *table;
  
  STG_XF("gtk_events_pending");
  while (gtk_events_pending())
  {
    STG_XF("gtk_main_iteration");
    ret = gtk_main_iteration();
    STG_XF("gtk_events_pending");
  }
  STG_F();

  // Quit the app if we have been told we should
  // We first destroy in windows that are still open.
  if (app->must_quit)
  {
    for (canvas = app->canvas; canvas != NULL; canvas = canvas->next)
      if (!canvas->destroyed)
        gtk_widget_destroy(canvas->frame);
    //for (table = app->table; table != NULL; table = table->next)
    //if (!table->destroyed)
    //  gtk_widget_destroy(table->frame);
    gtk_main_quit();
  }

  return app->must_quit;
}


/* Defined in stage.h/stagec.: */
void stg_quit_request_code(int code);

void stg_rtk_fatal_error_dialog(const char* primary_text, const char* secondary_text, GtkWidget* parent, int exit_code, gboolean continue_button)
{
  RTK_F
    GtkDialog* dialog = (GtkDialog*) gtk_message_dialog_new (
            GTK_WINDOW(parent), GTK_DIALOG_DESTROY_WITH_PARENT|GTK_DIALOG_MODAL, 
            GTK_MESSAGE_ERROR, GTK_BUTTONS_NONE, 
            "%s", primary_text);
#if GTK_CHECK_VERSION(2,6,0)
    gtk_message_dialog_format_secondary_text(
            GTK_MESSAGE_DIALOG(dialog),
            "%s", secondary_text);
#else
    GtkWidget* label = gtk_label_new(secondary_text);
    gtk_widget_show(label);
    gtk_box_pack_end(dialog->vbox, label, FALSE, FALSE, 0);
#endif

    if(continue_button)
      gtk_dialog_add_buttons(dialog, "Continue", 1, "Exit", 2, NULL);
    else
      gtk_dialog_add_buttons(dialog, "Exit", 2, NULL);
    gtk_dialog_set_default_response(dialog, 2);
    if(gtk_dialog_run(dialog) == 2)
        stg_quit_request_code(exit_code + 1);
    gtk_widget_destroy(GTK_WIDGET(dialog));
    return;
}

