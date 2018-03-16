
#include "gui.h"

//#define DEBUG

#include "stage_internal.h"
#include <assert.h>

/**
@ingroup model
@defgroup model_messages Messages service
This model allows applications to pass messages to Stage for display to the user.
This is useful when the design of the application seperates stage models from 
the world object, and also when the application needs to emit messages from different
threads (GTK is only accessible from the main thread, sometimes).

Or, this model might correspond to a display screen, internal log file, 
or even speech synthesis, on a real robot.

@note "Critical" level messages will create a popup dialog box, and provide the user with the option of exiting the program. 

<h2>Worldfile properties</h2>

There are none.

@verbatim
messages
(
)
@endverbatim

*/

/** 
@ingroup stg_model_mussages
@ingroup stg_model_props
@defgroup stg_model_messages_props Messages Properties

- "messages_data" stg_message_t;
  - Replace the fields of this struct, and include a new timestamp to cause the new message to be emitted.
- "messages_lasttime" time_t
  - Used internally to keep track of previous message's timestamp for comparison with a new one.
*/

#define MESSAGE_PROPERTY "messages_new"

stg_message_t* stg_messages_send(stg_model_t* mod, const char* cat, stg_message_level_t lev, const char* fmt, ...)
{
  va_list args;
  assert(fmt);


  // if the messages model is valid, use it, otherwise print to console:
  if(mod)
  {
    stg_message_t *message;
    gui_lock();

    /** @todo if property already contains text, append after \n, instead of
     * clobbering it */
    message = stg_model_get_property_fixed(mod, MESSAGE_PROPERTY, sizeof(stg_message_t));
    assert(message);
    if(cat)
    {
      strncpy(message->category, cat, 32);
    }
    else
    {
      message->category[0] = '\0'; // if we set it to null we loose the allocated buffer
    }
    va_start(args, fmt);
    vsnprintf(message->message, 1024, fmt, args);
    va_end(args);
    message->level = lev;
    message->timestamp = time(NULL);
    message->displayed = 0;
    stg_model_property_changed(mod, MESSAGE_PROPERTY);
    
    // bad hack since the model can only hold one message at a time (it ought
    // to have a small buffer or queue of messages really.)
    // usleep(10000); // 10ms

    gui_unlock();
    return message;
  }
  else
  {
    // stick cat onto the front of fmt:
    int len = strlen(fmt) + strlen(cat) + 3; // 2 for ": " + \0
    char *new_fmt = (char*) malloc(len);
    if(!new_fmt) return NULL;
    memset(new_fmt, 0, len);
    strcat(new_fmt, cat);
    strcat(new_fmt, ": ");
    strcat(new_fmt, fmt);
    va_start(args, fmt);
    switch(lev)
    {
      case STG_MSG_CRITICAL:
        stg_print_error_v(new_fmt, args);
        break;
      case STG_MSG_WARNING:
        stg_print_warning_v(new_fmt, args);
        break;
      default:
        stg_print_msg_v(new_fmt, args);
    }
    va_end(args);
    free(new_fmt);
  }

  return NULL;
}

stg_message_t* stg_messages_send_s(stg_model_t* mod, const char* cat, stg_message_level_t lev, const char* msg) 
{
  if(mod)
  {
    stg_message_t* message = stg_model_get_property_fixed(mod, MESSAGE_PROPERTY, sizeof(stg_message_t));
    strncpy(message->category, cat, 32);
    strncpy(message->message, msg, 1024);
    message->level = lev;
    message->timestamp = time(NULL);
    message->displayed = 0;
    stg_model_property_changed(mod, MESSAGE_PROPERTY);
    return message;
  }
  else
  {
    // stick cat onto the front of msg:
    int len = strlen(msg) + strlen(cat) + 3; // 2 for ": " + \0
    char *new_msg = (char*) malloc(len);
    memset(new_msg, 0, len);
    strcat(new_msg, cat);
    strcat(new_msg, ": ");
    switch(lev)
    {
      case STG_MSG_CRITICAL:
        stg_print_error("%s", msg);
        break;
      case STG_MSG_WARNING:
        stg_print_warning("%s", msg);
        break;
      default:
        stg_print_msg("%s", msg);
    }
  }
  return NULL;
}



int messages_callback( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{   
  stg_message_t* newmsg = (stg_message_t*) data;
  assert(len == sizeof(stg_message_t));
  if(newmsg->displayed)
    return 0;
  stg_world_display_message_s(mod->world, newmsg->timestamp, newmsg->category, newmsg->level, newmsg->message);
  newmsg->displayed = 1;
  return 0; //ok
}

/** @todo seperate text by \n into seperate messages, and clear property when done */
int messages_update(stg_model_t* mod)
{
  // Just get the property, and if it's not displayed yet, call what ought to be
  // the property callback.
  stg_message_t* msg = stg_model_get_property_fixed(mod, MESSAGE_PROPERTY, sizeof(stg_message_t));
  assert(msg);
  if(! msg->displayed)
    messages_callback(mod, MESSAGE_PROPERTY, msg, sizeof(stg_message_t), NULL);
  return 0;
}

int messages_init( stg_model_t* mod )
{
  // No normal stage events 
  mod->f_startup = NULL;
  mod->f_shutdown = NULL;
  mod->f_update =  NULL;
  mod->f_load = NULL;

  // set properties so others can just go in and poke the structs
  {
    stg_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.level = STG_MSG_INFORMATION;
    stg_model_set_property(mod, MESSAGE_PROPERTY, &msg, sizeof(msg));
  }

  // set callback for when property changes. 
  // Unfortunately, stg_model_property_changed calls callbacks synchronously, 
  // so we can't use this and instead have to do our output to GTK in the update
  // function; so this is disabled for now.
  // stg_model_add_property_callback(mod, MESSAGE_PROPERTY, messages_callback, 0);
  mod->f_update = messages_update;

  // All models have a default shape which we want to get rid of
  {
    stg_geom_t geom;
    stg_model_set_property( mod, "polygons", NULL, 0 );
    memset( &geom, 0, sizeof(geom));
    stg_model_set_property( mod, "geom", &geom, sizeof(geom) );
  }

  return 0;
}



