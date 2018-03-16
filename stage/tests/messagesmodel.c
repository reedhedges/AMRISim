

#include <stdio.h>
#include "stage.h"
#include <assert.h>

int main( int argc, char* argv[] )
{ 
  stg_world_t* world;
  stg_model_t* messages;
  stg_message_t* msg;
  stg_msec_t start;

  puts( "Stage test program: print messages to the world window, and wait until they are displayed.");
  if(argc > 1 && strcmp(argv[1], "--help") == 0)
    return 0;
  stg_init( argc, argv );
  world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);
  stg_world_start(world);

  messages = stg_world_new_model(world, "messages", NULL, NULL);
  assert(messages);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Informational Message One of Three");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Informational Message Two of Three");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Informational Message Three of Three");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Series Message One of Four");
  stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Series Message Two of Four");
  stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Series Message Three of Four");
  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Series Message Four of Four");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_WARNING, "Warning Message");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Switching log to HTML format");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  stg_set_print_format(STG_PRINT_HTML);
  stg_print_msg("Here is an HTML log message printed with stg_print_msg (sholud not show in GUI)");

  msg = stg_messages_send(messages, "messagestest", STG_MSG_WARNING, "Another Warning Message");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "Another Informational Message, then switching back to plain text");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  stg_set_print_format(STG_PRINT_COLOR_TEXT);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_CRITICAL, "Error Message");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  stg_set_print_format(STG_PRINT_PLAIN_TEXT);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_CRITICAL, "Error Message without color");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  msg = stg_messages_send(messages, "messagestest", STG_MSG_INFORMATION, "destroying the world and ending in 5 sec...");
  if(msg) while(!msg->displayed) stg_world_update(world, TRUE, FALSE);

  start = stg_world_get_time(world);
  while(stg_world_get_time(world) - start < 5000)
    stg_world_update(world, TRUE, FALSE);


  stg_world_destroy(world);
  puts("messagestest: end of program.");
  return 0;
}
