
/* This is a small test of the strategy used in the main loop of MobileSim to
 * update stage and send client output (send SIPs and laser packets) at the right times. 
 *
 * When run you should see stage updates and client output happening at the
 * right intervals (set intervals below)
 *
 * The actual time it takes to send stage updates, do client output, do client
 * input, etc. are simulated by sleeping for arbitrary amounts of time (set
 * durations below).
 *
 * Verbose information in printed on stderr, just the timing error on stdout.
 */

#include <stdio.h>
#include "ariaUtil.h"

const long stage_interval = 250; //ms
const long client_output_interval = 100;  //ms
const long stage_update_dur = 25;
const long client_output_dur = 10;
const long client_input_dur = 20;

ArTime stageUpdateDue;
ArTime clientOutputDue;
ArTime lastStageUpdate;
ArTime lastClientOutput;
ArTime lastClientInput;

void sleepms(long ms)
{
  fprintf(stderr, "sleep(%ld)...\n", ms);
  //fflush(stdout);
  if(ms > 0)
    //ArUtil::sleep(ms); // ArUtil::sleep() specifically sleeps for 10 msec
    //less than its argument, I guess this was to work around some error in some
    //versions of Linux?
    usleep(ms * 1000);
}
void stage_update()
{
  fprintf(stderr, "Stage update (%ld ms since last, should be %ld) (will take %ldms)...\n", lastStageUpdate.mSecSince(), stage_interval, stage_update_dur);
  printf("Stage update error\t%ld\n", lastStageUpdate.mSecSince() - stage_interval);
  //fflush(stdout);
  stageUpdateDue.setToNow();
  stageUpdateDue.addMSec(stage_interval);
  lastStageUpdate.setToNow();
  sleepms(stage_update_dur);
}

void client_output()
{
  fprintf(stderr, "Client output (%ld ms since last, should be %ld) (will take %ldms)...\n", lastClientOutput.mSecSince(), client_output_interval, client_output_dur);
  printf("Client output error\t%ld\n", lastClientOutput.mSecSince() - client_output_interval);
  //fflush(stdout);
  clientOutputDue.setToNow();
  clientOutputDue.addMSec(client_output_interval);
  lastClientOutput.setToNow();
  sleepms(client_output_dur);
}

void client_input()
{
  fprintf(stderr, "Client input (%ldms)...", client_input_dur);
  //fflush(stdout);
  lastClientInput.setToNow();
  sleepms(client_input_dur);
}

int main(int argc, char **argv)
{
  while(true)
  {
    //ArTime t;
    //t.addMSec(100);
    //printf("t.mSecTo()=%ld (should be almost 100)\n", t.mSecTo());

    if(stageUpdateDue.mSecTo() < clientOutputDue.mSecTo())
    {
      fprintf(stderr, "Stage update (due in %ld ms) should come BEFORE client output (due in %ld ms)\n", stageUpdateDue.mSecTo(), clientOutputDue.mSecTo());
      //fflush(stdout);
      sleepms(stageUpdateDue.mSecTo());
      stage_update();
      //fprintf(stderr, "Next stage update in %ld ms. Now doing client output (%ld ms) then client input (%ld ms)\n", stageUpdateDue.mSecTo(), client_output_dur, client_input_dur);
      //sleepms(clientOutputDue.mSecTo());
      //client_output();
    }
    else
    {
      fprintf(stderr, "Stage update (due in %ld ms) should come AFTER client output (due in %ld ms)\n", stageUpdateDue.mSecTo(), clientOutputDue.mSecTo());
      //fflush(stdout);
      sleepms(clientOutputDue.mSecTo());
      client_output();
      //fprintf(stderr, "Next client output in %ld ms. Now doing stage update (%ld ms) then client input (%ld ms)\n", clientOutputDue.mSecTo(), stage_update_dur, client_input_dur);
      //sleepms(stageUpdateDue.mSecTo());
      //stage_update();
    }
    // Client input is not time sensitive. We do it if we have at least 10ms
    // available or it's been twice the SIP frequency since we did it.
    if( ( stageUpdateDue.mSecTo() > 10 && clientOutputDue.mSecTo() > 10 ) || lastClientInput.mSecSince() > (2*client_output_interval) )
      client_input();
  }
}
