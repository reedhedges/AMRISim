
#include <stdio.h>
#include "stage.h"

FILE *fp;

void fullcb(FILE *fp, size_t sz, size_t max)
{
    FILE *newfp;
    printf("file-full callback called for file pointer 0x%x, current size %u, max %u\n", (unsigned int)fp, sz, max);
    puts("Writing message from full callback (before opening new file)...");
    stg_print_msg("Message inside callback");
    fflush(stdout);
    puts("opening new log file test2.log and switching to that.");
    newfp = fopen("test2.log", "w");
    fclose(fp);
    fp = newfp;
    stg_set_log_file(fp);
    printf("Opened new file 0x%x. Writing log message \"New file\"...\n", (unsigned int)fp);
    stg_print_msg("New file");
    fflush(stdout);
}

int main( int argc, char* argv[] )
{ 
    stg_print_msg("Message");
    stg_print_warning("Warning");
    stg_print_error("Error");
    puts("opening new log file test.log and usingthat for log from now on...");
    stg_set_log_file(fp = fopen("test.log", "w"));
    printf("fp is 0x%x\n", (unsigned int)fp);
    puts("Writing \"Message\"");
    stg_print_msg("Message");
    puts("Setting max file size to 30 with a file-full callback...");
    stg_set_log_file_max_size(30, &fullcb);
    puts("Writing some messages (more than 30 bytes)...");
    stg_print_msg("1234567");
    stg_print_msg("8901234");
    stg_print_msg("5678901");
    stg_print_msg("2345678");
    puts("closing log file, back to console output...");
    stg_set_log_file(NULL);
    fclose(fp);
    stg_print_msg("Message");
    stg_print_msg("Done with test.");
}
