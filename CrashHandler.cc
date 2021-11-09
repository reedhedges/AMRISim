
/*  
 *  AMRISim is based on MobileSim (Copyright 2005 ActivMedia Robotics, 2006-2010 
 *  MobileRobots Inc, 2011-2015 Adept Technology, 2016-2017 Omron Adept Technologies)
 *  and Stage version 2 (Copyright Richard Vaughan, Brian Gerkey, Andrew Howard, and 
 *  others), published under the terms of the GNU General Public License version 2.
 *
 *  Copyright 2018 Reed Hedges and others
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

#include "AMRISim.hh"
#include "stage.h"
#include <signal.h>
#include <unistd.h>

#include "RobotFactory.hh"
#include "EmulatePioneer.hh"
#include "RobotInterface.hh"

void mobilesim_crash_handler(int signum)
{
    // disable this signal handler to avoid "recursion" if something in this
    // function crashes
    signal(SIGABRT, SIG_DFL);
    signal(SIGSEGV, SIG_DFL);
    signal(SIGILL, SIG_DFL);
    signal(SIGFPE, SIG_DFL);
#ifndef WIN32
    signal(SIGBUS, SIG_DFL);
#endif

    const char *signame = "unknown";
    switch(signum) {
        case SIGABRT: signame = "SIGABRT"; break;
        case SIGSEGV: signame = "SIGSEGV"; break;
        case SIGILL: signame = "SIGILL"; break;
        case SIGFPE: signame = "SIGFPE"; break;
#ifndef WIN32
        case SIGBUS: signame = "SIGBUS"; break;
#endif
    }
    stg_print_error("AMRISim: Received fatal signal %d (%s)!", signum, signame);


    const char *progname = AMRISim::options.argv[0];
    if(AMRISim::options.EnableCrashDebug || AMRISim::options.EnableCrashRestart)
    {
      struct stat statbuf;
      if(stat(progname, &statbuf) != 0) {
          progname = "/usr/local/AMRISim/AMRISim";
          if(stat(progname, &statbuf) != 0) {
              stg_print_error("AMRISim: Can't find AMRISim program (tried %s and %s), Can't save debugging information or restart program.", AMRISim::options.argv[0], progname);
              return;
          }
      }
    }

    if(AMRISim::options.EnableCrashDebug)
    {
      const char *gdbhelper = "gdbhelper";
      struct stat statbuf;
      if(stat(gdbhelper, &statbuf) != 0) {
          gdbhelper = "/usr/local/AMRISim/gdbhelper";
          if(stat(gdbhelper, &statbuf) != 0) {
              gdbhelper = NULL;
              stg_print_error("AMRISim: Couldn't find gdbhelper in current directory or as /usr/local/AMRISim/gdbhelper, can't save debugging information.");
          }
      }
      if(gdbhelper != NULL && progname != NULL) { 
          char cmd[4096];
          snprintf(cmd, 4096, "gdb -batch -x %s %s %d %s%s", gdbhelper, progname, getpid(), (AMRISim::options.log_file==NULL)?"":">>", (AMRISim::options.log_file==NULL)?"":AMRISim::options.log_file);
          stg_print_error("AMRISim: Running gdb to get debugging information: %s", cmd);
          stg_print_error("AMRISim: --- Begin gdb ---");
          if(AMRISim::options.log_file) stg_close_log_file();
          if(system(cmd) != 0)
            stg_print_error("Warning: error running gdb!");
          if(AMRISim::options.log_file) stg_open_log_file(AMRISim::options.log_file, "a");
          stg_print_error("AMRISim: --- End gdb ---");
      }
    }

    if(AMRISim::options.NonInteractive && AMRISim::options.EnableCrashRestart && progname) {

        // Restart amrisim with same arguments, plus --restarting-after-crash

#if 0
        // Need copies because objects' destructors may try to remove themselves
        // from the lists, invalidating the iterators.
        stg_print_error("AMRISim: Cleaning allocated objects...");
        //stg_print_error("AMRISim: deleting emulators...");
        std::set<EmulatePioneer*> ems = emulators;
        for(std::set<EmulatePioneer*>::iterator i = ems.begin(); i != ems.end(); ++i)
        {
          //stg_print_error("AMRISim: ...deleting emulator 0x%x", (*i));
          delete(*i);
          //stg_print_error("AMRISim: ...finished deleting emulator 0x%x", (*i));
        }
        //stg_print_error("AMRISim: done deleting emulators, deleting interfaces...");
        std::set<RobotInterface*> ifs = robotInterfaces;
        for(std::set<RobotInterface*>::iterator i = ifs.begin(); i != ifs.end(); ++i)
        {
          //stg_print_error("AMRISim: ...deleting interface 0x%x", (*i));
          delete(*i);
        }
          //stg_print_error("AMRISim: done deleting interfaces, deleting factories...");
        std::set<RobotFactory*> facs = factories;
        for(std::set<RobotFactory*>::iterator i = facs.begin(); i != facs.end(); ++i)
        {
          //stg_print_error("AMRISim: ...deleting factory 0x%x", (*i));
          delete(*i);
        }
        stg_print_error("AMRISim: Done cleaning up.");
#endif

        // XXX TODO maybe we should store program arguments as object containing
        // std::array<std::string> or std::vector<std::string> plus conversion
        // functions to char*[] (either with pointers into std::string
        // c_str/data therefore tied to the lifetime of that or as copies).

        // Allocate argv: program name + options.argc + --restarting-after-crash:
        // It will not be freed, we go right into exec().
        char **argv = (char**) malloc(1 + (size_t)AMRISim::options.argc + 1 * sizeof(char*));
        argv[0] = (char*) progname;

        // Also build up a single string with arguments to print out.
        size_t argstr_len = strlen(progname) + 1; // +1 for NULL
        char *argstr = (char*) malloc(argstr_len);   // concat all argvs for logging

        strncpy(argstr, progname, argstr_len);
        argv[1] = (char*) malloc(25);
        strncpy(argv[1], "--restarting-after-crash", 24);
        std::string("--restarting-after-crash").c_str();
        argstr_len += strlen(" --restarting-after-crash");
        char *a = (char*) realloc(argstr, argstr_len);
        if(a == NULL)
        {
          perror("AMRISIM: Error in realloc()");
          free(argstr);
          free(argv[1]);
          free(argv);
          abort();
        }
        argstr = a;
        strncat(argstr, " --restarting-after-crash", argstr_len);

        for(int i = 2; i <= AMRISim::options.argc; ++i)
        {
            argv[i] = AMRISim::options.argv[i-1];
            argstr_len += strlen(argv[i]) + 1; // + 1 for " "
            argstr = (char*) realloc(argstr, argstr_len);
            assert(argstr);
            strncat(argstr, " ", 1);
            strncat(argstr, argv[i], argstr_len-1);
        }
        argv[AMRISim::options.argc+1] = NULL;
        stg_print_error("AMRISim: Restarting AMRISim : %s", argstr);
        free(argstr);

        // Close and move log files out of the way
        mobilesim_rotate_log_files("-crash");

        // Exec new AMRISim
        execvp(progname, argv);
        perror("AMRISim: Error in exec()");
        free(argv[1]); // not neccesary but prevent compiler/analyzer warnings
        free(argv); // not neccesary but prevent compiler/analyzer warnings
        abort(); // should never reach here unless execvp had an error and returned
    }
    else
    {
        exit(ERR_CRASH);
    }
}

int mobilesim_rotate_log_files(const char *suffix)
{
    if(!AMRISim::options.log_file) return 0;

    stg_print_msg("AMRISim: Rotating log files...");

    if(AMRISim::options.log_file && !(stg_output_file == NULL || stg_output_file == stdout || stg_output_file == stderr))
    {
      stg_print_msg("AMRISim: Closing this log file...");
      fclose(stg_output_file);
      stg_output_file = NULL;
    }



    char buf1[MAX_PATH_LEN];
    char buf2[MAX_PATH_LEN];
    char buf1s[MAX_PATH_LEN];
    char buf2s[MAX_PATH_LEN];
    char *oldfile;
    char *oldfile_s;
    char *newfile;
    char *newfile_s;
    char *tmp;
    char cmd[(MAX_PATH_LEN*2)+14];   // 14 chars for "mv -f -v '' ''".
    oldfile = buf1;
    newfile = buf2;
    oldfile_s = buf1s;
    newfile_s = buf2s;
    snprintf(newfile, MAX_PATH_LEN, "%s-5", AMRISim::options.log_file);
    if(suffix)
      snprintf(newfile_s, MAX_PATH_LEN, "%s%s-5", AMRISim::options.log_file, suffix);
    char cwdbuf[MAX_PATH_LEN];
    if(getcwd(cwdbuf, MAX_PATH_LEN) == NULL)
    {
      printf("(error getting current working directory, can't rotate log files.");
      return 0;
    }

    printf("(rotating log files... cwd is %s)\n", cwdbuf);  // helpful because 'mv -v' will print out mysterious messages 
    for(int i = 4; i >= 1; --i)
    {
      snprintf(oldfile, MAX_PATH_LEN, "%s-%d", AMRISim::options.log_file, i);
      snprintf(cmd, (MAX_PATH_LEN*2)+14, "mv -f -v '%s' '%s'", oldfile, newfile);
      if(system(cmd) != 0)
        stg_print_error("Warning: error rotating log file!");

      if(suffix)
      {
        snprintf(oldfile_s, MAX_PATH_LEN, "%s%s-%d", AMRISim::options.log_file, suffix, i);
        snprintf(cmd, (MAX_PATH_LEN*2)+14, "mv -f -v '%s' '%s'", oldfile_s, newfile_s);
        if(system(cmd) != 0)
          stg_print_error("Warning: error rotating log file!");
      }

      // swap each buffer pair
      tmp = oldfile;
      oldfile = newfile;
      newfile = tmp;

      if(suffix)
      {
        tmp = oldfile_s;
        oldfile_s = newfile_s;
        newfile_s = tmp;
      }
    }
    if(suffix)
      snprintf(cmd, (MAX_PATH_LEN*2)+4, "mv -f -v '%s' '%s%s'-1", AMRISim::options.log_file, AMRISim::options.log_file, suffix);
    else
      snprintf(cmd, (MAX_PATH_LEN*2)+4, "mv -f -v '%s' '%s'-1", AMRISim::options.log_file, AMRISim::options.log_file);
    if(system(cmd) != 0)
      stg_print_error("Warning: error rotating log file!");

    FILE *newfp = ArUtil::fopen(AMRISim::options.log_file, "w");
    if(newfp)
    {
      stg_set_log_file(newfp);
      stg_print_msg("AMRISim: Start of new log file at %s after rotating log files.", AMRISim::options.log_file);
    }
    else
    {
      stg_set_log_file(NULL);
      stg_print_error("AMRISim: Error opening new log file %s after rotating log files!", AMRISim::options.log_file);
      return 0;
    }
    

    return 1;
}

FILE* mobilesim_reopen_log_file()
{
  if(stg_output_file)
    fclose(stg_output_file);
  stg_output_file = ArUtil::fopen(AMRISim::options.log_file, "w");
  return stg_output_file;
}

