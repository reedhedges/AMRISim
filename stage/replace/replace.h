/*  $Id$
 *
 * replacement function prototypes
 */

#ifndef _REPLACE_H
#define _REPLACE_H


#if HAVE_CONFIG_H
  #include <config.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if !HAVE_SCANDIR
#include <sys/types.h>
#include <dirent.h>
int scandir(const char *dir, struct dirent ***namelist,
            int (*select)(const struct dirent *),
            int (*compar)(const struct dirent **, const struct dirent **));
#endif //!HAVE_SCANDIR

#if !HAVE_DIRNAME
  char * dirname (char *path);
#else
  #include <libgen.h> // for dirname(3)
#endif // !HAVE_DIRNAME

#if !HAVE_BASENAME
  char * basename (const char* filename);
#else
  #include <libgen.h> // for basename(3)
#endif // !HAVE_BASENAME

#if !HAVE_USLEEP
 int usleep(unsigned long usec);
#endif

#if !HAVE_GETTIMEOFDAY
 #include <sys/time.h>
 struct timezone {
  int tz_minuteswest;
  int tz_tsttime;
 };
 int gettimeofday(struct timeval* tv, struct timezone* tz);
#endif

#if !HAVE_RINDEX
 char* rindex(const char* s, int c);
#endif



#ifdef __cplusplus
}
#endif

 

#endif

