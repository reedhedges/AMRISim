
#include <stdlib.h>
char* rindex(const char* s, int c)
{
  char* ret = NULL;
  char* str = s;
  while(*str != NULL)
  {
    if(*str == c) ret = str;
    str++;
  }
  return ret;
}

