

/* Test comparing accuracy of gettimeofday() and stg_timenow() vs. usleep(). 
   Used to find values to use in stg_world_update() for various operating systems.
   Reed Hedges <reed@mobilerobots.com>
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "stage.h"
#include "replace.h"

int main(int argc, char **argv)
{
	int sum = 0;
	int trunc10sum = 0;
	int trunc100sum = 0;
	int round10sum = 0;
	int round100sum = 0;
	int n = 0;
        int max = 0;
	struct timeval tv;
	int i;
	for(i = 0; i < 4; ++i)
	{
		int t;
		for(t = 5; t < 200; t += 13)
		{
			int j;
			for(j = 0;  j < 3; ++j)
			{
				stg_msec_t start, end, diff, error;
				gettimeofday(&tv, NULL);
				start = (stg_msec_t)((tv.tv_sec * 1000.0) + (tv.tv_usec/1000));
				usleep(t*1000);
				gettimeofday(&tv, NULL);
				end = (stg_msec_t)((tv.tv_sec * 1000.0) + (tv.tv_usec/1000.0));
				printf("Sleeping for %d msec (usleep) took %lu msec clock time according to gettimeofday.\n", t, end-start);	
				fflush(stdout);

				start = stg_timenow();
				usleep(t*1000);
				end = stg_timenow();
				printf("Sleeping for %d msec (usleep) took %lu msec clock time according to stg_timenow().\n", t, end-start);	
				fflush(stdout);

				diff = end-start;
 				error = diff-t;
                                if(error > max) max = error;
				sum += error;
				trunc10sum += (trunc((double)(diff)/10)*10) - t;
				trunc100sum += (trunc((double)(diff)/100)*100) - t;
				round10sum += (rint((double)(diff)/10)*10) - t;
				round100sum += (rint((double)(diff)/100)*100) - t;
				++n;
			}
		}
	}
        printf("Maximum error is %d\n", max);
	printf("Average error is %0.3f.\n", ((double)sum)/n);
	printf("Average error of truncated-to-100 is %0.3f.\n", ((double)trunc10sum)/n);
	printf("Average error of rounded-to-100 is %0.3f.\n", ((double)round100sum)/n);
	printf("Average error of truncated-to-10 is %0.3f.\n", ((double)trunc10sum)/n);
	printf("Average error of rounded-to-10 is %0.3f.\n", ((double)round10sum)/n);
}
