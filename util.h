
/*  
 *  Copyright (C) 2011-2015 Adept Technology
 *  Copyright (C) 2016-2017 Omron Adept Technologies
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

#ifndef MOBILESIM_UTIL_H
#define MOBILESIM_UTIL_H

/* Macro expanding to a boolean expression matching variations of a command line argument. short_arg
 * and long_arg must be literal string constants (ignored if "")
 */
#define command_argument_match(arg_given, short_arg, long_arg) \
  ( (strlen(short_arg) > 0 && strcmp(arg_given, "-" short_arg) == 0) || \
    (strlen(long_arg) > 0 && strcmp(arg_given, "--" long_arg) == 0) || \
    (strlen(long_arg) > 0 && strcmp(arg_given, "-" long_arg) == 0)  )

#endif
