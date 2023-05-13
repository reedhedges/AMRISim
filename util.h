

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

#ifndef AMRISIM_UTIL_H
#define AMRISIM_UTIL_H

/* Macro expanding to a boolean expression matching variations of a command line argument. short_arg
 * and long_arg must be literal string constants (ignored if "")
 */
#define command_argument_match(arg_given, short_arg, long_arg) \
  ( (strlen(short_arg) > 0 && strcmp(arg_given, "-" short_arg) == 0) || \
    (strlen(long_arg) > 0 && strcmp(arg_given, "--" long_arg) == 0) || \
    (strlen(long_arg) > 0 && strcmp(arg_given, "-" long_arg) == 0)  )

#if (__cplusplus >= 202000)
  //#warning Building as C++20
  #if __has_cpp_attribute(likely)
    #define LIKELY [[likely]]
  #else
    #warning Building as C++20 or later but do not have [[likely]] according to __has_cpp_attribute(likely)
    #define LIKELY
  #endif
  #if __has_cpp_attribute(unlikely)
    #define UNLIKELY [[unlikely]]
  #else
    #warning Building as C++20 or later but do not have [[unlikely]] according to __has_cpp_attribute(unlikely)
    #define UNLIKELY
  #endif
#else
  //#warning not C++20
  #define LIKELY
  #define UNLIKELY
#endif


#endif
