/*************************************************************************
Title:    Math routines for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-math.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2019 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    The cos32s, cos32, and sin32 functions are derived from those 
    provided by The Ganssle Group: http://www.ganssle.com/approx.htm
*************************************************************************/

#include <stdlib.h>

#include "cst-math.h"

// *********************************************************
// ***
// ***   Routines to compute sine and cosine to 3.2 digits
// ***  of accuracy. 
// ***
// *********************************************************
//
//		cos_32s computes cosine (x)
//
//  Accurate to about 3.2 decimal digits over the range [0, pi/2].
//  The input argument is in radians.
//
//  Algorithm:
//		cos(x)= c1 + c2*x**2 + c3*x**4
//   which is the same as:
//		cos(x)= c1 + x**2(c2 + c3*x**2)
//
float cos_32s(float x)
{
const float c1= 0.99940307;
const float c2=-0.49558072;
const float c3= 0.03679168;

float x2;							// The input argument squared

x2=x * x;
return (c1 + x2*(c2 + c3 * x2));
}

//
//  This is the main cosine approximation "driver"
// It reduces the input argument's range to [0, pi/2],
// and then calls the approximator. 
// See the notes for an explanation of the range reduction.
//
float cos_32(float x){
	int quad;						// what quadrant are we in?

	x=fmod(x, TWOPI);				// Get rid of values > 2* pi
	if(x<0)x=-x;					// cos(-x) = cos(x)
	quad=(int)(x * TWO_OVER_PI);			// Get quadrant # (0 to 3) we're in
	switch (quad){
	case 0: return  cos_32s(x);
	case 1: return -cos_32s(PI-x);
	case 2: return -cos_32s(x-PI);
	case 3: return  cos_32s(TWOPI-x);
	default: return 0;
	}
}
//
//   The sine is just cosine shifted a half-pi, so
// we'll adjust the argument and call the cosine approximation.
//
float sin_32(float x){
	return cos_32(HALFPI-x);
}

