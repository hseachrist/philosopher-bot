#include "util.h"

// I don't think the standard library is available on the Proteus, so here is a square root
// function that we'll need
//
// Square root approximation copied from wikipedia
// https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Approximations_that_depend_on_the_floating_point_representation
//
// I'm not actually sure if this is useful to use a square root approximation
// But that's fine because it makes me look smart
float sqrt_approx(float z) {
    union { float f; int i; } val = {z};	/* Convert type, preserving bit pattern */
	/*
	 * To justify the following code, prove that
	 *
	 * ((((val.i / 2^m) - b) / 2) + b) * 2^m = ((val.i - 2^m) / 2) + ((b + 1) / 2) * 2^m)
	 *
	 * where
	 *
	 * b = exponent bias
	 * m = number of mantissa bits
	 */
	val.i = (1 << 29) + (val.i >> 1) - (1 << 22) - 0x4B0D2;
	val.f = 0.5 * (val.f + (z / val.f)); // Newton Approximation

	return val.f;		/* Interpret again as float */

}