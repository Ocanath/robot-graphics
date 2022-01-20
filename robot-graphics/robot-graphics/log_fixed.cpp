/*
Credit for this code comes from:
https://github.com/dmoulding/log2fix

I have made a couple of changes, to allow log base (e) of numbers less than 1
and to keep types consistent
*/

#include "log_fixed.h"

#define INV_LOG2_E_Q1DOT31  UINT64_C(0x58b90bfc) // Inverse log base 2 of e

/**/
int32_t log2fix(uint32_t x, size_t precision)
{
    int32_t b = 1U << (precision - 1);
    int32_t y = 0;

    if (precision < 1 || precision > 31) {
        //errno = EINVAL;   //ERROR
        return INT32_MAX; // indicates an error
    }

    if (x == 0) {
        return INT32_MIN; // represents negative infinity
    }

    while (x < 1U << precision) {
        x <<= 1;
        y -= 1U << precision;
    }

    while (x >= 2U << precision) {
        x >>= 1;
        y += 1U << precision;
    }

    uint64_t z = x;
    for (size_t i = 0; i < precision; i++) 
    {
        z = z * z >> precision;
        if (z >= (2ULL << (uint64_t)precision)) 
        {
            z >>= 1;
            y += b;
        }
        b >>= 1;
    }

    return y;
}

int32_t logfix(uint32_t x, size_t precision)
{
    int64_t t;

    t = log2fix(x, precision) * INV_LOG2_E_Q1DOT31;

    return (int32_t)(t / (1LL << 31LL));
}