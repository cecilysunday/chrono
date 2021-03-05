// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of some convenience functions for math operations
// =============================================================================

#pragma once

#include "chrono_multicore/math/real.h"
#include "chrono_multicore/math/real3.h"
#include "chrono_multicore/math/real4.h"

namespace chrono {

/// @addtogroup multicore_math
/// @{

/// Computes the nearest power of two to the given value and returns it
static inline uint nearest_pow(const uint& num) {
    uint n = num > 0 ? num - 1 : 0;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
}

/// Given a frame with origin 'p' and orientation 'q', transform the
/// position vector 'rl' expressed in the local frame into the parent
/// frame:  rp = p + A * rl
static inline real3 TransformLocalToParent(const real3& p, const quaternion& q, const real3& rl) {
    return p + Rotate(rl, q);
}

/// Given a frame with origin 'p' and orientation 'q', transform the
/// position vector 'rp' expressed in the parent frame into the local
/// frame:  rl = A^T * (rp - p)
static inline real3 TransformParentToLocal(const real3& p, const quaternion& q, const real3& rp) {
    return RotateT(rp - p, q);
}

class real3_int {
  public:
    real3_int() {}
    real3_int(real3 a, int b) : v(a), i(b) {}

    real3 v;
    int i;

    /// Method to allow serialization of transient array to archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient array from archives.
    void ArchiveIN(ChArchiveIn& marchive);
};

inline void real3_int::ArchiveOUT(ChArchiveOut& marchive) {
    // suggested: use versioning
    marchive.VersionWrite<real3_int>();  // must use specialized template (any)
    // stream out all member array
    marchive << CHNVP(v);
    marchive << CHNVP(i);
}

inline void real3_int::ArchiveIN(ChArchiveIn& marchive) {
    // suggested: use versioning
    int version = marchive.VersionRead<real3_int>();  // must use specialized template (any)
    // stream in all member array
    marchive >> CHNVP(v);
    marchive >> CHNVP(i);
}

/// @} multicore_math

} // end namespace chrono
