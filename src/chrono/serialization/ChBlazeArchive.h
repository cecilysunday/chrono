#pragma once

#include <vector>

#include "chrono_multicore/math/real.h"

#include <blaze/math/CompressedMatrix.h>
#include <blaze/math/DynamicVector.h>

using blaze::CompressedMatrix;
using blaze::DynamicVector;

using namespace chrono;

class ChBlazeArchive {
public:
	static void ArchiveOUTBlazeCompressedMatrix(ChArchiveOut& marchive, const CompressedMatrix<real>& src) {
    marchive.VersionWrite<size_t>();
    marchive << CHNVP(src.rows());

    marchive.VersionWrite<size_t>();
    marchive << CHNVP(src.columns());

    marchive.VersionWrite<std::vector<double>>();
    std::vector<double> v(src.rows() * src.columns());
    int c = 0;
    for (int i = 0; i < src.rows(); ++i) {
        for (int j = 0; j < src.columns(); ++j) {
          v[c] = src(i, j);
          ++c;
        }
    }
    marchive << CHNVP(v);
	}
	static void ArchiveINBlazeCompressedMatrix(ChArchiveIn& marchive, CompressedMatrix<real>& src) {
	    marchive.VersionRead<size_t>();
	    size_t rows = 0;
	    marchive >> CHNVP(rows);

	    marchive.VersionRead<size_t>();
	    size_t columns = 0;
	    marchive >> CHNVP(columns);

	    marchive.VersionRead<std::vector<double>>();
	    std::vector<double> v;
	    marchive >> CHNVP(v);
	    src.clear();
	    src.resize(rows, columns);
	    int c = 0;
	    for (int i = 0; i < rows; ++i) {
	        // std::cout << "\n";
	        for (int j = 0; j < columns; ++j) {
	            src(i, j) = v[c];
	            // std::cout << v[c];
	            if (j < columns-1) {
	              // std::cout << ", ";
	            }
	            ++c;
	        }
	    }
	}

	static void ArchiveOUTBlazeDynamicVector(ChArchiveOut& marchive, const DynamicVector<real>& src) {
	    marchive.VersionWrite<size_t>();
	    marchive << CHNVP(src.size());

	    marchive.VersionWrite<std::vector<real>>();
	    std::vector<real> v(src.size());
	    for (int i = 0; i < src.size(); ++i) {
	      v[i] = src[i];
	    }
	    marchive << CHNVP(v);
	}
	static void ArchiveINBlazeDynamicVector(ChArchiveIn& marchive, DynamicVector<real>& src) {
	    marchive.VersionRead<size_t>();
	    size_t size = 0;
	    marchive >> CHNVP(size);

	    marchive.VersionRead<std::vector<real>>();
	    std::vector<real> v;
	    marchive >> CHNVP(v);
	    for (int i = 0; i < size; ++i) {
	        src[i] = v[i];
	    }
	}
};