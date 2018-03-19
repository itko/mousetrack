#include "vec_addition.h"

namespace MouseTrack {

/// Add two STL vectors together, if one vector is larger, its elements are copied to the result.
std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b) {
	// Use a for-loop to add the common part
	// and then copy the rest of the larger vector
    int min = a.size();
    int max = b.size();
	if(max < min) {
		std::swap(min, max);
	}
	std::vector<double> sum(max);
    for(int i = 0; i < min; i += 1){
		sum[i] = a[i] + b[i];
	}
	if(a.size() < b.size()){
		sum.insert(sum.end(), &b[min], &b[max]);
	}
	if(b.size() < a.size()){
		sum.insert(sum.end(), &a[min], &a[max]);
	}
	return sum;
}

}
