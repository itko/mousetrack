#include<iostream>
#include<generic/vec_addition.h>

int main (int argc, char *argv[]) {
    std::cout << "hello world! It works!" << std::endl;
	std::vector<double> a, b;
	a.push_back(.3);
	b.push_back(.2);
    auto result = MouseTrack::add(a,b);
	for(auto r : result){
		std::cout << r << std::endl;
	}
	return 0;
} 
