#include <iostream>
#include <generic/vec_addition.h>
#include <memory>

#ifdef ENABLE_GUI
#include "qt_controller.h"
#endif

int main (int argc, char *argv[]) {

    std::cout << "hello world! It works!" << std::endl;
	std::vector<double> a, b;
	a.push_back(.3);
	b.push_back(.2);
    auto result = MouseTrack::add(a,b);
	for(auto r : result){
		std::cout << r << std::endl;
	}
    const bool gui_mode = true;
    std::unique_ptr<MouseTrack::Controller> controller;
#ifdef ENABLE_GUI
    if(gui_mode){
        controller = std::unique_ptr<MouseTrack::QtController>(new MouseTrack::QtController());
        return controller->main(argc, argv);
    }
#else
    // we might need this later on for integration tests
    return 0;
#endif
} 
