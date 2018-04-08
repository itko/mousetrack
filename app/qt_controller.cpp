/// \file
/// Maintainer: Luzian Hug
///
///

#include "qt_controller.h"

#include <QApplication>
#include <QPushButton>

namespace MouseTrack {

int QtController::main(int argc, char *argv[], op::variables_map& cli_options){

    // set up application
    QApplication app (argc, argv);

    // show a button in a window
    QPushButton button ("Hello world !");
    button.setToolTip("A tooltip");
    button.show();
    // start pipeline
    pipeline().start();

    // start run loop
    int returnCode = app.exec();

    pipeline().join();

    return returnCode;
}


} // MouseTrack
