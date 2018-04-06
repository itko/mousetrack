/// \file
/// Maintainer: Luzian Hug
///
///

#include "qt_controller.h"

#include <QApplication>
#include <QPushButton>
#include "camera_view_widget.h"
#include "gui_observer.h"

namespace MouseTrack {

int QtController::main(int argc, char *argv[]){

    // set up application
    QApplication app (argc, argv);

    // show a button in a window
    CameraViewWidget* widget = new CameraViewWidget();
    widget->setFixedHeight(480);
    widget->setFixedWidth(640);
    widget->show();

    GUIObserver* observer = new GUIObserver(widget);
    pipeline().addObserver(observer);
    // start pipeline
    pipeline().start();

    // start run loop
    int returnCode = app.exec();

    pipeline().join();

    return returnCode;
}


} // MouseTrack
