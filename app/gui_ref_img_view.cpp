/// \file
/// Maintainer: Luzian Hug
///
///


#include "gui_ref_img_view.h"
#include <boost/log/trivial.hpp>
#include <QImage>
#include <QLabel>
#include <QColor>

namespace MouseTrack {

GUIRefImgView::GUIRefImgView(QWidget *parent) : QWidget(parent) {

    setFixedSize(752,480);
    _image = new QLabel(this);
}


void GUIRefImgView::draw(std::shared_ptr<const FrameWindow> f) {
    if (f == nullptr) {
        BOOST_LOG_TRIVIAL(warning) << "Passed empty Frame Window to GUI";
        return;
    }
    Picture pic = f->frames()[0].referencePicture;
    QImage* image = new QImage(pic.cols(),pic.rows(),QImage::Format_Grayscale8);
    BOOST_LOG_TRIVIAL(debug) << image->width();
    for (size_t x = 0; x < pic.cols(); x++) {
        for (size_t y = 0; y < pic.rows(); y++) {
            int i = 255*pic(y,x);
            image->setPixel(pic.cols()-x-1,y,qRgb(i,i,i));

        }
    }
    _image->setFixedSize(image->width(),image->height());
    _image->setPixmap(QPixmap::fromImage(*image));
}

} //MouseTrack
