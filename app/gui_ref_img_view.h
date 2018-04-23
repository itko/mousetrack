/// \file
/// Maintainer: Luzian Hug
///
///


#pragma once
#include <QWidget>
#include <QLabel>

#include <memory>

#include "generic/frame_window.h"

namespace MouseTrack {

class GUIRefImgView : public QWidget {
    Q_OBJECT
public:
    explicit GUIRefImgView(QWidget *parent);
    void draw(std::shared_ptr<const FrameWindow> f);
private:
    QLabel* _image;
};

} //MouseTrack
