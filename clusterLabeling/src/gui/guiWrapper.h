//
// Created by janf on 02.05.20.
//

#ifndef CLUSTERLABELING_GUIWRAPPER_H
#define CLUSTERLABELING_GUIWRAPPER_H

#include <QApplication>
#include "mainwindow.h"

#include <opencv2/opencv.hpp>


class guiWrapper{
private:
    QApplication* a;
    MainWindow* w;
    QPixmap* pixmap;

public:
    guiWrapper(int argc, char** argv);

    ~guiWrapper();

    MainWindow* getMainwindow();

    void run();

    void visualize(cv::Mat image);

};


#endif //CLUSTERLABELING_GUIWRAPPER_H
