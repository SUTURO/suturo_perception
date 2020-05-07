//
// Created by janf on 02.05.20.
//

#include "guiWrapper.h"


guiWrapper::guiWrapper(int argc, char** argv){
    a = new QApplication(argc, argv);
    w = new MainWindow;
    pixmap = nullptr;
}

guiWrapper::~guiWrapper(){
    delete a;
    delete w;
}

void guiWrapper::run(){
    w->show();
    a->exec();
}

MainWindow* guiWrapper::getMainwindow(){
    return w;
}

void guiWrapper::visualize(cv::Mat image){
    QPixmap map = QPixmap::fromImage(QImage((uchar*) image.data, image.cols, image.rows, image.step, QImage::Format_RGB888));
    w->setPixmap(map);
}

