
#include "guiWrapper.h"

/*
 * Constructor. Initialized a new QApplication and MainWindow.
 */
guiWrapper::guiWrapper(int argc, char** argv){
    a = new QApplication(argc, argv);
    w = new MainWindow;
    pixmap = nullptr;
}

guiWrapper::~guiWrapper(){
    delete a;
    delete w;
}

/*
 * Visualize the MainWindow and run the QApplication.
 */
void guiWrapper::run(){
    w->show();
    a->exec();
}

MainWindow* guiWrapper::getMainwindow(){
    return w;
}

/*
 * Convert cv::Mat to Pixmap and set the MainWindows Pixmap to it.
 */
void guiWrapper::visualize(cv::Mat image){
    QPixmap map = QPixmap::fromImage(QImage((uchar*) image.data, image.cols, image.rows, image.step, QImage::Format_RGB888));
    w->setPixmap(map);
}

