#include <QtGui>
#include <QApplication>
#include "image_map_info.h"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    map_visualization::ImageMapInfo info(argc,argv);
    info.show();
    //app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}
