#include <QtGui/QApplication>
#include "mobots_info.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Mobots_Info w;
    w.show();
    
    return a.exec();
}
