#ifndef IMAGE_MAP_INFO_H
#define IMAGE_MAP_INFO_H

#include <QMainWindow>

#include "ui_image_map_info.h"

namespace Ui {
    class ImageMapInfo;
}

namespace map_visualization{

class ImageMapInfo : public QMainWindow
{
    Q_OBJECT
public:
    explicit ImageMapInfo(QWidget *parent = 0);
    ~ImageMapInfo();
private:
    Ui::ImageMapInfo *ui;
};

}
#endif // IMAGE_MAP_INFO_H
