#ifndef IMAGE_MAP_INFO_H
#define IMAGE_MAP_INFO_H

#include <QMainWindow>

#include <ros/ros.h>

#include "ui_image_map_info.h"
#include "image_map_model.h"

namespace Ui {
    class ImageMapInfo;
}

namespace map_visualization_helper{

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
