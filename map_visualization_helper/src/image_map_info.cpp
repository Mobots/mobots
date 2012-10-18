#include "image_map_info.h"

namespace map_visualization_helper{

ImageMapInfo::ImageMapInfo(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::ImageMapInfo)
{
    ROS_INFO("[ImageMapInfo]");
    ui->setupUi(this);
    ImageMapModel imageMapModel(0);
    ui->mobotInfoTable->setModel(&imageMapModel);
    ROS_INFO("[ImageMapInfo]");
}

ImageMapInfo::~ImageMapInfo(){
    delete ui;
}

}
