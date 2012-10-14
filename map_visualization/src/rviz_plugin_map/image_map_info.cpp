#include "image_map_info.h"

namespace map_visualization{

ImageMapInfo::ImageMapInfo(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::ImageMapInfo)
{
    ui->setupUi(this);
}

ImageMapInfo::~ImageMapInfo(){
    delete ui;
}

}
