#ifndef IMAGE_MAP_INFO_H
#define IMAGE_MAP_INFO_H

#include <QtGui>
#include "image_map_model.h"
#include "image_map_waypoint.h"

namespace map_visualization{

class ImageMapInfo : public QMainWindow {
Q_OBJECT

public:
    ImageMapInfo(int argc, char** argv, QWidget *parent);
private:
    ImageMapModel model;
    ImageMapWaypoint waypoint;
};

}
#endif // IMAGE_MAP_INFO_H
