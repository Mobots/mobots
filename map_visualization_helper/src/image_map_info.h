#ifndef IMAGE_MAP_INFO_H
#define IMAGE_MAP_INFO_H

#include <QtGui>
#include "image_map_model.h"
#include "image_map_waypoint.h"

namespace map_visualization{

class ImageMapInfo : public QMainWindow {
Q_OBJECT

public:
    ImageMapInfo(int argc, char** argv, QWidget *parent = 0);
private:
    ImageMapModel model;
    ImageMapWaypoint waypoint;
    QComboBox* waypointComboBox;
    QComboBox* replayComboBox;
public Q_SLOTS:
    void removeWaypointMobot(int mobotID);
    void addWaypointMobot(int mobotID);
};

}
#endif // IMAGE_MAP_INFO_H
