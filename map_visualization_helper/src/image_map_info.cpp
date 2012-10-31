#include "image_map_info.h"

namespace map_visualization{

ImageMapInfo::ImageMapInfo(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , model(parent)
    , waypoint(argc, argv)
{
    QHBoxLayout* headerBox = new QHBoxLayout();
    QLabel* waypointLabel = new QLabel("Active Mobot");
    waypointComboBox = new QComboBox(0);
    //waypointComboBox->setModel(&imageMapModel);
    headerBox->addWidget(waypointLabel);
    headerBox->addWidget(waypointComboBox);
    headerBox->addStretch(0);
    QLabel* replayLabel = new QLabel("Replay Session");
    replayComboBox = new QComboBox(0);
    headerBox->addWidget(replayLabel);
    headerBox->addWidget(replayComboBox);
    headerBox->addStretch(0);

    QVBoxLayout* centralWidget = new QVBoxLayout();
    QTableView* infoTable = new QTableView();
    infoTable->setModel(&model);
    centralWidget->addLayout(headerBox);
    centralWidget->addWidget(infoTable);

    QWidget* window = new QWidget(0);
    window->setLayout(centralWidget);
    window->setWindowTitle("Image Map Info");
    window->show();

    waypoint.start();
    // TODO connect updateRviz
    QObject::connect(&waypoint, SIGNAL(dataChanged(int,int,int,int)),
                     &model, SLOT(updateData(int,int,int,int)));
    QObject::connect(waypointComboBox, SIGNAL(currentIndexChanged(QString)),
                     &waypoint, SLOT(setActiveMobot(QString)));
    QObject::connect(&model, SIGNAL(addWaypointMobot(int)),
                     this, SLOT(addWaypointMobot(int)));
    QObject::connect(&model, SIGNAL(removeWaypointMobot(int)),
                     this, SLOT(removeWaypointMobot(int)));
    QObject::connect(&model, SIGNAL(clearWaypointMobot()),
                     waypointComboBox, SLOT(clear()));

}

void ImageMapInfo::addWaypointMobot(int mobotID){
    QString mobotID_ = QString::number(mobotID);
    waypointComboBox->addItem(mobotID_);
}

void ImageMapInfo::removeWaypointMobot(int mobotID){
    QString mobotID_ = QString::number(mobotID);
    int i = waypointComboBox->findText(mobotID_);
    waypointComboBox->removeItem(i);
}

}
