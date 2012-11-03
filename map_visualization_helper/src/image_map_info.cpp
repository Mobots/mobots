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

    // Running ROS communication in a thread
    QThread* thread = new QThread;
    ImageMapWaypoint* waypoint = new ImageMapWaypoint(argc, argv);
    waypoint->moveToThread(thread);
    // Thread/Waypoint Connections
    // TODO clean shutdown through gui quit
    QObject::connect(thread, SIGNAL(started()), waypoint, SLOT(process()));
    QObject::connect(waypoint, SIGNAL(finished()), thread, SLOT(quit()));
    QObject::connect(waypoint, SIGNAL(finished()), waypoint, SLOT(deleteLater()));
    QObject::connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();

    // TODO connect updateRviz
    // Model/Waypoint connections
    QObject::connect(&model, SIGNAL(tableChanged(int,int,int,int)),
                     waypoint, SLOT(updateRviz(int,int,int,int)));
    QObject::connect(waypoint, SIGNAL(rvizChanged(int,int,int,int)),
                     &model, SLOT(updateTable(int,int,int,int)));
    QObject::connect(waypointComboBox, SIGNAL(currentIndexChanged(QString)),
                     waypoint, SLOT(setActiveMobot(QString)));
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
