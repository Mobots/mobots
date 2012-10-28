#include "image_map_info.h"

namespace map_visualization{

ImageMapInfo::ImageMapInfo(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , model(parent)
    , waypoint(argc, argv)
{
    QHBoxLayout* headerBox = new QHBoxLayout();
    QLabel* waypointLabel = new QLabel("Active Mobot");
    QComboBox* waypointComboBox = new QComboBox(0);
    //waypointComboBox->setModel(&imageMapModel);
    headerBox->addWidget(waypointLabel);
    headerBox->addWidget(waypointComboBox);
    headerBox->addStretch(0);
    QLabel* replayLabel = new QLabel("Replay Session");
    QComboBox* replayComboBox = new QComboBox(0);
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

    QObject::connect(waypointComboBox, SIGNAL(currentIndexChanged(int)),
                     &waypoint, SLOT(setActiveMobot(int)));
}

}
