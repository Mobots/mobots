#include "image_map_info.h"

int main(int argc, char *argv[]){
    map_visualization::ImageMapWaypoint imageMapWaypoint(0);
    QApplication a(argc, argv);
    map_visualization::ImageMapModel imageMapModel(0);

    QHBoxLayout* headerBox = new QHBoxLayout();
    QLabel* sessionLabel = new QLabel("Session");
    QComboBox* sessionComboBox = new QComboBox(0);
    sessionComboBox->setModel(&imageMapModel);
    headerBox->addWidget(sessionLabel);
    headerBox->addWidget(sessionComboBox);
    headerBox->addStretch(0);
    QLabel* waypointLabel = new QLabel("waypoint for Mobot");
    QComboBox* waypointComboBox = new QComboBox(0);
    waypointComboBox->setModel(&imageMapModel);
    headerBox->addWidget(waypointLabel);
    headerBox->addWidget(waypointComboBox);
    headerBox->addStretch(0);

    QVBoxLayout* centralWidget = new QVBoxLayout();
    QTableView* infoTable = new QTableView();
    infoTable->setModel(&imageMapModel);
    centralWidget->addLayout(headerBox);
    centralWidget->addWidget(infoTable);

    QWidget* window = new QWidget(0);
    window->setLayout(centralWidget);
    window->setWindowTitle("Image Map Info");
    window->show();

    return a.exec();
}