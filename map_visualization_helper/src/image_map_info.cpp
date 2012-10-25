#include "image_map_info.h"

int main(int argc, char *argv[]){
    QApplication app(argc, argv);
    map_visualization::ImageMapModel imageMapModel(0);

    QHBoxLayout* headerBox = new QHBoxLayout();
    QLabel* waypointLabel = new QLabel("Active Mobot");
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

    map_visualization::ImageMapWaypoint waypoint;
    waypoint.start();
    qDebug() << "hello from GUI thread " << app.thread()->currentThreadId();
    waypoint.wait();  // do not exit before the thread is completed!

    return app.exec();
}
