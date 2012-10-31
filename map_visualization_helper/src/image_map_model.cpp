#include "image_map_model.h"

namespace map_visualization{

ImageMapModel::ImageMapModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    int char1[] = {0,1,1,23,1,-1};
    int char2[] = {0,2,0,3,1,0};
    std::vector<int> vect1(char1, char1 + sizeof(char1) / sizeof(int) );
    std::vector<int> vect2(char2, char2 + sizeof(char2) / sizeof(int) );
    tableData.push_back(vect1);
    tableData.push_back(vect2);
}

int ImageMapModel::rowCount(const QModelIndex & /*parent*/) const{
    return tableData.size();
}

int ImageMapModel::columnCount(const QModelIndex & /*parent*/) const
{
    return tableData[0].size();
}

QVariant ImageMapModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal) {
            switch (section)
            {
            case 0:
                return QString("Session ID");
            case 1:
                return QString("Mobot ID");
            case 2:
                return QString("Visible");
            case 3:
                return QString("Images");
            case 4:
                return QString("Relative");
            case 5:
                return QString("Absolute");
            }
        }
    }
    return QVariant();
}

QVariant ImageMapModel::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    int col = index.column();

    switch(role){
    case Qt::DisplayRole:
        if (col == ENABLED || col == RELATIVE || col == ABSOLUTE){
            if(tableData[row][col] == -1){
                return QString("Not Available");
            } else {
                return QString("Available");
            }
        }
        return QString("%1").arg(tableData[row][col]);
    case Qt::CheckStateRole:
        if (col == ENABLED || col == RELATIVE || col == ABSOLUTE){
            if(tableData[row][col] <= 0){
                return Qt::Unchecked;
            } else {
                return Qt::Checked;
            }
        }
        break;
    }
    return QVariant();
}

bool ImageMapModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
    if (role == Qt::EditRole)
    {
        //save value from editor to member m_gridData
        tableData[index.row()][index.column()] = value.toInt();
    }
    return true;
}

Qt::ItemFlags ImageMapModel::flags(const QModelIndex &index) const{
    int row = index.row();
    int col = index.column();
    if (col == 1 || col == 3 || col == 4){
        if(tableData[row][col] != -1){
            return Qt::ItemIsEnabled | Qt::ItemIsEditable;
        }
    }
    return Qt::ItemIsEnabled;
}

/**********************************************************
  * Table update Slot helpers
  ********************************************************/

/**
  * The slot which recieves the updates from the waypoint/ROS interface
  */
void ImageMapModel::updateData(int sessionID, int mobotID, int key, int value){
    // Clear data
    if(key == -1){
        clearData();
    }
    // Negative sign -> delete mobot/session entry
    if(sessionID < 0){
        sessionID *= -1;
        if(mobotID < 0){
            mobotID *= -1;
            removeMobot(sessionID, mobotID);
            return;
        }
        removeSession(sessionID);
        return;
    }
    // Find the mobot, and add/update entry
    for(int i = 0; i < tableData.size(); i++){
        if(tableData[i][SESSION] == sessionID){
            if(tableData[i][MOBOT] == mobotID){
                tableData[i][key] = value;
                return;
            }
        }
    }

    addMobot(sessionID, mobotID, key, value);
    return;
}

void ImageMapModel::addMobot(int sessionID, int mobotID, int key, int value){
    std::vector<int> dataEntry;
    dataEntry[SESSION] = sessionID;
    dataEntry[MOBOT] = mobotID;
    dataEntry[key] = value;
    tableData.push_back(dataEntry);

    Q_EMIT addWaypointMobot(mobotID);
    return;
}

void ImageMapModel::removeMobot(int sessionID, int mobotID){
    for(int i = 0; i < tableData.size(); i++){
        if(tableData[i][SESSION] == sessionID){
            if(tableData[i][MOBOT] == mobotID){
                tableData.erase(tableData.begin() + i);
            }
        }
    }

    Q_EMIT removeWaypointMobot(mobotID);
    return;
}

void ImageMapModel::removeSession(int sessionID){
    for(int i = 0; i < tableData.size(); i++){
        if(tableData[i][SESSION] == sessionID){
            tableData.erase(tableData.begin() + i);
            Q_EMIT removeWaypointMobot(tableData[i][MOBOT]);
        }
    }
    return;
}

void ImageMapModel::clearData(){
    tableData.clear();
    Q_EMIT clearWaypointMobot();
    return;
}

}
