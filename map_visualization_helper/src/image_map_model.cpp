#include "image_map_model.h"

namespace map_visualization{

ImageMapModel::ImageMapModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    /*int char1[] = {0,0,1,23,1,-1};
    int char2[] = {0,1,0,3,1,0};
    std::vector<int> vect1(char1, char1 + sizeof(char1) / sizeof(int) );
    std::vector<int> vect2(char2, char2 + sizeof(char2) / sizeof(int) );
    tableData.push_back(vect1);
    tableData.push_back(vect2);*/
}

int ImageMapModel::rowCount(const QModelIndex & /*parent*/) const{
    return tableData.size();
}

int ImageMapModel::columnCount(const QModelIndex & /*parent*/) const
{
    return COLUMN_COUNT;
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
    if (role == Qt::EditRole){
        int row = index.row();
        int col = index.column();
        //save value from editor to member m_gridData
        tableData[row][col] = value.toInt();
        switch(col){
        case ABSOLUTE: // Flip flop between absolute and relative
            if(value == 1){
                if(tableData[row][RELATIVE] == 1){
                    tableData[row][RELATIVE] = 0;
                }
            }
            break;
        case RELATIVE: // Flip flop between absolute and relative
            if(value == 1){
                if(tableData[row][ABSOLUTE] == 1){
                    tableData[row][ABSOLUTE] = 0;
                }
            }
            break;
        }
        Q_EMIT tableChanged(tableData[row][SESSION], tableData[row][MOBOT], col, value.toInt());
    }
    return true;
}

Qt::ItemFlags ImageMapModel::flags(const QModelIndex &index) const{
    int row = index.row();
    int col = index.column();
    if (col == ENABLED || col == RELATIVE || col == ABSOLUTE){
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
void ImageMapModel::updateTable(int sessionID, int mobotID, int key, int value){
    // Clear data
    if(key == -1){
        ROS_INFO("[updateTable] clear");
        removeRows(0,tableData.size());
    }
    // Negative sign -> delete mobot/session entry
    if(sessionID < 0){
        ROS_INFO("[updateTable] delete");
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
                // Entry specific behavoir
                switch(key){
                case ABSOLUTE: // Flip flop between absolute and relative
                    if(value == 1){
                        if(tableData[i][RELATIVE] == 1){
                            tableData[i][RELATIVE] = 0;
                        }
                    }
                    break;
                case RELATIVE: // Flip flop between absolute and relative
                    if(value == 1){
                        if(tableData[i][ABSOLUTE] == 1){
                            tableData[i][ABSOLUTE] = 0;
                        }
                    }
                    break;
                }

                return;
            }
        }
    }
    // Add a mobot Entry
    insertRows(mobotID, 1);
    for(int i = 0; i < tableData.size(); i++){
        if(tableData[i][SESSION] == -1){
            for(int j = i; j < tableData.size(); j++){
                if(tableData[j][MOBOT] == -1){
                    tableData[j][SESSION] = sessionID;
                    tableData[j][MOBOT] = mobotID;
                    tableData[j][key] = value;
                    Q_EMIT addWaypointMobot(mobotID);
                    return;
                }
            }
        }
    }
    return;
}

void ImageMapModel::removeMobot(int sessionID, int mobotID){
    for(int i = 0; i < tableData.size(), i++;){
        if(tableData[i][SESSION] == sessionID){
            for(int j = i; j < tableData.size(); j++){
                if(tableData[j][MOBOT] == mobotID){
                    if(tableData.back()[SESSION] == sessionID){
                        if(tableData.back()[MOBOT] == mobotID){
                            tableData.erase(tableData.begin() + j, tableData.end() - 1);
                            return;
                        }
                        for(int l = j; l < tableData.size(); l++){
                            if(tableData[l][MOBOT] != mobotID){
                                l--;
                                tableData.erase(tableData.begin() + j, tableData.begin() + l);
                                return;
                            }
                        }
                    }
                    for(int k = j; k < tableData.size(); k++){
                        if(tableData[k][SESSION] != sessionID){
                            for(int l = k; l < tableData.size(); l++){
                                if(tableData[l][MOBOT] != mobotID){
                                    l--;
                                    tableData.erase(tableData.begin() + j, tableData.begin() + l);
                                    return;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void ImageMapModel::removeSession(int sessionID){
    for(int i = 0; i < tableData.size(), i++;){
        if(tableData[i][SESSION] == sessionID){
            if(tableData.back()[SESSION] == sessionID){
                tableData.erase(tableData.begin() + i, tableData.end() - 1);
                return;
            }
            for(int j = i; j < tableData.size(); j++){
                if(tableData[j][SESSION] != sessionID){
                    j--;
                    tableData.erase(tableData.begin() + i, tableData.begin() + j);
                    return;
                }
            }
        }
    }
}

// has to be called after to sync the combotbox Q_EMIT addWaypointMobot(mobotID);
bool ImageMapModel::insertRows(int row, int count, const QModelIndex & parent){
    beginInsertRows(parent, row, row + count - 1);
    std::vector<int> dataEntry(COLUMN_COUNT, -1);
    std::vector< std::vector<int> >::iterator it;
    it = tableData.begin();
    if(tableData.size() < row){ // row exceeds vector size
        tableData.insert(it + tableData.size(), count, dataEntry);
    } else if(row < 0){ // row is less than 0
        tableData.insert(it, count, dataEntry);
    } else { // row is within vector
        tableData.insert(it + row, count, dataEntry);
    }
    endInsertRows();
    return true;
}

// has to be called after to sync the combotbox Q_EMIT removeWaypointMobot(mobotID);
bool ImageMapModel::removeRows(int row, int count, const QModelIndex & parent){
    std::vector< std::vector<int> >::iterator it;
    it = tableData.begin();
    beginRemoveRows(parent, row, row + count - 1);
    if(row < 0){
        if(tableData.size() < count){
            tableData.erase(it, tableData.end() - 1);
        } else {
            tableData.erase(it, it + count - 1);
        }
    } else if(row > tableData.size()){
        return true;
    } else {
        if(tableData.size() < count + row){
            tableData.erase(it + row, tableData.end() - 1);
        } else {
            tableData.erase(it + row, it + count - 1);
        }
    }
    endRemoveRows();
    return true;
}

}
