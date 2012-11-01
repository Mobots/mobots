#ifndef IMAGE_MAP_MODEL_H
#define IMAGE_MAP_MODEL_H

#include <QAbstractTableModel>
#include <QtGui/QMouseEvent>
#include <QString>

#include <vector>

// The row in which the data is located in the vector
static const int SESSION =  0;
static const int MOBOT =    1;
static const int ENABLED =  2;
static const int IMAGES  =  3;
static const int RELATIVE = 4;
static const int ABSOLUTE = 5;

namespace map_visualization{

class ImageMapModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    ImageMapModel(QObject *parent);
    int rowCount(const QModelIndex &parent = QModelIndex()) const ;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);
    Qt::ItemFlags flags(const QModelIndex &index) const;

    void clearData();
private:
    void addMobot(int sessionID, int mobotID, int key, int value);
    void removeMobot(int sessionID, int mobotID);
    void removeSession(int sessionID);
    // TODO support sessions
    //{sessionID, mobotID,enabled,images,rel_pose,abs_pose}
    std::vector< std::vector<int> > tableData;
    int activeMobot;
public Q_SLOTS:
    void updateData(int sessionID, int mobotID, int key, int value);
Q_SIGNALS:
     void removeWaypointMobot(int mobotID);
     void addWaypointMobot(int mobotID);
     void clearWaypointMobot();
};

}
#endif // IMAGE_MAP_MODEL_H
