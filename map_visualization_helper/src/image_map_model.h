#ifndef IMAGE_MAP_MODEL_H
#define IMAGE_MAP_MODEL_H

#include <QAbstractTableModel>
#include <QtGui/QMouseEvent>
#include <QString>

#include <vector>

#include <ros/ros.h>

#include <map_visualization/definitions.h>

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
    void removeMobot(int sessionID, int mobotID);
    void removeSession(int sessionID);
    bool insertRows(int position, int rows, const QModelIndex &index = QModelIndex());
    bool removeRows(int position, int rows, const QModelIndex &index = QModelIndex());

private:
    // TODO support sessions properly
    std::vector< std::vector<int> > tableData;
    int activeMobot;
public Q_SLOTS:
    void updateTable(int sessionID, int mobotID, int key, int value);
Q_SIGNALS:
    void tableChanged(int session, int mobot, int key, int value);
    void removeWaypointMobot(int mobotID);
    void addWaypointMobot(int mobotID);
    void clearWaypointMobot();
};

}
#endif // IMAGE_MAP_MODEL_H
