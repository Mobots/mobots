#ifndef IMAGE_MAP_MODEL_H
#define IMAGE_MAP_MODEL_H

#include <QAbstractTableModel>
#include <QFont>
#include <QBrush>
#include <QDebug>
#include <QTime>
#include <QTimer>

#include <vector>

#include <ros/ros.h>

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
private:
    std::vector< std::vector<int> > tableData;
};

}
#endif // IMAGE_MAP_MODEL_H
