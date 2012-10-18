#ifndef IMAGE_MAP_MODEL_H
#define IMAGE_MAP_MODEL_H

#include <QAbstractTableModel>
#include <QFont>
#include <QBrush>
#include <QDebug>
#include <QTime>
#include <QTimer>

#include <ros/ros.h>

namespace map_visualization_helper{

class ImageMapModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    ImageMapModel(QObject *parent);
    int rowCount(const QModelIndex &parent = QModelIndex()) const ;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QTimer* timer;
private Q_SLOTS:
    void timerHit();
};

}
#endif // IMAGE_MAP_MODEL_H
