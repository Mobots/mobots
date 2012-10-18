#include "image_map_model.h"

namespace map_visualization_helper{

ImageMapModel::ImageMapModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    timer = new QTimer(this);
    timer->setInterval(1000);
    connect(timer, SIGNAL(timeout()) , this, SLOT(timerHit()));
    timer->start();
}

int ImageMapModel::rowCount(const QModelIndex & /*parent*/) const
{
   return 2;
}

int ImageMapModel::columnCount(const QModelIndex & /*parent*/) const
{
    return 3;
}

QVariant ImageMapModel::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    int col = index.column();

    if (role == Qt::DisplayRole)
    {
        if (row == 0 && col == 0)
        {
            return QTime::currentTime().toString();
        }
    }
    return QVariant();
}

void ImageMapModel::timerHit()
{
    //we identify the top left cell
    qDebug() << QString("row , col, role");
    QModelIndex topLeft = createIndex(0,0);
    //emit a signal to make the view reread identified data
    Q_EMIT dataChanged(topLeft, topLeft);
}

}
