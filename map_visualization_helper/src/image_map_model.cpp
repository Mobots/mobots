#include "image_map_model.h"

namespace map_visualization{

ImageMapModel::ImageMapModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    int char1[] = {1,1,23,1,1};
    int char2[] = {2,0,3,1,0};
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
    return 5;
}

QVariant ImageMapModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal) {
            switch (section)
            {
            case 0:
                return QString("Mobot ID");
            case 1:
                return QString("Visible");
            case 2:
                return QString("Images");
            case 3:
                return QString("Relative");
            case 4:
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
    // generate a log message when this method gets called
    qDebug() << QString("row %1, col%2, role %3")
                .arg(row).arg(col).arg(role);

    switch(role){
    case Qt::DisplayRole:
        return QString("%1").arg(tableData[row][col]);
        break;
    case Qt::CheckStateRole:

        if (row == 1 && col == 0) //add a checkbox to cell(1,0)
        {
            return Qt::Checked;
        }
    }
    return QVariant();
}

}
