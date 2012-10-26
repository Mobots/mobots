#include "image_map_model.h"

namespace map_visualization{

ImageMapModel::ImageMapModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    int char1[] = {1,1,23,1,-1};
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

    switch(role){
    case Qt::DisplayRole:
        if (col == 1 || col == 3 || col == 4){
            if(tableData[row][col] == -1){
                return QString("Not Available");
            } else {
                return QString("Available");
            }
        }
        return QString("%1").arg(tableData[row][col]);
        break;
    case Qt::CheckStateRole:
        if (col == 1 || col == 3 || col == 4)
        {
            if(tableData[row][col] <= 0){
                return Qt::Unchecked;
            } else {
                return Qt::Checked;
            }
        }
    }
    return QVariant();
}

bool ImageMapModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
    if (role == Qt::EditRole)
    {
        //save value from editor to member m_gridData
        tableData[index.row()][index.column()] = value.toInt();
        //for presentation purposes only: build and emit a joined string
        QString result;
        for(int row= 0; row < tableData.size(); row++)
        {
            for(int col= 0; col < tableData[0].size(); col++)
            {
                result += tableData[row][col] + " ";
            }
        }
        Q_EMIT editCompleted( result );
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

}
