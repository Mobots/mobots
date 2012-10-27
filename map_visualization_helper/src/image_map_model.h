#ifndef IMAGE_MAP_MODEL_H
#define IMAGE_MAP_MODEL_H

#include <QAbstractTableModel>
#include <QtGui/QMouseEvent>
#include <QString>

#include <vector>

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

    void addMobot(int sessionID, int mobotID, bool visible, int images, bool relative, bool absolute);
    void updateMobot(int sessionID, int mobotID, int visible, int images, int relative, int absolute);
    void removeMobot(int sessionID, int mobotID);
    void removeSession(int sessionID);
    void clear();
private:
    // TODO support sessions
    //{mobotID,enabled,images,rel_pose,abs_pose}
    std::vector< std::vector<int> > tableData;
    int activeMobot;
Q_SIGNALS:
     void editCompleted(const QString &);
};

}
#endif // IMAGE_MAP_MODEL_H
