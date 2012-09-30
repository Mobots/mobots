#ifndef MOBOTS_INFO_H
#define MOBOTS_INFO_H

#include <QMainWindow>
#include <QLCDNumber>
#include <QLabel>
#include <QIcon>
#include <QString>
#include <QIcon>
#include <QTimer>


namespace Ui {
class Mobots_Info;
}

class Mobots_Info : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit Mobots_Info(QWidget *parent = 0);
    ~Mobots_Info();
    void addPicture(int id);
    void toogleMobotState(int id);
    void poseAbsolute(int id);

public slots:
    void refresh();

private:
    Ui::Mobots_Info *ui;

    // In den Attributen werden alle Zelleninhalte gespeichert,
    // um einen einfach zugriff zu gewähren.
    QLabel *enabled_1;
    QLabel *enabled_2;
    QLabel *enabled_3;

    QLabel *absolute_1;
    QLabel *absolute_2;
    QLabel *absolute_3;

    QLCDNumber *id_1;
    QLCDNumber *id_2;
    QLCDNumber *id_3;

    QLabel *coord_1;
    QLabel *coord_2;
    QLabel *coord_3;

    QLCDNumber *pictures_1;
    QLCDNumber *pictures_2;
    QLCDNumber *pictures_3;

    // In dem struct mobot_data werden die Daten der einzelnen Mobots gespeichert.
    struct mobot_data {
        int id;
        int pictures;
        bool enabled;
        bool absolute;
    } mobot1, mobot2, mobot3;


};

#endif // MOBOTS_INFO_H
