#include "mobots_info.h"
#include "ui_mobots_info.h"


Mobots_Info::Mobots_Info(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Mobots_Info)
{
    ui->setupUi(this); // User Inteface wird initialisiert

    // Die drei Mobots werden in ihre Grundzustände versetzt
    mobot1.id =  1;
    mobot1.pictures = 0;
    mobot1.enabled = false;
    mobot1.absolute = false;

    mobot2.id =  2;
    mobot2.pictures = 0;
    mobot2.enabled = false;
    mobot2.absolute = false;

    mobot3.id =  3;
    mobot3.pictures = 0;
    mobot3.enabled = false;
    mobot3.absolute = false;

    // Die Tabelle wird mit den jeweiligen QWidgets bestückt.
    for(int i=0; i<3; i++){
        for(int j=0; j<5; j++){
            if(j==0 || j==2){
                ui->mobot_table->setCellWidget(i,j, new QLCDNumber); // Die Zähler in Spalte 0 und 2
            }
            if(j==1 || j==3 || j==4){
               ui->mobot_table->setCellWidget(i,j, new QLabel); // Die Label für Bild- und Textanzeige
            }
        }
    }

    // Hier werden die jeweiligen Widgets aus den Zellen der Tabelle in die Attribute der Klasse gespeichert.
    // Man muss auf jedes einzeln und in der ganzen Klasse zugreifen können, um später die Tabelle
    // aktualisieren zu können.
    enabled_1 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(0,1)); // Die drei Zellen der Spalte "Enabled"
    enabled_2 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(1,1));
    enabled_3 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(2,1));
    enabled_1->setAlignment(Qt::AlignCenter); // Sie werden zentriert ausgerichtet.
    enabled_2->setAlignment(Qt::AlignCenter);
    enabled_3->setAlignment(Qt::AlignCenter);

    absolute_1 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(0,3)); // Die drei Zellen der Spalte "Absolute"
    absolute_2 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(1,3));
    absolute_3 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(2,3));
    absolute_1->setAlignment(Qt::AlignCenter); // Auch sie werden zentriert.
    absolute_2->setAlignment(Qt::AlignCenter);
    absolute_3->setAlignment(Qt::AlignCenter);

    id_1 = qobject_cast<QLCDNumber*>(ui->mobot_table->cellWidget(0,0)); // Die drei Zähler der "Id" Spalte
    id_2 = qobject_cast<QLCDNumber*>(ui->mobot_table->cellWidget(1,0));
    id_3 = qobject_cast<QLCDNumber*>(ui->mobot_table->cellWidget(2,0));

    coord_1 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(0,4));  // Die drei Spalten der "CoordSys" Spalte
    coord_2 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(1,4));
    coord_3 = qobject_cast<QLabel*>(ui->mobot_table->cellWidget(2,4));
    coord_1->setAlignment(Qt::AlignCenter); // zentriert
    coord_2->setAlignment(Qt::AlignCenter);
    coord_3->setAlignment(Qt::AlignCenter);

    pictures_1 = qobject_cast<QLCDNumber*>(ui->mobot_table->cellWidget(0,2)); // Die drei Bildzähler
    pictures_2 = qobject_cast<QLCDNumber*>(ui->mobot_table->cellWidget(1,2));
    pictures_3 = qobject_cast<QLCDNumber*>(ui->mobot_table->cellWidget(2,2));

    refresh(); // refresh wird zum Start einmal ausgeführt

    // Der QTimer führt den SLOT refresh einmal alle 2000 Millisekunden aus. Also wird die Tabelle alle
    // zwei Sekunden aktualisiert.
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(refresh()));
    timer->start(2000);


}

Mobots_Info::~Mobots_Info()
{
    delete ui;
}

void Mobots_Info::refresh(){

    QIcon off(":/icons/off"); // Die beiden Icons die den Status in
    QIcon on(":/icons/on");   // zwei Spalten anziegen.

    // Die Spalten "Enabled" und "Absolute" erhalten ihre Icons.
    // Dabei wird abgefragt, ob der Mobot aktiviert ist, oder ob
    // die Pose schon absolut ist. Je nach dem wird dann das "on" oder "off" Symbol angezeigt.
    enabled_1->setPixmap((mobot1.enabled ? on : off).pixmap(45,45));
    enabled_2->setPixmap((mobot2.enabled ? on : off).pixmap(45,45));
    enabled_3->setPixmap((mobot3.enabled ? on : off).pixmap(45,45));

    absolute_1->setPixmap((mobot1.absolute ? on : off).pixmap(45,45));
    absolute_2->setPixmap((mobot2.absolute ? on : off).pixmap(45,45));
    absolute_3->setPixmap((mobot3.absolute ? on : off).pixmap(45,45));

    id_1->display(mobot1.id); // Die Id Spalte zeigt die Ids der Mobots an.
    id_2->display(mobot2.id);
    id_3->display(mobot3.id);

    // Es wird angezeigt ob man sich noch im relativen Coordinatensystem befindet,
    // oder ob schon eine absolute Pose gefunden wurde.
    coord_1->setText(mobot1.absolute ? "absolute and relative" : "relative");
    coord_2->setText(mobot1.absolute ? "absolute and relative" : "relative");
    coord_3->setText(mobot1.absolute ? "absolute and relative" : "relative");

    // Die Zähler der Bilder zeigen die gespeicherten Werte an.
    pictures_1->display(mobot1.pictures);
    pictures_2->display(mobot2.pictures);
    pictures_3->display(mobot3.pictures);
}

// In dieser Funktion wird ein Bild zu einem bestimmten Mobot hinzugefügt.
void Mobots_Info::addPicture(int id){
    switch(id)
    {
    case 1:
        if(mobot1.pictures == 0) {
            toogleMobotState(1);
        }
        mobot1.pictures++;
        break;
    case 2:
        if(mobot2.pictures == 0) {
            toogleMobotState(2);
        }
        mobot2.pictures++;
        break;
    case 3:
        if(mobot3.pictures == 0) {
            toogleMobotState(3);
        }
        mobot3.pictures++;
        break;
    default:
        break;
    }
}

// In dieser Funktion kann der Aktivierungszustand des Mobots geändert werden.
void Mobots_Info::toogleMobotState(int id){
    switch(id)
    {
    case 1 : mobot1.enabled ? false : true;
        break;
    case 2 : mobot2.enabled ? false : true;
        break;
    case 3 : mobot3.enabled ? false : true;
        break;
    default :
        break;
    }
}

// Diese Funktion wird aufgerufen sobald der Mobot in das absolute Koordinatensystem wechselt.
void Mobots_Info::poseAbsolute(int id){
    switch(id)
    {
    case 1 : mobot1.absolute = true;
        break;
    case 2 : mobot2.absolute = true;
        break;
    case 3 : mobot3.absolute = true;
        break;
    default :
        break;
    }
}
