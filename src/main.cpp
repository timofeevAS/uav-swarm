#include "swarmsimulator/swarmsimulator.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    qDebug() << "Run sim.";
    // TODO: Make pretty beutiful part of running SwarmSimulator.
    QVector<Uav> uavs =
    {
        Uav(QVector3D(5, 10, 0), 100, 1, 1, 100),
        Uav(QVector3D(50, 50, 0), 100, 1, 2, 300),
        Uav(QVector3D(10, 10, 0), 100, 1, 3, 100),
        Uav(QVector3D(15, 15, 0), 100, 1, 4, 100),
        Uav(QVector3D(8, 10, 0), 100, 1, 5, 100),
    };

    QVector3D swarmCenter(10, 10, 10);
    for (Uav & uav : uavs)
    {
        uav.m_CurrentGoal = swarmCenter;
    }

    SwarmSimulator::SwarmSphereFormation sim(uavs, swarmCenter, 0.8);
    sim.RunSimulation(30);

    return a.exec();
}
