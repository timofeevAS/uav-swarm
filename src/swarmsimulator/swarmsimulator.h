#ifndef SWARMSIMULATOR_H
#define SWARMSIMULATOR_H

#include "QVector3D"
#include "QVector"
#include <QtSql/QSqlDatabase>

#include "../uav/uav.h"

namespace SwarmSimulator
{

// Simulate swarm formation with boids [1] principles.
//
// [1] http://www.kfish.org/boids/pseudocode.html
class SwarmSphereFormation
{
public:
    SwarmSphereFormation(const QVector<Uav> & uavs,
                         QVector3D swarmCenterPoint,
                         double swarmDensity);
    ~SwarmSphereFormation();

    void RunSimulation(int steps = 5);
private:
    // Constants.
    unsigned short AGENTS_COUNT;
    double DENSITY;
    int16_t WORLD_MAX_SIZE;
    QVector3D SWARM_CENTER;
    int EXPERIMENT_ID;

    QVector<Uav> m_Agents;
    int m_CurrentIteration;
    QSqlDatabase m_Database;

    void InitSwarm();
    void SaveSwarmStateIntoDB();
    void BoidsStep();
    void IsNeedStop();
};


}

#endif // SWARMSIMULATOR_H
