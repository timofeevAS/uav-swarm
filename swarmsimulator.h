#ifndef SWARMSIMULATOR_H
#define SWARMSIMULATOR_H

#include "QVector3D"
#include "QVector"
#include <QtSql/QSqlDatabase>

#include "uav.h"

namespace SwarmSimulator
{

// Simulate swarm formation with boids [1] principles.
//
// [1] http://www.kfish.org/boids/pseudocode.html
class SwarmSphereFormation
{
public:
    SwarmSphereFormation(int agentsCount, QVector3D swarmCenterPoint, double swarmDensity);
    ~SwarmSphereFormation();
private:
    // Constants.
    unsigned short AGENTS_COUNT;
    double DENSITY;
    double WORLD_MAX_SIZE;
    QVector3D SWARM_CENTER;
    int EXPERIMENT_ID;

    QVector<Uav> m_Agents;
    int m_CurrentIteration;
    QSqlDatabase m_Database;

    void InitSwarm();
    void SaveSwarmStateIntoDB();
    // TODO: Add simulation core.
};


}

#endif // SWARMSIMULATOR_H
