#ifndef SWARMSIMULATOR_H
#define SWARMSIMULATOR_H

#include "QVector3D"
#include "QVector"

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
private:
    // Constants.
    unsigned short AGENTS_COUNT;
    double DENSITY;
    double WORLD_MAX_SIZE;
    QVector3D SWARM_CENTER;

    QVector<Uav> m_Agents;

    void InitSwarm();
    void SaveSwarmStateIntoDB();
    // TODO: Add simulation core.
};


}

#endif // SWARMSIMULATOR_H
