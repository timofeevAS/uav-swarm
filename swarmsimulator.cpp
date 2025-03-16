#include "swarmsimulator.h"

#include <QRandomGenerator>

SwarmSimulator::SwarmSphereFormation::SwarmSphereFormation(int agentsCount, QVector3D swarmCenterPoint, double swarmDensity)
    : AGENTS_COUNT(agentsCount)
    , DENSITY(swarmDensity)
    , WORLD_MAX_SIZE(100.0) // Now using hard-code value for WORLD_MAX_SIZE.
    , SWARM_CENTER(swarmCenterPoint)
{
    InitSwarm();
}

void SwarmSimulator::SwarmSphereFormation::InitSwarm()
{
    // Using seed = 42 for repeatables.
    // TODO: Make parameter for seed.
    QRandomGenerator qrng(42);
    for (int i = 0; i < AGENTS_COUNT; i++)
    {
        // TODO: Decide what would be in z-coord.
        QVector3D randomUavStartPos(qrng.bounded(WORLD_MAX_SIZE),
                                    qrng.bounded(WORLD_MAX_SIZE),
                                    0);

    }
}

void SwarmSimulator::SwarmSphereFormation::SaveSwarmStateIntoDB()
{
    // TODO: I am planning to save states of Swarm into *.sqlite3 DB (file-database),
    // cause it useful way to save data.
}
