#include "swarmsimulator.h"

#include <QRandomGenerator>
#include <QDateTime>
#include <QProcessEnvironment>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>

typedef QVector<QVector3D> Points;

static Points fibonacciSphere(int samples, double r)
{
    Points positions;
    double offset = 2.0 / samples;
    double increment = M_PI * (3.0 - std::sqrt(5.0));

    for (int i = 0; i < samples; i++)
    {
        double y = ((i * offset) - 1) + (offset / 2);
        double r_xy = std::sqrt(1 - y * y);
        double phi = i * increment;
        double x = std::cos(phi) * r_xy;
        double z = std::sin(phi) * r_xy;
        QVector3D point = QVector3D(x, y, z) * r;

        positions.emplace_back(point);
    }

    return positions;
}

static Points makeSwarmPositions(uint16_t N, double density, double r, const QVector3D & center)
{
    Points positions;
    // Радиус слоя задаем изначально как 2r.
    double layerRadius = 2 * r;
    while (true)
    {
        uint16_t numPositions = static_cast<uint16_t>(positions.size());
        uint16_t remaining = N - numPositions;

        if (remaining == 0)
        {
            break;
        }

        // Число точек на слое.
        uint16_t numOnLayer = (4 * M_PI * (layerRadius*layerRadius) / (4 * r * r));
        numOnLayer = std::min(numOnLayer, remaining);

        // Получаем точки с помощью Fibonacci Sphere.
        Points newPoints = fibonacciSphere(numOnLayer, layerRadius);

        // Проверка на пересечения.
        for (size_t i = 0; i < newPoints.size(); i++)
        {
            bool hasIntersaction = false;
            for (size_t j = 0; j < newPoints.size(); j++)
            {
                if (i != j && newPoints[i].distanceToPoint(newPoints[j]) <= 2*r)
                {
                    hasIntersaction = true;
                    break;
                }
            }
            if (!hasIntersaction)
            {
                positions.emplace_back(newPoints[i]);
            }
        }

        // Увеличиваем радиус оболочки.
        layerRadius += 2 * r;
    }

    for (uint16_t i = 0; i < N; i++)
    {
        positions[i] *= (1.0 / density);
    }

    for (uint16_t i = 0; i < N; i++)
    {
        positions[i] += center;
    }

    return positions;
}

SwarmSimulator::SwarmSphereFormation::SwarmSphereFormation(const QVector<Uav> &uavs, QVector3D swarmCenterPoint, double swarmDensity)
    : AGENTS_COUNT(uavs.size())
    , DENSITY(swarmDensity)
    , WORLD_MAX_SIZE(100) // Now using hard-code value for WORLD_MAX_SIZE.
    , SWARM_CENTER(swarmCenterPoint)
    , m_CurrentIteration(0)
{

    // Configure connection for sqlite3 database.
    m_Database = QSqlDatabase::addDatabase("QSQLITE");

    // Get database name fron env variable: UAV_DB;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    m_Database.setDatabaseName(env.value("UAV_DB"));

    // Get database name from system variable.
    if (!m_Database.open())
    {
        qWarning() << "Failed to open database:" << m_Database.lastError().text();
        return;
    }

    // Get new experiment id for current simulation.
    QSqlQuery query("SELECT MAX(experiment_id) FROM swarm_state", m_Database);
    query.exec();
    if (query.next() && !query.value(0).isNull())
    {
        EXPERIMENT_ID = query.value(0).toInt() + 1;
    }
    else
    {
        EXPERIMENT_ID = 1;
    }

    // TODO: temporary clear database.
    QSqlQuery deleteQuery("DELETE FROM swarm_state", m_Database);
    deleteQuery.next();

    m_Agents = uavs;
    SaveSwarmStateIntoDB();
}
SwarmSimulator::SwarmSphereFormation::~SwarmSphereFormation()
{
    m_Database.close();
}

void SwarmSimulator::SwarmSphereFormation::SaveSwarmStateIntoDB()
{
    QSqlQuery query(m_Database);
    query.prepare("INSERT INTO swarm_state "
                  "(timestamp, iteration, "
                  "position_x, position_y, position_z, "
                  "speed_x, speed_y, speed_z, "
                  "state, experiment_id, uav_id, "
                  "target_x, target_y, target_z) "
                  "VALUES (:timestamp, :iteration, "
                  ":position_x, :position_y, :position_z, "
                  ":speed_x, :speed_y, :speed_z, "
                  ":state, :experiment_id, :uav_id, "
                  ":target_x, :target_y, :target_z) ");

    for(QVector<Uav>::ConstIterator it = m_Agents.constBegin(); it != m_Agents.constEnd(); ++it)
    {
        QVector3D agentPosition = it->m_CurrentPosition;
        QVector3D agentSpeed = it->m_Direction;
        QVector3D agentTarget = it->m_CurrentGoal;
        uint16_t agentId = it->m_Id;

        query.bindValue(":timestamp", QDateTime::currentDateTimeUtc());
        query.bindValue(":iteration", m_CurrentIteration);
        query.bindValue(":position_x", agentPosition.x());
        query.bindValue(":position_y", agentPosition.y());
        query.bindValue(":position_z", agentPosition.z());
        query.bindValue(":speed_x", agentSpeed.x());
        query.bindValue(":speed_y", agentSpeed.y());
        query.bindValue(":speed_z", agentSpeed.z());
        query.bindValue(":state", "");
        query.bindValue(":experiment_id", EXPERIMENT_ID);
        query.bindValue(":uav_id", agentId);
        query.bindValue(":target_x", agentTarget.x());
        query.bindValue(":target_y", agentTarget.y());
        query.bindValue(":target_z", agentTarget.z());

        if (!query.exec())
        {
            qWarning() << "Failed to insert data:" << query.lastError().text();
        }
    }
}

void SwarmSimulator::SwarmSphereFormation::BoidsStep()
{
    QVector<QVector<double>> distances(AGENTS_COUNT, QVector<double>(AGENTS_COUNT, -1));
    for(int i = 0; i < AGENTS_COUNT; i++)
    {
        for(int j = 0; j < AGENTS_COUNT; j++)
        {
            double realDistance =
                m_Agents[i].m_CurrentPosition.distanceToPoint(m_Agents[j].m_CurrentPosition) -
                                  m_Agents[i].m_SelfRadius - m_Agents[j].m_SelfRadius;
            distances[i][j] = realDistance;
            distances[j][i] = realDistance;
        }
    }

    // Move each agents;
    for(int i = 0; i < AGENTS_COUNT; i++)
    {
        Uav &agent = m_Agents[i];
        if (agent.m_State == Uav::State::FINISH)
        {
            continue;
        }
        // Find neighbours.
        QVector<Uav*> neighbours;
        size_t neighboursCount = 0;
        for (int j = 0; j < AGENTS_COUNT; j++)
        {
            if (i != j && distances[i][j] <= agent.m_InterractionRadius)
            {
                neighbours.push_back(&m_Agents[j]);
                neighboursCount++;
            }
            else
            {
                neighbours.push_back(nullptr);
            }
        }

        // TODO: mind about it.
        // dynamicMaxVelocity = agent.m_MaxVelocity * (SOME COEFFICIENT);

        // Rule `Cohesion`: steer to move towards the average position (center of mass) of local flockmates;
        QVector3D cohesion;
        for(int j = 0; j < AGENTS_COUNT; j++)
        {
            const Uav *neighbour = neighbours[j];
            if (!neighbour)
            {
                continue;
            }

            if (neighbour->m_State != Uav::State::FINISH)
            {
                cohesion += neighbour->m_CurrentPosition;
            }
        }

        // Вычисление среднего положения для когезии
        if (neighboursCount > 0)
        {
            cohesion /= neighboursCount; // Вычисляем центр масс соседей (не включая самого агента)
        }
        // Для направления агента к центру масс, вычитаем его позицию.
        cohesion -= agent.m_CurrentPosition;

        if (!cohesion.isNull())
        {
            cohesion.normalize();
        }

        // Rule `Separation`: steer avoid crowding local flockmates;
        QVector3D separation;
        for(int j = 0; j < AGENTS_COUNT; j++)
        {
            const Uav *neighbour = neighbours[j];
            if (!neighbour)
            {
                continue;
            }
            // Расстояние между агентом и соседом
            double distance = distances[i][j];

            // Если сосед слишком близко, отталкиваемся от него
            if (distance < agent.m_SelfRadius + neighbour->m_SelfRadius)
            {
                // Вектор от соседа к текущему агенту
                QVector3D separationVector = agent.m_CurrentPosition - neighbour->m_CurrentPosition;

                // Усиливаем эффект отталкивания, пропорционально обратному квадрату расстояния
                separationVector /= distance * distance;

                // Добавляем этот вектор к общему вектору разделения
                separation += separationVector;
            }
        }

        if (!separation.isNull())
        {
            separation.normalize();
        }
        // Rule `Alignment`: steer towards the average heading of local flockmates;
        QVector3D alignment;
        for (const Uav* neighbour : neighbours)
        {
            if (neighbour && neighbour->m_State != Uav::State::FINISH)
            {
                // Суммируем скорости всех соседей
                alignment += neighbour->m_Direction;
            }
        }

        if (neighboursCount > 0)
        {
            alignment /= neighboursCount; // Вычисляем среднее направление
        }

        // Для направления агента в сторону среднего направления, вычитаем его текущую скорость.
        alignment -= agent.m_Direction;

        if (alignment.length() > agent.m_MaxVelocity)
        {
            alignment = alignment.normalized();
        }

        // Rule `Tendency`: steer towards self point;
        QVector3D tendency;
        tendency = (agent.m_CurrentGoal - agent.m_CurrentPosition); // TODO: prove it.
        if (!tendency.isNull())
        {
            tendency.normalize();
        }

        QVector3D newDirection = agent.m_Direction + 0.5 * cohesion + separation + 0.5 * alignment + 0.8 * tendency;
        newDirection.normalize();
        newDirection *= agent.m_MaxVelocity;

        agent.m_Direction = newDirection / 10;
    }

    for(Uav & agent : m_Agents)
    {
        if (agent.m_State == Uav::State::MOVE)
        {
               agent.m_CurrentPosition = agent.m_CurrentPosition + agent.m_Direction;
        }
    }
}

void SwarmSimulator::SwarmSphereFormation::IsNeedStop()
{
    for (uint16_t i = 0; i < AGENTS_COUNT; i++)
    {
        Uav agent = m_Agents[i];
        if (agent.m_CurrentGoal.distanceToPoint(agent.m_CurrentPosition) <= 0.1)
        {
            agent.m_State = Uav::State::FINISH;
        }
    }
}

void SwarmSimulator::SwarmSphereFormation::RunSimulation(int steps)
{
    m_CurrentIteration = 0;

    // Generate swarm positions.
    double safeR = m_Agents.back().m_SelfRadius;
    Points targets = makeSwarmPositions(AGENTS_COUNT, DENSITY, safeR, SWARM_CENTER);

    Q_ASSERT(targets.size() == AGENTS_COUNT);

    for (int i = 0; i < AGENTS_COUNT; i++)
    {
        m_Agents[i].m_CurrentGoal = targets[i];
    }

    while (m_CurrentIteration < steps)
    {
        IsNeedStop();
        BoidsStep();
        SaveSwarmStateIntoDB();
        m_CurrentIteration++;
    }
}
