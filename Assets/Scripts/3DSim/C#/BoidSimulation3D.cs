using System;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;

public class BoidSimulation3D : MonoBehaviour
{
    [Header("Enviornment")]
    [SerializeField] Vector3 margin         = new(95, 55, 80);
    [SerializeField] float boidRadius       = .1f;
    [SerializeField] uint numOfBoids        = 200;
    [SerializeField] uint simSteps          = 120;

    [Header("Factors")]
    [SerializeField] float avoidFactor      = .03f;
    [SerializeField] float matchingFactor   = .001f;
    [SerializeField] float centeringFactor  = .003f;

    [Header("Other Boid Variables")]
    [SerializeField] float turnFactor       = .15f;
    [SerializeField] uint maxSpeed          = 8;
    [SerializeField] uint minSpeed          = 5;
    [SerializeField] uint sightRange        = 10;
    [SerializeField] uint dangerRange       = 1;

    [Header("Rules")]
    [SerializeField] bool seperation        = true;
    [SerializeField] bool alignment         = true;
    [SerializeField] bool cohesion          = true;

    [Header("Visuals")]
    [SerializeField] bool visualSeperation  = false;
    [SerializeField] bool visualAlignment   = false;
    [SerializeField] bool visualCohesion    = false;
    [SerializeField] Color colorSeperation  = new(1, .92f, .016f, .1f);
    [SerializeField] Color colorAlignment   = new(1, 0, 0, .1f);
    [SerializeField] Color colorCohesion    = new(0, 1, 0, .1f);

    Boid3D[] boids;
    Vector3[] centerOfMass;
    System.Random rand;

    float rightMargin;
    float leftMargin;
    float topMargin;
    float bottomMargin;
    float backMargin;
    float frontMargin;


    private void Start()
    {
        rand = new System.Random();
        boids = new Boid3D[numOfBoids];
        centerOfMass = new Vector3[numOfBoids];

        AdjustMargin();

        Parallel.For(0, numOfBoids, i =>
        {
            RandomPositions(i);
            RandomVelocites(i);
        });
    }
    void Update()
    {
        AdjustMargin();


        Parallel.For(0, numOfBoids, i =>
        {
            WallAlignment(i);
            if (seperation) Seperation(i);
            if (alignment) Alignment(i);
            if (cohesion) Cohesion(i);
            CheckSpeed(i);
            UpdatePositions(i, 1f / simSteps);
        });
    }

    void AdjustMargin()
    {
        rightMargin     =  margin.x * .45f;
        leftMargin      = -margin.x * .45f;
        topMargin       =  margin.y * .45f;
        bottomMargin    = -margin.y * .45f;
        frontMargin     =  margin.z * .45f;
        backMargin      = -margin.z * .45f;
    }
    void RandomPositions(long index)
    {
        boids[index].x = (float)rand.NextDouble() * margin.x - margin.x / 2;
        boids[index].y = (float)rand.NextDouble() * margin.y - margin.y / 2;
        boids[index].z = (float)rand.NextDouble() * margin.z - margin.z / 2;
    }
    void RandomVelocites(long index)
    {
        boids[index].vx = (float)rand.NextDouble() * maxSpeed * 2 - maxSpeed;
        boids[index].vy = (float)rand.NextDouble() * maxSpeed * 2 - maxSpeed;
        boids[index].vz = (float)rand.NextDouble() * maxSpeed * 2 - maxSpeed;
    }
    void UpdatePositions(long index, float deltaTime)
    {
        boids[index].x += boids[index].vx * deltaTime;
        boids[index].y += boids[index].vy * deltaTime;
        boids[index].z += boids[index].vz * deltaTime;
    }

    private void Seperation(long index)
    {
        float close_dx  = 0;
        float close_dy  = 0;
        float close_dz  = 0;
        Boid3D boidA    = boids[index];

        for (int i = 0; i < numOfBoids; i++)
        {
            if (index == i) continue;

            Boid3D boidB    = boids[i];
            float distance = Distance(new Vector3(boids[index].x, boids[index].y, boids[index].z), new Vector3(boids[i].x, boids[i].y, boids[i].z));

            if (distance < dangerRange)
            {
                close_dx += boidA.x - boidB.x;
                close_dy += boidA.y - boidB.y;
                close_dz += boidA.z - boidB.z;
            }
        }

        boids[index].vx += close_dx * avoidFactor;
        boids[index].vy += close_dy * avoidFactor;
        boids[index].vz += close_dz * avoidFactor;
    }

    private void Alignment(long index)
    {
        float xvel_avg          = 0;
        float yvel_avg          = 0;
        float zvel_avg          = 0;
        uint neighboring_boids  = 0;

        for (int i = 0; i < numOfBoids; i++)
        {
            if (i == index) continue;

            float distance = Distance(new Vector3(boids[index].x, boids[index].y, boids[index].z), new Vector3(boids[i].x, boids[i].y, boids[i].z));
            if ((distance > dangerRange) && (distance < sightRange))
            {
                xvel_avg += boids[i].vx;
                yvel_avg += boids[i].vy;
                zvel_avg += boids[i].vy;
                neighboring_boids++;
            }
        }

        if (neighboring_boids > 0)
        {
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;
            zvel_avg /= neighboring_boids;

            boids[index].vx += (xvel_avg - boids[index].vx) * matchingFactor;
            boids[index].vy += (yvel_avg - boids[index].vy) * matchingFactor;
            boids[index].vz += (zvel_avg - boids[index].vz) * matchingFactor;
        }
    }

    private void Cohesion(long index)
    {
        float xpos_avg          = 0;
        float ypos_avg          = 0;
        float zpos_avg          = 0;
        uint neighboring_boids  = 0;

        for (int i = 0; i < numOfBoids; i++)
        {
            if (i == index) continue;

            float distance = Distance(new Vector3(boids[index].x, boids[index].y, boids[index].z), new Vector3(boids[i].x, boids[i].y, boids[i].z));
            if ((distance > dangerRange) && (distance < sightRange))
            {
                xpos_avg += boids[i].x;
                ypos_avg += boids[i].y;
                zpos_avg += boids[i].z;
                neighboring_boids++;
            }
        }

        if (neighboring_boids > 0)
        {
            centerOfMass[index] = new Vector3(xpos_avg, ypos_avg, zpos_avg);

            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            zpos_avg /= neighboring_boids;

            boids[index].vx += (xpos_avg - boids[index].x) * centeringFactor;
            boids[index].vy += (ypos_avg - boids[index].y) * centeringFactor;
            boids[index].vz += (zpos_avg - boids[index].z) * centeringFactor;
        }
    }

    void CheckSpeed(long index)
    {
        Vector3 vel = new(boids[index].vx, boids[index].vy, boids[index].vz);

        float speed = MathF.Sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

        if (speed < minSpeed)
        {
            boids[index].vx = (vel.x / speed) * minSpeed;
            boids[index].vy = (vel.y / speed) * minSpeed;
            boids[index].vz = (vel.z / speed) * minSpeed;
        }
        else if (speed > maxSpeed)
        {
            boids[index].vx = (vel.x / speed) * maxSpeed;
            boids[index].vy = (vel.y / speed) * maxSpeed;
            boids[index].vz = (vel.z / speed) * maxSpeed;
        }
    }

    void WallAlignment(long index)
    {
        if (boids[index].x > leftMargin)
            boids[index].vx = boids[index].vx - turnFactor;

        if (boids[index].x < rightMargin)
            boids[index].vx = boids[index].vx + turnFactor;

        if (boids[index].y > bottomMargin)
            boids[index].vy = boids[index].vy - turnFactor;

        if (boids[index].y < topMargin)
            boids[index].vy = boids[index].vy + turnFactor;

        if (boids[index].z > backMargin)
            boids[index].vz = boids[index].vz - turnFactor;

        if (boids[index].y < frontMargin)
            boids[index].vz = boids[index].vz + turnFactor;
    }


    [ExecuteInEditMode]
    void OnDrawGizmos()
    {
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(Vector3.zero, margin);

        if (boids == null) return;

        for (int i = 0; i < numOfBoids; i++)
        {
            Vector3 position = new(boids[i].x, boids[i].y, boids[i].z);
            Gizmos.color = Color.grey;
            Gizmos.DrawSphere(position, boidRadius);

            if (visualSeperation)
            {
                Gizmos.color = colorSeperation;
                Gizmos.DrawWireSphere(position, dangerRange);
            }
            if (visualAlignment)
            {
                Gizmos.color = colorAlignment;
                Gizmos.DrawWireSphere(position, sightRange);
            }
            if (visualCohesion)
            {
                Gizmos.color = colorCohesion;
                Gizmos.DrawLine(position, centerOfMass[i]);
            }
        }
    }
    static float Distance(Vector3 pointA, Vector3 pointB)
    {
        float distance = MathF.Sqrt(Vector3.Dot(pointB, pointA));
        return distance;
    }
}

public struct Boid3D
{
    public float x;
    public float y;
    public float z;
    public float vx;
    public float vy;
    public float vz;
}