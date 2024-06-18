using System;
using System.Security.Cryptography;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;

public class BoidSimulation2D : MonoBehaviour
{
    [Header("Enviornment")]
    [SerializeField] Vector2 margin = new(95, 55);
    [SerializeField] float boidRadius = .1f;
    [SerializeField] uint numOfBoids = 200;

    [Header("Factors")]
    [SerializeField] float avoidFactor = .03f;
    [SerializeField] float matchingFactor = .001f;
    [SerializeField] float centeringFactor = .003f;

    [Header("Other Boid Variables")]
    [SerializeField] float turnFactor = .15f;
    [SerializeField] uint maxSpeed = 8;
    [SerializeField] uint minSpeed = 5;
    [SerializeField] uint sightRange = 10;
    [SerializeField] uint dangerRange = 1;

    [Header("Rules")]
    [SerializeField] bool seperation = true;
    [SerializeField] bool alignment = true;
    [SerializeField] bool cohesion = true;

    [Header("Visuals")]
    [SerializeField] bool visualSeperation = false;
    [SerializeField] bool visualAlignment = false;
    [SerializeField] bool visualCohesion = false;
    [SerializeField] Color colorSeperation = new(1, .92f, .016f, .1f);
    [SerializeField] Color colorAlignment = new(1, 0, 0, .1f);
    [SerializeField] Color colorCohesion = new(0, 1, 0, .1f);

    Boid2D[] boids;
    Vector2[] centerOfMass;
    System.Random rand;

    float rightMargin;
    float leftMargin;
    float topMargin;
    float bottomMargin;


    void Start()
    {
        rand = new System.Random();
        boids = new Boid2D[numOfBoids];
        centerOfMass = new Vector2[numOfBoids];

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

        float deltaTime = Time.deltaTime;
        Parallel.For(0, numOfBoids, i =>
        {
            WallAlignment(i);
            if (seperation) Seperation(i);
            if (alignment)  Alignment(i);
            if (cohesion)   Cohesion(i);
            CheckSpeed(i);
            UpdatePositions(i, deltaTime);
        });
    }

    void AdjustMargin()
    {
        rightMargin     =  margin.x * .45f;
        leftMargin      = -margin.x * .45f;
        topMargin       =  margin.y * .45f;
        bottomMargin    = -margin.y * .45f;
    }
    void RandomPositions(long index)
    {
        boids[index].x = (float)rand.NextDouble() * margin.x - margin.x / 2;
        boids[index].y = (float)rand.NextDouble() * margin.y - margin.y / 2;
    }
    void RandomVelocites(long index)
    {
        boids[index].vx = (float)rand.NextDouble() * maxSpeed * 2 - maxSpeed;
        boids[index].vy = (float)rand.NextDouble() * maxSpeed * 2 - maxSpeed;
    }
    void UpdatePositions(long index, float deltaTime)
    {
        boids[index].x += boids[index].vx * deltaTime;
        boids[index].y += boids[index].vy * deltaTime;
    }

    void Seperation(long index)
    {
        float close_dx = 0;
        float close_dy = 0;
        Boid2D boidA = boids[index];

        for (int i = 0; i < numOfBoids; i++)
        {
            if (index == i) continue;

            Boid2D boidB = boids[i];
            Vector2 posA = new(boidA.x, boidA.y);
            Vector2 posB = new(boidB.x, boidB.y);

            float distance = Distance(posA.x, posA.y, posB.x, posB.y);
            if (distance < dangerRange)
            {
                close_dx += boidA.x - boidB.x;
                close_dy += boidA.y - boidB.y;
            }
        }

        boids[index].vx += close_dx * avoidFactor;
        boids[index].vy += close_dy * avoidFactor;
    }

    void Alignment(long index)
    {
        float xvel_avg          = 0;
        float yvel_avg          = 0;
        uint neighboring_boids  = 0;
        Boid2D boidA = boids[index];

        for (int i = 0; i < numOfBoids; i++)
        {
            if (i == index) continue;

            Boid2D boidB = boids[i];
            Vector2 posA = new(boidA.x, boidA.y);
            Vector2 posB = new(boidB.x, boidB.y);

            float distance = Distance(posA.x, posA.y, posB.x, posB.y);
            if ((distance > dangerRange) && (distance < sightRange))
            {
                xvel_avg += boids[i].vx;
                yvel_avg += boids[i].vy;
                neighboring_boids++;
            }
        }

        if (neighboring_boids > 0)
        {
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;
            boids[index].vx += (xvel_avg - boids[index].vx) * matchingFactor;
            boids[index].vy += (yvel_avg - boids[index].vy) * matchingFactor;
        }
    }

    void Cohesion(long index)
    {
        float xpos_avg = 0;
        float ypos_avg = 0;
        uint neighboring_boids = 0;
        Boid2D boidA = boids[index];

        for (int i = 0; i < numOfBoids; i++)
        {
            if (i == index) continue;

            Boid2D boidB = boids[i];
            Vector2 posA = new(boidA.x, boidA.y);
            Vector2 posB = new(boidB.x, boidB.y);

            float distance = Distance(posA.x, posA.y, posB.x, posB.y);
            if ((distance > dangerRange) && (distance < sightRange))
            {
                xpos_avg += boids[i].x;
                ypos_avg += boids[i].y;
                neighboring_boids++;
            }
        }

        if (neighboring_boids > 0)
        {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            centerOfMass[index] = new Vector2(xpos_avg, ypos_avg);

            boids[index].vx += (xpos_avg - boids[index].x) * centeringFactor;
            boids[index].vy += (ypos_avg - boids[index].y) * centeringFactor;
        }
    }

    void CheckSpeed(long index)
    {
        Vector2 vel = new(boids[index].vx, boids[index].vy);

        float speed = MathF.Sqrt(vel.x * vel.x + vel.y * vel.y);

        if (speed < minSpeed)
        {
            boids[index].vx = (vel.x / speed) * minSpeed;
            boids[index].vy = (vel.y / speed) * minSpeed;
        }
        else if (speed > maxSpeed)
        {
            boids[index].vx = (vel.x / speed) * maxSpeed;
            boids[index].vy = (vel.y / speed) * maxSpeed;
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
    }




    [ExecuteInEditMode]
    void OnDrawGizmos()
    {
        if (boids == null) return;

        for (int i = 0; i < numOfBoids; i++)
        {
            Vector2 position = new(boids[i].x, boids[i].y);
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
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(Vector3.zero, margin);
    }
    public static float Distance(float x1, float y1, float x2, float y2)
    {
        float distance = MathF.Sqrt(Mathf.Pow(x2 - x1, 2) + Mathf.Pow(y2 - y1, 2));
        return distance;
    }
}

public struct Boid2D
{
    public float x;
    public float y;
    public float vx;
    public float vy;
}