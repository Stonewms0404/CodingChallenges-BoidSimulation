using System;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;

public class BoidSimulation2DCompute : MonoBehaviour
{
    [Header("Enviornment")]
    [SerializeField] Vector2 margin = new(95, 55);
    [SerializeField] float boidRadius = .1f;
    [SerializeField] uint numOfBoids = 200;
    [SerializeField] uint simSteps = 120;

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

    [Header("Visuals")]
    [SerializeField] bool visualSeperation = false;
    [SerializeField] bool visualAlignment = false;
    [SerializeField] bool visualCohesion = false;
    [SerializeField] Color colorSeperation = new(1, .92f, .016f, .1f);
    [SerializeField] Color colorAlignment = new(1, 0, 0, .1f);
    [SerializeField] Color colorCohesion = new(0, 1, 0, .1f);

    [Header("Compute Shader")]
    [SerializeField] ComputeShader boidShader;

    ComputeBuffer boidBuffer;
    ComputeBuffer distancesBuffer;
    ComputeBuffer centerOfMassBuffer;

    Boid2D[] boids;
    float[] distances;
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
        distances = new float[numOfBoids];
        centerOfMass = new Vector2[numOfBoids];

        boidBuffer = new((int)numOfBoids, sizeof(float) * 4);
        centerOfMassBuffer = new((int)numOfBoids, sizeof(float) * 2);
        distancesBuffer = new((int)numOfBoids, sizeof(float));

        AdjustMargin();


        Parallel.For(0, numOfBoids, i =>
        {
            RandomPositions(i);
            RandomVelocites(i);
        });
    }

    void Update()
    {
        SetValues();
        DispatchShaders();
    }

    private void DispatchShaders()
    {
        int boidKernel = boidShader.FindKernel("CalculateBoid");

        Compute.Dispatch(boidShader, boidKernel, (int)numOfBoids);

        boidBuffer.GetData(boids);
        centerOfMassBuffer.GetData(centerOfMass);
        distancesBuffer.GetData(distances);
    }

    void SetValues()
    {
        int boidKernel = boidShader.FindKernel("CalculateBoid");
        AdjustMargin();

        distancesBuffer.SetData(distances);

        boidShader.SetInt("numOfBoids", (int)numOfBoids);
        boidShader.SetFloat("simSteps", simSteps);

        boidShader.SetFloat("avoidFactor", avoidFactor);
        boidShader.SetFloat("matchingFactor", matchingFactor);
        boidShader.SetFloat("centeringFactor", centeringFactor);
        boidShader.SetFloat("turnFactor", turnFactor);

        boidShader.SetInt("maxSpeed", (int)maxSpeed);
        boidShader.SetInt("minSpeed", (int)minSpeed);
        
        boidShader.SetInt("sightRange", (int)sightRange);
        boidShader.SetInt("dangerRange", (int)dangerRange);
        
        boidShader.SetFloat("rightMargin", rightMargin);
        boidShader.SetFloat("leftMargin", leftMargin);
        boidShader.SetFloat("topMargin", topMargin);
        boidShader.SetFloat("bottomMargin", bottomMargin);

        boidBuffer.SetData(boids);
        centerOfMassBuffer.SetData(centerOfMass);
        distancesBuffer.SetData(distances);
        boidShader.SetBuffer(boidKernel, "boids", boidBuffer);
        boidShader.SetBuffer(boidKernel, "centerOfMass", centerOfMassBuffer);
        boidShader.SetBuffer(boidKernel, "distances", distancesBuffer);
    }

    void AdjustMargin()
    {
        rightMargin = margin.x * .45f;
        leftMargin = -margin.x * .45f;
        topMargin = margin.y * .45f;
        bottomMargin = -margin.y * .45f;
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

    [ExecuteInEditMode]
    void OnDrawGizmos()
    {
        if (boids == null) return;

        for (int i = 0; i < numOfBoids; i++)
        {
            Vector2 position = new(boids[i].x, boids[i].y);
            Gizmos.color = i == numOfBoids - 1 ? Color.cyan : Color.grey;
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

    private void OnDestroy()
    {
        centerOfMassBuffer.Release();
        centerOfMassBuffer.Dispose();
        boidBuffer.Release();
        boidBuffer.Dispose();
        distancesBuffer.Release();
        distancesBuffer.Dispose();
    }
}