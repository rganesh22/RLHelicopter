using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

public class HelicopterAgent : Agent
{
    public GameObject jet;
    public bool useVecObs;
    Vector3 goalPosition;
    Rigidbody fighterjetRB;

    EnvironmentParameters m_ResetParams;

    Vector3 initPos;
    Quaternion initRot;
    Vector3 initGoalPosition;

    private float episode_start_time;

    public override void Initialize()
    {
        fighterjetRB = GetComponent<Rigidbody>();
        goalPosition = GameObject.Find("Goal").transform.position;

        initGoalPosition = goalPosition;
      
        initPos = gameObject.transform.position;
        initRot = gameObject.transform.rotation;
        
        SetResetParameters();

        Time.timeScale = 5f;
        // Time.timeScale = 100f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (useVecObs)
        {
            sensor.AddObservation(transform.position.x);
            sensor.AddObservation(transform.position.y);
            sensor.AddObservation(transform.position.z);

            sensor.AddObservation(transform.rotation.x);
            sensor.AddObservation(transform.rotation.y);
            sensor.AddObservation(transform.rotation.z);

            sensor.AddObservation(GetComponent<HelicopterScript>().throttleTarget);
            sensor.AddObservation(GetComponent<HelicopterScript>().Pitch);
            sensor.AddObservation(GetComponent<HelicopterScript>().Roll);
            sensor.AddObservation(GetComponent<HelicopterScript>().Yaw);

            sensor.AddObservation((goalPosition - transform.position).x);
            sensor.AddObservation((goalPosition - transform.position).y);
            sensor.AddObservation((goalPosition - transform.position).z);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float throttleTarget = Mathf.Clamp(actionBuffers.ContinuousActions[0], 0f, 1f);
        float stickInputX = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
        float stickInputY = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
        float stickInputZ = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);

        jet.GetComponent<HelicopterScript>().Pitch = stickInputX;
        jet.GetComponent<HelicopterScript>().Roll =  stickInputY;
        jet.GetComponent<HelicopterScript>().Yaw =  stickInputZ;
        jet.GetComponent<HelicopterScript>().throttleTarget = throttleTarget;
        // jet.GetComponent<HelicopterScript>().throttleTarget = 1f;

        float distToGoal = Vector3.Distance(transform.position, goalPosition);
        
        Debug.Log(transform.position.y);

        // // [(CL STEP 1) JUST FLY!]
        // if (transform.position.y < 10){
        //     Debug.Log("Hit the Ground :(");
        //     SetReward(-1f);
        //     EndEpisode();
        // } else {
        //     // speed + not hitting ground
        //     float reward = 0;
        //     reward += 0.0001f;
        //     SetReward(reward);
        //     float cumReward = GetCumulativeReward();
        //     if (cumReward >= 1f) {
        //         EndEpisode();
        //     }
        // }

        // [(CL STEP 2) HIT TARGET!]
        // Vector3(-1872.30005,72.3000031,-2044.40002)
        if (transform.position.y < 10){
            Debug.Log("Hit the Ground :(");
            SetReward(-1f);
            EndEpisode();
        } else if (distToGoal < 50) {
            Debug.Log("Reached Goal!");
            SetReward(1f);
            EndEpisode();
        } else {
            // speed + not hitting ground
            float reward = 0;
            reward -= 0.001f * distToGoal;
            reward += 0.001f;
            float cumReward = GetCumulativeReward();
            if (cumReward <= -1f) {
                EndEpisode();
            }
            SetReward(reward);
        }
        
    }    

    public override void OnEpisodeBegin()
    {
        //Reset the parameters when the Agent is reset.
        SetResetParameters();        
    }

    public void SetResetParameters()
    {
        episode_start_time = Time.realtimeSinceStartup;
        fighterjetRB.velocity = new Vector3(0, 0, 0);
        transform.position = initPos;
        transform.rotation = initRot;

        // [(CL STEP 3) HIT RANDOM TARGET!]
        float newX = Random.Range(-80.0f, 80.0f) + initGoalPosition.x;
        float newY = Mathf.Clamp(Random.Range(-80.0f, 80.0f) + initGoalPosition.y, 40f, float.MaxValue);
        float newZ = Random.Range(-80.0f, 80.0f) + initGoalPosition.z;
        GameObject.Find("Goal").transform.position = new Vector3(newX, newY, newZ);
    }
}
