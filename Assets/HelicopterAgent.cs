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

    private float episode_start_time;

    public override void Initialize()
    {
        fighterjetRB = GetComponent<Rigidbody>();
        goalPosition = GameObject.Find("Goal").transform.position;
      
        initPos = gameObject.transform.position;
        initRot = gameObject.transform.rotation;
        
        SetResetParameters();
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

            sensor.AddObservation(fighterjetRB.velocity.x);
            sensor.AddObservation(fighterjetRB.velocity.y);
            sensor.AddObservation(fighterjetRB.velocity.z);

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
        // jet.GetComponent<HelicopterScript>().throttleTarget = throttleTarget;
        jet.GetComponent<HelicopterScript>().throttleTarget = 1f;

        float distToGoal = Vector3.Distance(transform.position, goalPosition);
        
        if ((transform.position.y < 2) || ((Time.realtimeSinceStartup - episode_start_time) > 20f)){
            Debug.Log("Hit the Ground :(");
            SetReward(-500000f);
            EndEpisode();
        } else if (distToGoal < 3f) {
            SetReward(500000f);
            EndEpisode();
        } else {
            AddReward(10f * Mathf.Pow(transform.position.y, 2));
        }
        
        if (Mathf.Abs(transform.rotation.x) > 0.5 || Mathf.Abs(transform.rotation.y) > 0.5 || Mathf.Abs(transform.rotation.z) > 0.5) {
            Debug.Log("Too Much Rotation");
            SetReward(-500000f);
            EndEpisode();            
        }

        // Debug.Log(Mathf.Abs(transform.rotation.x));
        // Debug.Log(Mathf.Abs(transform.rotation.y));
        // Debug.Log(Mathf.Abs(transform.rotation.z));
        
        // else if (distToGoal < 2) {
        //     Debug.Log("Reached Goal!");
        //     SetReward(500000f);
        //     EndEpisode();
        // } else {
        //     float reward = -0.1f * distToGoal * distToGoal;
        //     reward += (1000 * transform.position.y);
        //     reward += (10 * fighterjetRB.velocity.magnitude);
        //     // if (transform.rotation.y > 150) {
        //     //    SetReward(-4000f); 
        //     // }
        //     // float reward = 0f;
        //     SetReward(reward);
        // }
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
    }
}
