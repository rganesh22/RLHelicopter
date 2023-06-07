using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HelicopterScript : MonoBehaviour
{
    private Rigidbody _rigidbody;

    public float _responsiveness = 75f;
    public float _maxThrust = 300f;

    [Tooltip("How quickly the jet can accelerate and decelerate.")]
    public float acceleration = 10.0f;
    [Tooltip("How quickly the jet will brake when the throttle goes below neutral.")]
    public float brakeDrag = 5f;

    public float throttleTarget = 0f;

    public const float ThrottleNeutral = 0.33f;
    public const float ThrottleMin = 0.1f;
    public const float ThrottleMax = 1f;
    public const float ThrottleSpeed = 5f;

    public float Roll;
    public float Pitch;
    public float Yaw;

    private int throttlePressed = 0;
    
    [SerializeField] private float _rotorSpeedModifier = 10f;
    [SerializeField] private Transform _rotorsTransform;

    private const float FORCE_MULT = 10.0f;

    private float throttleTrue = 0.33f;

    void Awake(){
        _rigidbody = GetComponent<Rigidbody>();
    }

    // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void Update()
    {
        // HandleInputs();
        // Debug.Log(Vector3.up * (_maxThrust * throttleTrue) * _rotorSpeedModifier);
        _rotorsTransform.Rotate(Vector3.up * Mathf.Clamp((_maxThrust * throttleTrue) * _rotorSpeedModifier, 0f, 300f));

        Debug.Log(transform.rotation);
    }

    void FixedUpdate()
    {

        float brakePower = brakeDrag * Mathf.InverseLerp(ThrottleNeutral, ThrottleMin, throttlePressed);
        float brakeAccel = brakeDrag * brakePower;

        // Throttle has to move slowly so that the plane still accelerates slowly using high
        // drag physics. Without them, the plane would change speed almost instantly.        
        throttleTrue = Mathf.MoveTowards(throttleTrue, throttleTarget, ((acceleration + brakeAccel) / FORCE_MULT) * Time.deltaTime);

        // Debug.Log(throttleTrue);
        // Debug.Log(transform.up * _maxThrust * throttleTrue * FORCE_MULT);

        _rigidbody.AddRelativeForce(transform.up * _maxThrust * throttleTrue * FORCE_MULT, ForceMode.Force);
        // _rigidbody.AddForce(transform.up * Throttle * throttlePressed, ForceMode.Impulse);

        _rigidbody.AddTorque(transform.right * Pitch * _responsiveness);
        _rigidbody.AddTorque(-transform.forward * Roll * _responsiveness);
        _rigidbody.AddTorque(transform.up * Yaw * _responsiveness);
    }

    void HandleInputs()
    {
        Roll = Input.GetAxis("Roll");
        Pitch = Input.GetAxis("Pitch");
        Yaw = Input.GetAxis("Yaw");

        throttlePressed = 0;
        if (Input.GetKey(KeyCode.Space)) {
            // Throttle += Time.deltaTime * _throttleAmt;
            throttlePressed = 1;
        } else if (Input.GetKey(KeyCode.LeftShift)) {
            // Throttle -= Time.deltaTime * _throttleAmt;
        }

        throttleTarget = throttlePressed;
    }
}
