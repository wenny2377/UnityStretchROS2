using UnityEngine;
using WebSocketSharp;
using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using PimDeWitte.UnityMainThreadDispatcher;

public class StretchJointRosSubscriber : MonoBehaviour
{
    [Header("ROS setting")]
    [SerializeField] private string rosBridgeUrl = "ws://localhost:9090";
    [SerializeField] private string jointStateTopic = "/joint_states";
    [SerializeField] private string odomTopic = "/odom";

    [Header("Base Link Articulation")]
    public ArticulationBody baseLinkBody;

    [Header("Joints Articulation mapping")]
    public ArticulationBody joint_right_wheel;
    public ArticulationBody joint_left_wheel;
    public ArticulationBody joint_lift;
    public ArticulationBody joint_arm_l3;
    public ArticulationBody joint_arm_l2;
    public ArticulationBody joint_arm_l1;
    public ArticulationBody joint_arm_l0;
    public ArticulationBody joint_wrist_yaw;
    public ArticulationBody joint_wrist_pitch;
    public ArticulationBody joint_wrist_roll;
    public ArticulationBody joint_head_pan;
    public ArticulationBody joint_head_tilt;
    public ArticulationBody joint_gripper_finger_left;
    public ArticulationBody joint_gripper_finger_right;

    private WebSocket rosWebSocket;
    private Dictionary<string, JointMapping> jointMap = new Dictionary<string, JointMapping>();
    private Dictionary<string, float> jointTargetValues = new Dictionary<string, float>();
    private readonly object lockObj = new object();

    [Serializable]
    public enum JointType { Rotational, Prismatic }

    public class JointMapping
    {
        public string jointName;
        public ArticulationBody articulationBody;
        public JointType jointType;
        public Vector3 axis;
    }

    void Start()
    {
        var jointMappings = new List<JointMapping>
        {
            new JointMapping { jointName = "joint_right_wheel", articulationBody = joint_right_wheel, jointType = JointType.Rotational, axis = Vector3.up },
            new JointMapping { jointName = "joint_left_wheel", articulationBody = joint_left_wheel, jointType = JointType.Rotational, axis = Vector3.up },
            new JointMapping { jointName = "joint_lift", articulationBody = joint_lift, jointType = JointType.Prismatic, axis = Vector3.up },
            new JointMapping { jointName = "joint_arm_l3", articulationBody = joint_arm_l3, jointType = JointType.Prismatic, axis = Vector3.up },
            new JointMapping { jointName = "joint_arm_l2", articulationBody = joint_arm_l2, jointType = JointType.Prismatic, axis = Vector3.up },
            new JointMapping { jointName = "joint_arm_l1", articulationBody = joint_arm_l1, jointType = JointType.Prismatic, axis = Vector3.up },
            new JointMapping { jointName = "joint_arm_l0", articulationBody = joint_arm_l0, jointType = JointType.Prismatic, axis = Vector3.up },
            new JointMapping { jointName = "joint_wrist_yaw", articulationBody = joint_wrist_yaw, jointType = JointType.Rotational, axis = Vector3.forward },
            new JointMapping { jointName = "joint_wrist_pitch", articulationBody = joint_wrist_pitch, jointType = JointType.Rotational, axis = Vector3.forward },
            new JointMapping { jointName = "joint_wrist_roll", articulationBody = joint_wrist_roll, jointType = JointType.Rotational, axis = Vector3.forward },
            new JointMapping { jointName = "joint_head_pan", articulationBody = joint_head_pan, jointType = JointType.Rotational, axis = Vector3.forward },
            new JointMapping { jointName = "joint_head_tilt", articulationBody = joint_head_tilt, jointType = JointType.Rotational, axis = Vector3.forward },
            new JointMapping { jointName = "joint_gripper_finger_left", articulationBody = joint_gripper_finger_left, jointType = JointType.Rotational, axis = -Vector3.right },
            new JointMapping { jointName = "joint_gripper_finger_right", articulationBody = joint_gripper_finger_right, jointType = JointType.Rotational, axis = Vector3.right},
        };

        foreach (var joint in jointMappings)
        {
            if (joint.articulationBody != null)
            {
                jointMap[joint.jointName] = joint;
                jointTargetValues[joint.jointName] = 0f;
            }
        }

        rosWebSocket = new WebSocket(rosBridgeUrl);
        rosWebSocket.OnMessage += (sender, e) =>
        {
            if (e.Data.Contains(jointStateTopic))
                ProcessJointMessage(e.Data);
            else if (e.Data.Contains(odomTopic))
                ProcessOdomMessage(e.Data);
        };
        rosWebSocket.Connect();

        SubscribeToRosTopic(jointStateTopic, "sensor_msgs/JointState");
        SubscribeToRosTopic(odomTopic, "nav_msgs/Odometry");
    }

    void SubscribeToRosTopic(string topic, string type)
    {
        string msg = $@"{{""op"":""subscribe"",""topic"":""{topic}"",""type"":""{type}""}}";
        rosWebSocket.Send(msg);
        Debug.Log($"Subscribed to: {topic}");
    }

    void ProcessJointMessage(string message)
    {
        try
        {
            JObject rosMsg = JObject.Parse(message);
            var msgData = rosMsg["msg"];
            var names = msgData["name"] as JArray;
            var positions = msgData["position"] as JArray;
            if (names == null || positions == null) return;

            lock (lockObj)
            {
                for (int i = 0; i < names.Count; i++)
                {
                    string jointName = names[i].ToString();
                    float value = positions[i].ToObject<float>();
                    if (jointMap.ContainsKey(jointName))
                        jointTargetValues[jointName] = value;
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"JointState parsing error: {ex.Message}");
        }
    }

    void ProcessOdomMessage(string message)
    {
        try
        {
            JObject rosMsg = JObject.Parse(message);
            var msg = rosMsg["msg"];
            var pos = msg["pose"]["pose"]["position"];
            var ori = msg["pose"]["pose"]["orientation"];

            // ROS -> Unity coordinate conversion
            Vector3 rosPosition = new Vector3(
                pos["x"].ToObject<float>(),
                pos["y"].ToObject<float>(),
                pos["z"].ToObject<float>()
            );

            Quaternion rosOrientation = new Quaternion(
                ori["x"].ToObject<float>(),
                ori["y"].ToObject<float>(),
                ori["z"].ToObject<float>(),
                ori["w"].ToObject<float>()
            );

            float heightOffset = 0.71f; 

            Vector3 unityPosition = new Vector3(rosPosition.y, rosPosition.z + heightOffset, rosPosition.x);

            Quaternion unityRotation = new Quaternion(-rosOrientation.y, rosOrientation.z, -rosOrientation.x, rosOrientation.w);

            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                if (baseLinkBody != null)
                {
                    baseLinkBody.TeleportRoot(unityPosition, unityRotation);
                    baseLinkBody.linearVelocity = Vector3.zero;
                    baseLinkBody.angularVelocity = Vector3.zero;
                }
            });
        }
        catch (Exception ex)
        {
            Debug.LogError($"Odometry parsing error: {ex.Message}");
        }
    }

    void Update()
    {
        lock (lockObj)
        {
            foreach (var kv in jointTargetValues)
            {
                if (jointMap.TryGetValue(kv.Key, out var joint))
                    ApplyJointTransform(joint, kv.Value);
            }
        }
    }

    void ApplyJointTransform(JointMapping joint, float value)
    {
        if (joint.articulationBody == null) return;

        var drive = joint.articulationBody.xDrive;
        drive.stiffness = 10000f;      
        drive.damping = 200f;          
        drive.forceLimit = 100000f;      

        switch (joint.jointType)
        {
            case JointType.Rotational:
                drive.target = value * Mathf.Rad2Deg;
                break;
            case JointType.Prismatic:
                drive.target = value;
                break;
        }

        
        if (joint.jointName.Contains("gripper"))
        {
            Debug.Log($"Gripper target value: {value}");
        }


        joint.articulationBody.xDrive = drive;
    }

    void OnApplicationQuit()
    {
        rosWebSocket?.Close();
    }
}
