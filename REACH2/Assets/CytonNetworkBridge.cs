using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SocketIOClient;
using System;
using System.Text.Json; // <--- Switching to the native library serializer

[Serializable]
public class JointMap
{
	public string name = "Joint";
	public int servoId;
	public ArticulationBody body;
	[Tooltip("Offset in degrees to match Unity 0 with Servo 2048")]
	public float angleOffset = 0;
	public bool invert = false;
}

[Serializable]
public class GripperMap
{
	public int servoId;
	public ArticulationBody leftFinger;
	public ArticulationBody rightFinger;
	public float openServoVal = 2048;
	public float closedServoVal = 3000;
	public float openUnityPos = 0.04f;
	public float closedUnityPos = 0.0f;
}

public class CytonNetworkBridge : MonoBehaviour
{
	[Header("Network Config")]
	public string serverUrl = "http://localhost:5000";

	[Header("Arm Setup")]
	public List<JointMap> armJoints = new List<JointMap>();

	[Header("Gripper Setup")]
	public GripperMap gripper;

	private SocketIOUnity socket;
	private bool isConnected = false;
	private bool iHaveControl = false;
	private string mySessionId = "";

	// Thread-safe data storage
	private Dictionary<int, int> incomingPositions = new Dictionary<int, int>();
	private bool newDataAvailable = false;

	void Start()
	{
		SetupSocket();
	}

	void SetupSocket()
	{
		var uri = new Uri(serverUrl);
		socket = new SocketIOUnity(uri, new SocketIOOptions
		{
			Transport = SocketIOClient.Transport.TransportProtocol.WebSocket
		});

		socket.OnConnected += (sender, e) => {
			Debug.Log("Connected to Robot Server");
			isConnected = true;
		};

		// --- FIX 1: Handling "my_id" with System.Text.Json ---
		socket.OnUnityThread("my_id", (response) => {
			// 1. Get the first argument as a generic JsonElement
			var root = response.GetValue<JsonElement>(0);

			// 2. Safe property access
			if (root.TryGetProperty("sid", out JsonElement sidEl))
			{
				mySessionId = sidEl.GetString();
				Debug.Log($"My Session ID: {mySessionId}");
			}
		});

		// --- FIX 2: Handling "robot_state" with System.Text.Json ---
		socket.OnUnityThread("robot_state", (response) => {
			// 1. Get the JSON object
			var root = response.GetValue<JsonElement>(0);

			// 2. Parse Controller ID
			string controllerId = null;
			if (root.TryGetProperty("controller_id", out JsonElement ctrlIdEl) &&
				ctrlIdEl.ValueKind == JsonValueKind.String)
			{
				controllerId = ctrlIdEl.GetString();
			}

			iHaveControl = (controllerId == mySessionId);

			// 3. Parse Positions
			if (root.TryGetProperty("positions", out JsonElement posEl))
			{
				lock (incomingPositions)
				{
					// Loop over the JSON object properties (keys are IDs, values are positions)
					foreach (JsonProperty prop in posEl.EnumerateObject())
					{
						if (int.TryParse(prop.Name, out int id))
						{
							int val = prop.Value.GetInt32();
							if (incomingPositions.ContainsKey(id))
								incomingPositions[id] = val;
							else
								incomingPositions.Add(id, val);
						}
					}
					newDataAvailable = true;
				}
			}
		});

		socket.Connect();
		StartCoroutine(SendCommandsRoutine());
	}

	void Update()
	{
		// === VISUALIZATION (SERVER -> UNITY) ===
		if (!iHaveControl && newDataAvailable)
		{
			lock (incomingPositions)
			{
				// Update Arm
				foreach (var joint in armJoints)
				{
					if (incomingPositions.ContainsKey(joint.servoId))
					{
						int rawVal = incomingPositions[joint.servoId];
						float degrees = ServoToUnityAngle(rawVal, joint);
						SetArticulationTarget(joint.body, degrees);
					}
				}

				// Update Gripper
				if (gripper.leftFinger != null && incomingPositions.ContainsKey(gripper.servoId))
				{
					int rawVal = incomingPositions[gripper.servoId];
					float linearPos = ServoToGripperLinear(rawVal);
					SetArticulationTarget(gripper.leftFinger, linearPos);
					if (gripper.rightFinger != null)
						SetArticulationTarget(gripper.rightFinger, linearPos);
				}
				newDataAvailable = false;
			}
		}
	}

	// === DRIVING (UNITY -> SERVER) ===
	IEnumerator SendCommandsRoutine()
	{
		while (true)
		{
			if (isConnected && iHaveControl)
			{
				// Send Arm Data
				foreach (var joint in armJoints)
				{
					float currentAngle = GetArticulationPosition(joint.body);
					int servoValue = UnityAngleToServo(currentAngle, joint);

					// Simple anonymous object works fine here
					socket.Emit("command_move", new { id = joint.servoId, val = servoValue });
				}

				// Send Gripper Data
				if (gripper.leftFinger != null)
				{
					float currentLinear = GetArticulationPosition(gripper.leftFinger);
					int servoValue = GripperLinearToServo(currentLinear);
					socket.Emit("command_move", new { id = gripper.servoId, val = servoValue });
				}
			}
			yield return new WaitForSeconds(0.1f);
		}
	}

	// --- HELPER MATH (Unchanged) ---
	float ServoToUnityAngle(int servoVal, JointMap config)
	{
		float centered = servoVal - 2048;
		float degrees = centered * 0.088f;
		if (config.invert) degrees *= -1;
		degrees -= config.angleOffset;
		return degrees;
	}

	int UnityAngleToServo(float unityDeg, JointMap config)
	{
		float degrees = unityDeg + config.angleOffset;
		if (config.invert) degrees *= -1;
		float units = degrees / 0.088f;
		return Mathf.Clamp(Mathf.RoundToInt(units + 2048), 0, 4095);
	}

	float ServoToGripperLinear(int servoVal)
	{
		float t = Mathf.InverseLerp(gripper.openServoVal, gripper.closedServoVal, servoVal);
		return Mathf.Lerp(gripper.openUnityPos, gripper.closedUnityPos, t);
	}

	int GripperLinearToServo(float linearPos)
	{
		float t = Mathf.InverseLerp(gripper.openUnityPos, gripper.closedUnityPos, linearPos);
		return Mathf.RoundToInt(Mathf.Lerp(gripper.openServoVal, gripper.closedServoVal, t));
	}

	void SetArticulationTarget(ArticulationBody body, float target)
	{
		var drive = body.xDrive;
		drive.target = target;
		body.xDrive = drive;
	}

	float GetArticulationPosition(ArticulationBody body)
	{
		if (body.jointPosition.dofCount > 0)
		{
			if (body.jointType == ArticulationJointType.RevoluteJoint)
				return body.jointPosition[0] * Mathf.Rad2Deg;
			return body.jointPosition[0];
		}
		return 0f;
	}

	public void RequestControl()
	{
		socket.Emit("request_control");
	}

	void OnGUI()
	{
		GUILayout.BeginArea(new Rect(10, 10, 200, 150));
		GUILayout.Label($"Conn: {(isConnected ? "OK" : "...")}");
		if (iHaveControl)
		{
			GUI.color = Color.green; GUILayout.Label("CONTROL: ACTIVE");
		}
		else
		{
			GUI.color = Color.white; GUILayout.Label("CONTROL: PASSIVE");
			if (GUILayout.Button("Take Control")) RequestControl();
		}
		GUILayout.EndArea();
	}

	void OnDestroy()
	{
		if (socket != null) socket.Disconnect();
	}
}