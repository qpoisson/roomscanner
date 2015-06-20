/*
/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
using System.Collections;
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Diagnostics;
using UnityEngine;
using Tango;

/// <summary>
/// Point cloud visualize using depth frame API.
/// </summary>
public class Pointcloud : MonoBehaviour, ITangoDepth
{
	[HideInInspector]
	public float m_overallZ = 0.0f;
	[HideInInspector]
	public int m_pointsCount = 0;
	[HideInInspector]
	public float m_depthDeltaTime = 0.0f;
	
	private TangoApplication m_tangoApplication;
	
	// Matrices for transforming pointcloud to world coordinates.
	// This equation will take account of the camera sensors extrinsic.
	// Full equation is:
	//   Matrix4x4 uwTuc = m_uwTss * m_ssTd * Matrix4x4.Inverse(m_imuTd) * m_imuTc * m_cTuc;
	private Matrix4x4 m_uwTss = new Matrix4x4 ();
	private Matrix4x4 m_ssTd = new Matrix4x4 ();
	private Matrix4x4 m_imuTd = new Matrix4x4 ();
	private Matrix4x4 m_imuTc = new Matrix4x4();
	private Matrix4x4 m_cTuc = new Matrix4x4 ();
	
	// Some const value.
	private const int VERT_COUNT = 61440;
	
	// m_vertices will be assigned to this mesh.
	private Mesh m_mesh;
	private int[] m_triangles;
	private Color[] m_colors;
	
	// Logging data.
	private double m_previousDepthDeltaTime = 0.0;
	private bool m_isExtrinsicQuerable = false;
	
	//List<GameObject> m_pointCloud;
	GameObject m_subCloud;

	// Control scanning
	public const int SCAN_STATE_OFF = 0;
	public const int SCAN_STATE_ON = 1;
	public const int SCAN_STATE_SCREENSHOT = 2;
	public const int SCAN_STATE_STREAM = 3;
	public int m_scan_state = SCAN_STATE_OFF;
	private bool m_initiatal_delete = false;

	string message = "hi";
	
	/// <summary>
	/// Use this for initialization.
	/// </summary>
	public void Start() 
	{
		m_tangoApplication = FindObjectOfType<TangoApplication>();
		m_tangoApplication.Register(this);
		
		m_uwTss.SetColumn (0, new Vector4 (1.0f, 0.0f, 0.0f, 0.0f));
		m_uwTss.SetColumn (1, new Vector4 (0.0f, 0.0f, 1.0f, 0.0f));
		m_uwTss.SetColumn (2, new Vector4 (0.0f, 1.0f, 0.0f, 0.0f));
		m_uwTss.SetColumn (3, new Vector4 (0.0f, 0.0f, 0.0f, 1.0f));
		
		m_cTuc.SetColumn (0, new Vector4 (1.0f, 0.0f, 0.0f, 0.0f));
		m_cTuc.SetColumn (1, new Vector4 (0.0f, -1.0f, 0.0f, 0.0f));
		m_cTuc.SetColumn (2, new Vector4 (0.0f, 0.0f, 1.0f, 0.0f));
		m_cTuc.SetColumn (3, new Vector4 (0.0f, 0.0f, 0.0f, 1.0f));
		
		m_triangles = new int[VERT_COUNT];
		m_colors = new Color[VERT_COUNT];
		// Assign triangles, note: this is just for visualizing point in the mesh data.
		for (int i = 0; i < VERT_COUNT; i++)
		{
			m_triangles[i] = i;
			m_colors[i] = Color.white;
		}
		
		//m_pointCloud = new List<GameObject>();
		m_subCloud = new GameObject("PointCloud");
		m_subCloud.AddComponent<MeshFilter>();
		m_subCloud.AddComponent<MeshRenderer>();
		Material newMat = Resources.Load("pointcloud", typeof(Material)) as Material;
		m_subCloud.GetComponent<MeshRenderer>().material = newMat;
		
		m_mesh = m_subCloud.GetComponent<MeshFilter>().mesh;
		m_mesh.Clear();
		m_mesh.triangles = m_triangles;
		m_mesh.colors = m_colors;
		m_mesh.RecalculateBounds();
		m_mesh.RecalculateNormals();
		
	}
	
	/// <summary>
	/// Callback that gets called when depth is available
	/// from the Tango Service.
	/// DO NOT USE THE UNITY API FROM INSIDE THIS FUNCTION!
	/// </summary>
	/// <param name="callbackContext">Callback context.</param>
	/// <param name="xyzij">Xyzij.</param>
	public void OnTangoDepthAvailable(TangoUnityDepth tangoDepth)
	{
		// Calculate the time since the last successful depth data
		// collection.
		if (m_previousDepthDeltaTime == 0.0)
		{
			m_previousDepthDeltaTime = tangoDepth.m_timestamp;
		}
		else
		{
			m_depthDeltaTime = (float)((tangoDepth.m_timestamp - m_previousDepthDeltaTime) * 1000.0);
			m_previousDepthDeltaTime = tangoDepth.m_timestamp;
		}
		
		// Fill in the data to draw the point cloud.
		if (tangoDepth != null && tangoDepth.m_points != null)
		{
			int numberOfActiveVertices = tangoDepth.m_pointCount;
			m_pointsCount = numberOfActiveVertices;
			float validPointCount = 0;
			if(numberOfActiveVertices > 0 && (m_scan_state == SCAN_STATE_ON || m_scan_state == SCAN_STATE_SCREENSHOT || m_scan_state == SCAN_STATE_STREAM))
			{   
				if (m_scan_state == SCAN_STATE_SCREENSHOT)
					m_scan_state = SCAN_STATE_OFF;
				if (m_scan_state == SCAN_STATE_ON && m_initiatal_delete)
				{
					ClearPointClouds();
					m_initiatal_delete = false;
				}
				_SetUpExtrinsics();
				TangoCoordinateFramePair pair;
				TangoPoseData poseData = new TangoPoseData();
				// Query pose to transform point cloud to world coordinates, here we are using the timestamp
				// that we get from depth.
				pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_START_OF_SERVICE;
				pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
				PoseProvider.GetPoseAtTime(poseData, m_previousDepthDeltaTime, pair);
				Vector3 position = new Vector3((float)poseData.translation[0],
				                               (float)poseData.translation[1],
				                               (float)poseData.translation[2]);
				Quaternion quat = new Quaternion((float)poseData.orientation[0],
				                                 (float)poseData.orientation[1],
				                                 (float)poseData.orientation[2],
				                                 (float)poseData.orientation[3]);
				m_ssTd = Matrix4x4.TRS(position, quat, Vector3.one);
				
				// The transformation matrix that represents the pointcloud's pose. 
				// Explanation: 
				// The pointcloud which is in RGB's camera frame, is put in unity world's 
				// coordinate system(wrt unit camera).
				// Then we are extracting the position and rotation from uwTuc matrix and applying it to 
				// the PointCloud's transform.
				Matrix4x4 uwTuc = m_uwTss * m_ssTd * Matrix4x4.Inverse(m_imuTd) * m_imuTc * m_cTuc;
				
				Vector3[] pointCloudVertices  = new Vector3[VERT_COUNT];
				
				// Converting points array to point vector array for Unity to create a mesh.
				for(int i = 0; i < numberOfActiveVertices; ++i)
				{
					pointCloudVertices[i] = new Vector3(tangoDepth.m_points[i * 3],
					                                    tangoDepth.m_points[i * 3 + 1],
					                                    tangoDepth.m_points[i * 3 + 2]);
					m_overallZ += pointCloudVertices[i].z;
					++validPointCount;
				}

				if (m_scan_state == SCAN_STATE_STREAM)
				{
					ClearPointClouds();
				}

				m_subCloud = new GameObject("PointCloud");
				m_subCloud.transform.position = uwTuc.GetColumn(3);
				m_subCloud.transform.rotation = Quaternion.LookRotation(uwTuc.GetColumn(2), uwTuc.GetColumn(1));
				m_subCloud.AddComponent<MeshFilter>();
				m_subCloud.AddComponent<MeshRenderer>();
				Material newMat;
				if (m_scan_state == SCAN_STATE_STREAM)
				{
					newMat = Resources.Load("pointcloud", typeof(Material)) as Material;
				}
				else
				{
					newMat = Resources.Load("pointcloudcustom", typeof(Material)) as Material;
				}
				m_subCloud.GetComponent<MeshRenderer>().material = newMat;
				m_mesh = m_subCloud.GetComponent<MeshFilter>().mesh;
				
				m_mesh.Clear();
				m_mesh.vertices = pointCloudVertices;
				m_mesh.triangles = m_triangles;
				m_mesh.colors = m_colors;
				m_mesh.SetIndices(m_triangles, MeshTopology.Points, 0);
			}
			// Don't divide by zero!
			if (validPointCount != 0)
			{
				m_overallZ = m_overallZ / validPointCount;
			} 
			else
			{
				m_overallZ = 0;
			}
		}
	}
	
	private void _SetUpExtrinsics()
	{
		double timestamp = 0.0;
		TangoCoordinateFramePair pair;
		TangoPoseData poseData = new TangoPoseData();
		// Query the extrinsics between IMU and device frame.
		pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
		pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
		PoseProvider.GetPoseAtTime(poseData, timestamp, pair);
		Vector3 position = new Vector3((float)poseData.translation[0],
		                               (float)poseData.translation[1],
		                               (float)poseData.translation[2]);
		Quaternion quat = new Quaternion((float)poseData.orientation[0],
		                                 (float)poseData.orientation[1],
		                                 (float)poseData.orientation[2],
		                                 (float)poseData.orientation[3]);
		m_imuTd = Matrix4x4.TRS(position, quat, new Vector3 (1.0f, 1.0f, 1.0f));
		
		// Query the extrinsics between IMU and color camera frame.
		pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
		pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_CAMERA_COLOR;
		PoseProvider.GetPoseAtTime(poseData, timestamp, pair);
		position = new Vector3((float)poseData.translation[0],
		                       (float)poseData.translation[1],
		                       (float)poseData.translation[2]);
		quat = new Quaternion((float)poseData.orientation[0],
		                      (float)poseData.orientation[1],
		                      (float)poseData.orientation[2],
		                      (float)poseData.orientation[3]);
		m_imuTc = Matrix4x4.TRS(position, quat, new Vector3 (1.0f, 1.0f, 1.0f));
	}
	
	void OnGUI()
	{
		Color oldColor = GUI.color;
		//Color oldBackgroundColor = GUI.backgroundColor;

		GUI.color = Color.white;
		//GUI.backgroundColor = new Color(0.0F, 0.4F, 0.5F);

		GUIStyle scanButtonStyle = new GUIStyle("button");
		Texture2D blueTexture = new Texture2D(1,1);
		blueTexture.SetPixel(0,0,new Color(0.0F, 0.4F, 1.0F, 0.8F));
		blueTexture.Apply();
		scanButtonStyle.normal.background = blueTexture;
		scanButtonStyle.fontSize = 25;

		if (GUI.Button (new Rect (50, Screen.height/2, 200, 100), "Screenshot", scanButtonStyle))
			m_scan_state = SCAN_STATE_SCREENSHOT;

		if (m_scan_state == SCAN_STATE_ON) {
			if (GUI.Button (new Rect (50, Screen.height/2 + 120, 200, 100), "Stop Scan", scanButtonStyle))
				m_scan_state = SCAN_STATE_OFF;
		} else {
			if (GUI.Button (new Rect (50, Screen.height / 2 + 120, 200, 100), "Start Scan", scanButtonStyle))
			{
				m_scan_state = SCAN_STATE_ON;
				m_initiatal_delete = true;
			}
		}
		
		if (GUI.Button (new Rect (50, Screen.height / 2 + 240, 200, 100), "Clear Scan", scanButtonStyle)) {
			ClearPointClouds();
		}
		
		
		if (GUI.Button (new Rect (50, Screen.height / 2 + 360, 200, 100), "Save Scan", scanButtonStyle)) {
			TimeSpan tspan = DateTime.UtcNow - new DateTime(1970, 1, 1);
			double epoch = tspan.TotalMilliseconds;

			string filePath = Application.persistentDataPath + "/data.txt";

			FileStream _file = new System.IO.FileStream(filePath, FileMode.Create, FileAccess.Write);
			StreamWriter sw = new StreamWriter(_file); 	
			// sw.WriteLine("write stuff here");
			sw.Close();
			_file.Close();

			message = epoch.ToString();


		}


		if (GUI.Button (new Rect (Screen.width-270, Screen.height-300, 250, 80), "Raw", scanButtonStyle)) {
			CameraController cameraRef = GameObject.Find("Camera").GetComponent<CameraController>();
			cameraRef.EnableCamera(CameraController.CameraType.FIRST_PERSON);
			ClearPointClouds();
			m_scan_state = SCAN_STATE_STREAM;
		}

		GUIStyle videoButtonStyle = new GUIStyle("button");
		Texture2D whiteTexture = new Texture2D(1,1);
		whiteTexture.SetPixel(0,0,new Color(1.0F, 1.0F, 1.0F, 0.8F));
		whiteTexture.Apply();
		videoButtonStyle.normal.background = whiteTexture;
		videoButtonStyle.fontSize = 25;
		
		GUI.Button(new Rect (50, 50, 533, 300), "Video Preview", videoButtonStyle);

		//GUI.Button (new Rect (Screen.width / 2 + 50, 300, 900, 100), message);

		GUI.color = oldColor;
		//GUI.backgroundColor = oldBackgroundColor;
	}

	public void ClearPointClouds()
	{
		foreach (var gameObj in FindObjectsOfType(typeof(GameObject)) as GameObject[])
		{
			if(gameObj.name == "PointCloud")
			{
				Destroy(gameObj);
			}
		}
	}
}