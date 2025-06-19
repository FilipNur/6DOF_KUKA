using UnityEngine;
using System.Globalization;
using Unity.VisualScripting;
using System;
using Unity.Mathematics;

public class RobotScript : MonoBehaviour
{
	private float[] angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

   private Vector3 tempVec3 = new Vector3();
   private Vector2 tempVec2 = new Vector2();
   private Vector3 axisX = new Vector3(1, 0, 0);
   private Vector3 axisY = new Vector3(0, 1, 0);
   private Vector3 axisZ = new Vector3(0, 0, 1);

   private const float length_a = 0.52f;
   private const float length_b = 0.16f;
   private const float length_c = 0.98f;
   private const float length_d = 0.15f;
   private const float length_e = 0.86f;
   private float length_de;
	
	private float q3_bias;

	private float[] diffScore = new float[8];

	private bool firstPos = true;

   [SerializeField] Transform GoalPoint;
	[SerializeField] Transform transform_Base;
	[SerializeField] Transform transform_A1;
	[SerializeField] Transform transform_A2;
	[SerializeField] Transform transform_A3;
	[SerializeField] Transform transform_A4;
	[SerializeField] Transform transform_A5;
	[SerializeField] Transform transform_A6;

   [SerializeField] float length_f = 0.18328f;

	[Range(0, 7)]
	[SerializeField] int startPosID;

	[SerializeField] bool overwritePosition = false;

	private bool isFirstAngleAssigmnemt = true;
	private float[] currentAngles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	private float[] maxAngularSpeeds = {200.0f, 175.0f, 190.0f, 430.0f, 430.0f, 630.0f};
	private float[] lowerAngleLimits = {-185.0f, -185.0f, -138.0f, -350.0f, -130.0f, -350.0f};
	private float[] upperAngleLimits = {185.0f, 65.0f, 175.0f, 350.0f, 130.0f, 350.0f};

   void Start()
   {
      length_de = Mathf.Sqrt(length_d * length_d + length_e * length_e);
		q3_bias = (float)Math.Atan2(length_d, length_e);

		isFirstAngleAssigmnemt = true;
   }

   void Update()
   {
		InverseKinematics(ref angles, GoalPoint.position, GoalPoint.rotation);

		if (isFirstAngleAssigmnemt == true)
		{
			isFirstAngleAssigmnemt = false;

			for (int i = 0; i < 6; i++)
			{
				currentAngles[i] = angles[i];
			}
		}
		else
		{
			NextPosCalculation(ref currentAngles, ref angles, Time.deltaTime);
		}

		transform_A1.localRotation = Quaternion.Euler(0.0f, -currentAngles[0], 0.0f);
		transform_A2.localRotation = Quaternion.Euler(0.0f, 0.0f, -currentAngles[1]);
		transform_A3.localRotation = Quaternion.Euler(0.0f, 0.0f, -currentAngles[2]);
		transform_A4.localRotation = Quaternion.Euler(-currentAngles[3], 0.0f, 0.0f);
		transform_A5.localRotation = Quaternion.Euler(0.0f, 0.0f, -currentAngles[4]);
		transform_A6.localRotation = Quaternion.Euler(-currentAngles[5], 0.0f, 0.0f);
   }


   void InverseKinematics(ref float[] angles, Vector3 goalPos, Quaternion goalOrientation)
   {
		float[] q1 = new float[8];

		float[] q2 = new float[8];

		float[] q3 = new float[8];

		float[] q4 = new float[8];

		float[] q5 = new float[8];

		float[] q6 = new float[8];

		Vector3 newGoalPos = goalPos - transform_Base.position;
		newGoalPos.x = -newGoalPos.x;
		
		goalOrientation.Normalize();
      Vector3 localAxisX = goalOrientation * axisX;
      Vector3 localAxisY = goalOrientation * axisY;
      Vector3 localAxisZ = goalOrientation * axisZ;

		localAxisX.y = -localAxisX.y;
		localAxisX.z = -localAxisX.z;
		localAxisY.x = -localAxisY.x;
		localAxisZ.x = -localAxisZ.x;

		Vector3 posA5 = newGoalPos - length_f * localAxisZ;

		#region q1
		
			q1[0] = (float)Math.Atan2(posA5.x, posA5.z);

			if (q1[0] < 0)
			{
				q1[0] += (float)(2.0 * Math.PI);
			}

			if (q1[0] >= Math.PI)
			{
				q1[2] = (float)(q1[0] - Math.PI);
			}
			else
			{
				q1[2] = (float)(q1[0] + Math.PI);
			}

			q1[0] *= Mathf.Rad2Deg;
			q1[2] *= Mathf.Rad2Deg;

			q1[1] = q1[0];
			q1[3] = q1[2];

			q1[4] = q1[0];
			q1[5] = q1[1];
			q1[6] = q1[2];
			q1[7] = q1[3];

		#endregion

		#region q2_q3

			tempVec2.x = (float)Math.Sqrt(posA5.x * posA5.x + posA5.z * posA5.z);
			tempVec2.y = posA5.y;
			Vector2 P5xy = tempVec2;

			Vector2 P5xy0 = P5xy;
			P5xy0.x -= length_b;
			P5xy0.y -= length_a;
		
			float length_P5xy0_sq = P5xy0.x * P5xy0.x + P5xy0.y * P5xy0.y;
			float length_P5xy0 = (float)Math.Sqrt(length_P5xy0_sq);

			float length_c_sq = length_c * length_c;
			float length_de_sq = length_de * length_de;

			float alpha = (float)Math.Acos((length_c_sq + length_de_sq - length_P5xy0_sq) / (2.0f * length_c * length_de));
			float beta = (float)Math.Atan2(P5xy0.y, P5xy0.x);
			float gamma = (float)Math.Acos((length_c_sq + length_P5xy0_sq - length_de_sq) / (2.0f * length_c * length_P5xy0));

			q2[0] = beta + gamma;
			q2[1] = beta - gamma;

			q3[0] = (float)(-(Math.PI - alpha) - q3_bias);
			q3[1] = (float)((Math.PI - alpha) - q3_bias);

			q2[0] *= -Mathf.Rad2Deg; 
			q2[1] *= -Mathf.Rad2Deg;

			q3[0] *= -Mathf.Rad2Deg;
			q3[1] *= -Mathf.Rad2Deg;
			

			
			P5xy0 = P5xy;
			P5xy0.x += length_b;
			P5xy0.y -= length_a;

			length_P5xy0_sq = P5xy0.x * P5xy0.x + P5xy0.y * P5xy0.y;
			length_P5xy0 = (float)Math.Sqrt(length_P5xy0_sq);

			length_c_sq = length_c * length_c;
			length_de_sq = length_de * length_de;

			alpha = (float)Math.Acos((length_c_sq + length_de_sq - length_P5xy0_sq) / (2.0f * length_c * length_de));
			beta = (float)Math.Atan2(P5xy0.y, P5xy0.x);
			gamma = (float)Math.Acos((length_c_sq + length_P5xy0_sq - length_de_sq) / (2.0f * length_c * length_P5xy0));

			q2[2] = beta + gamma;
			q2[3] = beta - gamma;

			q2[2] = (float)(Math.PI - (double)q2[2]);
			q2[3] = (float)(Math.PI - (double)q2[3]);

			q3[2] = -(float)(-(Math.PI - alpha) + q3_bias);
			q3[3] = -(float)((Math.PI - alpha) + q3_bias);

			q2[2] *= -Mathf.Rad2Deg;
			q2[3] *= -Mathf.Rad2Deg;

			q3[2] *= -Mathf.Rad2Deg;
			q3[3] *= -Mathf.Rad2Deg;


			q2[4] = q2[0];
			q2[5] = q2[1];
			q2[6] = q2[2];
			q2[7] = q2[3];

			q3[4] = q3[0];
			q3[5] = q3[1];
			q3[6] = q3[2];
			q3[7] = q3[3];

		#endregion

		#region q4

		for (int i = 0; i < 4; i++)
		{
			Vector3 A3_axisZ = Quaternion.Euler(q2[i] + q3[i], 0.0f, 0.0f) * axisZ;
			A3_axisZ = Quaternion.Euler(0.0f, q1[i], 0.0f) * A3_axisZ;
			A3_axisZ.Normalize();

			Vector3 goalPoint_A3PlanePos = (newGoalPos - posA5) - A3_axisZ * Vector3.Dot((newGoalPos - posA5), A3_axisZ);

			Vector3 goalPoint_A3PlanePos0 = Quaternion.Euler(0.0f, -q1[i], 0.0f) * goalPoint_A3PlanePos;
			goalPoint_A3PlanePos0 = Quaternion.Euler(-q2[i] - q3[i], 0.0f, 0.0f) * goalPoint_A3PlanePos0;

			q4[i] = (float)Math.Atan2(goalPoint_A3PlanePos0.x, goalPoint_A3PlanePos0.y);

			q4[i] *= -Mathf.Rad2Deg;

			if (q4[i] < 0.0f)
			{
				q4[i+4] = q4[i];
				q4[i] += 180.0f;
			}
			else
			{
				q4[i+4] = q4[i] - 180.0f;
			}
		}

		#endregion

		#region q5	

			for (int i = 0; i < 8; i++)
			{
				Vector3 goalPos_A5_0 = newGoalPos - posA5;

				goalPos_A5_0 = Quaternion.Euler(0.0f, -q1[i], 0.0f) * goalPos_A5_0;
				goalPos_A5_0 = Quaternion.Euler(-q2[i] - q3[i], 0.0f, 0.0f) * goalPos_A5_0;
				goalPos_A5_0 = Quaternion.Euler(0.0f, 0.0f, -q4[i]) * goalPos_A5_0;


				q5[i] = (float)Math.Atan2(goalPos_A5_0.y, goalPos_A5_0.z);

				q5[i] *= -Mathf.Rad2Deg;
			}

		#endregion

		#region q6

			for (int i = 0; i < 8; i++)
			{

				Quaternion Q1;
				Quaternion Q2_Q3;
				Quaternion Q4;
				Quaternion Q5;
				
				if (Mathf.Approximately(q1[i], 0.0f))
				{
					Q1 = Quaternion.identity;
				}
				else
				{
					Q1 = Quaternion.Euler(0.0f, q1[i], 0.0f);
				}

				if (Mathf.Approximately((-q2[i] - q3[i]), 0.0f))
				{
					Q2_Q3 = Quaternion.identity;
				}
				else
				{
					Q2_Q3 = Quaternion.Euler(q2[i] + q3[i], 0.0f, 0.0f);
				}

				if (Mathf.Approximately(q4[i], 0.0f))
				{
					Q4 = Quaternion.identity;
				}
				else
				{
					Q4 = Quaternion.Euler(0.0f, 0.0f, q4[i]);
				}
				
				if (Mathf.Approximately(q4[i], 0.0f))
				{
					Q5 = Quaternion.identity;
				}
				else
				{
					Q5 = Quaternion.Euler(q5[i], 0.0f, 0.0f);
				}

				Quaternion Q = Q1 * Q2_Q3 * Q4 * Q5;

				Vector3 transformedX = Q * axisX;
				transformedX.Normalize();

				Vector3 transformedY = Q * axisY;
				transformedY.Normalize();

				//localAxisX.Normalize();
				//localAxisY.Normalize();

				float cos_xx_q6 = Vector3.Dot(transformedX, localAxisX);
				float cos_yx_q6 = Vector3.Dot(transformedY, localAxisX);
				float cos_xy_q6 = Vector3.Dot(transformedX, localAxisY);
				float cos_yy_q6 = Vector3.Dot(transformedY, localAxisY);


				float q6_xx = (float)Math.Acos(cos_xx_q6);
				float q6_yx = (float)Math.Acos(cos_yx_q6);
				float q6_xy = (float)Math.Acos(cos_xy_q6);
				float q6_yy = (float)Math.Acos(cos_yy_q6);

				if (q6_xx <= (float)(Math.PI / 2.0))
				{
					if (q6_yx <= (float)(Math.PI / 2.0))
					{
						q6[i] = q6_xx;
					}
					else
					{
						q6[i] = -q6_xx;
					}
				}
				else
				{
					if (q6_yx <= (float)(Math.PI / 2.0))
					{
						q6[i] = q6_xx;
					}
					else
					{
						q6[i] = -q6_xx;
					}
				}

				q6[i] *= Mathf.Rad2Deg;
			}

		#endregion

		int chosenID = 0;

		#region nextPosChoosing

			if (firstPos == true)
			{
				firstPos = false;
				chosenID = startPosID;
			}
			else
			{
				if (overwritePosition == true)
				{
					chosenID = startPosID;
				}
				else
				{
					for (int i = 0; i < 8; i++)
					{
						diffScore[i] = Mathf.Abs(Mathf.DeltaAngle(currentAngles[0], q1[i])) * 1.0f;
						diffScore[i] += Mathf.Abs(Mathf.DeltaAngle(currentAngles[1], q2[i])) * 0.5f;
						diffScore[i] += Mathf.Abs(Mathf.DeltaAngle(currentAngles[2], q3[i])) * 0.25f;
						diffScore[i] += Mathf.Abs(Mathf.DeltaAngle(currentAngles[3], q4[i])) * 0.3f;
						diffScore[i] += Mathf.Abs(Mathf.DeltaAngle(currentAngles[4], q5[i])) * 0.0625f;
						diffScore[i] += Mathf.Abs(Mathf.DeltaAngle(currentAngles[5], q6[i])) * 0.03125f;
					}

					chosenID = 0;
					float minScore = diffScore[0];

					for (int i = 1; i < 8; i++)
					{
						if (diffScore[i] < minScore)
						{
							chosenID = i;
							minScore = diffScore[i];
						}
					}
				}
			}

		#endregion nextPosChoosing

		angles[0] = q1[chosenID];
		angles[1] = q2[chosenID];
		angles[2] = q3[chosenID];
		angles[3] = q4[chosenID];
		angles[4] = q5[chosenID];
		angles[5] = q6[chosenID];
   }

	float Signum(float value)
	{
		if (value > 0.0f)
		{
			return 1.0f;
		}
		else if (value < 0.0f)
		{
			return -1.0f;
		}
		else
		{
			return 0.0f;
		}
	}

	float jointAngleDifference(int jointID, float start, float end)
	{
		float angleDiff = Mathf.DeltaAngle(start, end);

		if (((start + angleDiff) > upperAngleLimits[jointID]) || ((start + angleDiff) < lowerAngleLimits[jointID]))
		{
			if (angleDiff < 0.0f)
			{
				angleDiff = 360.0f + angleDiff;
			}
			else
			{
				angleDiff = -360.0f + angleDiff;
			}

			if (((start + angleDiff) > upperAngleLimits[jointID]) || ((start + angleDiff) < lowerAngleLimits[jointID]))
			{
				return float.NaN;
			}
			else
			{
				return angleDiff;
			}
		}
		else
		{
			return angleDiff;
		}
	}
	
	void NextPosCalculation(ref float[] currentAngles, ref float[] destinationAngles, float deltaTime)
	{
		if (destinationAngles[0] != 0.0f)
		{
			destinationAngles[0] = destinationAngles[0];
		}

		float[] angleDiffs = new float[6];
		float[] movementTimeWithMaxSpeed = new float[6];

		float maxMoveTime = -1.0f;

		for (int i = 0; i < 6; i++)
		{
			angleDiffs[i] = jointAngleDifference(i, currentAngles[i], destinationAngles[i]);
			
			movementTimeWithMaxSpeed[i] = Mathf.Abs(angleDiffs[i]) / maxAngularSpeeds[i];

			if (movementTimeWithMaxSpeed[i] > maxMoveTime)
			{
				maxMoveTime = movementTimeWithMaxSpeed[i];
			}
		}

		if (maxMoveTime == 0.0f)
		{
			return;
		}

		float[] newJointAngularSpeed = new float[6];

		for (int i = 0; i < 6; i++)
		{
			newJointAngularSpeed[i] = Mathf.Abs(angleDiffs[i] / maxMoveTime);
		}

		for (int i = 0; i < 6; i++)
		{
			if (float.IsNaN(angleDiffs[i]) == false)
			{
				currentAngles[i] = Mathf.MoveTowards(currentAngles[i], currentAngles[i] + angleDiffs[i], newJointAngularSpeed[i] * Time.deltaTime);
			}
		}
	}
}
