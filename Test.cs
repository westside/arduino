using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System;

public class ArduinoSerialCommunication : MonoBehaviour {

    public SerialPort stream = new SerialPort("\\\\.\\COM31", 115200);
    public float smooth = 2.0F;
	// Use this for initialization
	void Start () {
        stream.Open(); //Open the Serial Stream.
        Debug.Log(this);
         
        //set(90f, 90f, 90f);
	}
	
	// Update is called once per frame
	void Update () {
        if (stream.IsOpen)
        {
            try
            {
                string lines = stream.ReadLine();
                string[] values = lines.Split(' ');
                float yaw = float.Parse(values[0]);
                float pitch = float.Parse(values[1]);
                float roll = float.Parse(values[2]);
                Debug.Log(yaw + " " + pitch + " " + roll);

                set(yaw, pitch, roll);

            }
            catch (Exception e)
            {
                Debug.Log(e.GetBaseException());
            }
        }
	}

    void set(float yaw, float pitch, float roll)
    {
        Quaternion target = Quaternion.Euler(pitch, yaw, roll);
        transform.rotation = Quaternion.Slerp(transform.rotation, target, Time.time * smooth);
    }
}
