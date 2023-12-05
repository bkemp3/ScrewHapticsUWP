using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class BleTest : MonoBehaviour
{
    // Change this to match your device.
    string targetDeviceName = "ESP32-RUST";
    string serviceUuid = "{4fafc201-1fb5-459e-8fcc-c5c9c331914b}";
    string[] characteristicUuids = {
         "{beb5483e-36e1-4688-b7f5-ea07361b26a8}",      // CUUID 1
    };

    BLE ble;
    BLE.BLEScan scan;
    bool isScanning = false, isConnected = false;
    string deviceId = null;  
    IDictionary<string, string> discoveredDevices = new Dictionary<string, string>();
    int devicesCount = 0;
    Quaternion quaternion = new Quaternion(0, 0, 0, 0);

    float revolutions = 0.0f;

    // BLE Threads 
    Thread scanningThread, connectionThread, readingThread;

    // GUI elements
    public Text TextDiscoveredDevices, TextIsScanning, TextTargetDeviceConnection, TextTargetDeviceData;
    public Button ButtonEstablishConnection, ButtonStartScan;
    string remoteAngle, lastRemoteAngle;

    // Start is called before the first frame update
    void Start()
    {
        ble = new BLE();
        ButtonEstablishConnection.enabled = false;
        TextTargetDeviceConnection.text = targetDeviceName + " not found.";
        readingThread = new Thread(ReadBleData);
    }

    // Update is called once per frame
    void Update()
    {  
        if (isScanning)
        {
            if (ButtonStartScan.enabled)
                ButtonStartScan.enabled = false;

            if (discoveredDevices.Count > devicesCount)
            {
                UpdateGuiText("scan");
                devicesCount = discoveredDevices.Count;
            }                
        } else
        {
            /* Restart scan in same play session not supported yet.
            if (!ButtonStartScan.enabled)
                ButtonStartScan.enabled = true;
            */
            if (TextIsScanning.text != "Not scanning.")
            {
                TextIsScanning.color = Color.white;
                TextIsScanning.text = "Not scanning.";
            }
        }

        // The target device was found.
        if (deviceId != null && deviceId != "-1")
        {
            // Target device is connected and GUI knows.
            if (ble.isConnected && isConnected)
            {
                UpdateGuiText("writeData");
            }
            // Target device is connected, but GUI hasn't updated yet.
            else if (ble.isConnected && !isConnected)
            {
                UpdateGuiText("connected");
                isConnected = true;
            // Device was found, but not connected yet. 
            } else if (!ButtonEstablishConnection.enabled && !isConnected)
            {
                ButtonEstablishConnection.enabled = true;
                TextTargetDeviceConnection.text = "Found target device:\n" + targetDeviceName;
            } 
        } 
    }

    private void OnDestroy()
    {
        CleanUp();
    }

    private void OnApplicationQuit()
    {
        CleanUp();
    }

    // Prevent threading issues and free BLE stack.
    // Can cause Unity to freeze and lead
    // to errors when omitted.
    private void CleanUp()
    {
        try
        {
            scan.Cancel();
            ble.Close();
            scanningThread.Abort();
            connectionThread.Abort();
        } catch(NullReferenceException e)
        {
            Debug.Log("Thread or object never initialized.\n" + e);
        }        
    }

    public void StartScanHandler()
    {
        devicesCount = 0;
        isScanning = true;
        discoveredDevices.Clear();
        scanningThread = new Thread(ScanBleDevices);
        scanningThread.Start();
        TextIsScanning.color = new Color(244, 180, 26);
        TextIsScanning.text = "Scanning...";
        TextDiscoveredDevices.text = "";
    }

    public void ResetHandler()
    {
        TextTargetDeviceData.text = "";
        TextTargetDeviceConnection.text = targetDeviceName + " not found.";
        // Reset previous discovered devices
        discoveredDevices.Clear();
        TextDiscoveredDevices.text = "No devices.";
        deviceId = null;
        CleanUp();
    }

    private void ReadBleData(object obj)
    {
        byte[] packageReceived = BLE.ReadBytes();
        string string_in = System.Text.Encoding.ASCII.GetString(packageReceived);
        string[] string_array = string_in.Split(",");  
        Debug.Log("Received: " + string_in); 
        // float[] q = new float[4];
        // int i = 0;
        // foreach(var item in string_array)
        // {
        //     Debug.Log(item.ToString());
        //     q[i] = float.Parse(item.ToString());
        //     i = i+1;
        // }

        quaternion.x = float.Parse(string_array[0].ToString());
        quaternion.y = float.Parse(string_array[1].ToString());
        quaternion.z = float.Parse(string_array[2].ToString());
        quaternion.w = float.Parse(string_array[3].ToString());
        // revolutions = float.Parse(string_array[4].ToString());
        Thread.Sleep(15);
    }

    void UpdateGuiText(string action)
    {
        switch(action) {
            case "scan":
                TextDiscoveredDevices.text = "";
                // IDictionary<string, string> tempDiscoveredDevices = new Dictionary<string, string>();
                // tempDiscoveredDevices = discoveredDevices;
                // foreach (KeyValuePair<string, string> entry in tempDiscoveredDevices)
                // {
                //     TextDiscoveredDevices.text += "DeviceID: " + entry.Key + "\nDeviceName: " + entry.Value + "\n\n";
                //     Debug.Log("Added device: " + entry.Key);
                // }
                break;
            case "connected":
                ButtonEstablishConnection.enabled = false;
                TextTargetDeviceConnection.text = "Connected to target device:\n" + targetDeviceName;
                break;
            case "writeData":
                if (!readingThread.IsAlive)
                {
                    readingThread = new Thread(ReadBleData);
                    readingThread.Start();
                }
                if (remoteAngle != lastRemoteAngle)
                {
                    TextTargetDeviceData.text = "Remote angle: " + remoteAngle;
                    lastRemoteAngle = remoteAngle;
                }
                break;
        }
    }

    void ScanBleDevices()
    {
        scan = BLE.ScanDevices();
        Debug.Log("BLE.ScanDevices() started.");
        scan.Found = (_deviceId, deviceName) =>
        {
            if (!discoveredDevices.ContainsKey(_deviceId))
            {
                Debug.Log("found device with name: " + deviceName);
                discoveredDevices.TryAdd(_deviceId, deviceName);
                // Thread.Sleep(50);

            }
            if (deviceId == null && deviceName == targetDeviceName)
                deviceId = _deviceId;
        };

        scan.Finished = () =>
        {
            isScanning = false;
            Debug.Log("scan finished");
            if (deviceId == null)
                deviceId = "-1";
        };
        while (deviceId == null)
            Thread.Sleep(500);
        scan.Cancel();
        scanningThread = null;
        isScanning = false;

        if (deviceId == "-1")
        {
            Debug.Log("no device found!");
            return;
        }
    }

    // Start establish BLE connection with
    // target device in dedicated thread.
    public void StartConHandler()
    {
        connectionThread = new Thread(ConnectBleDevice);
        connectionThread.Start();
    }

    void ConnectBleDevice()
    {
        if (deviceId != null)
        {
            try
            {
                ble.Connect(deviceId,
                serviceUuid,
                characteristicUuids);
            } catch(Exception e)
            {
                Debug.Log("Could not establish connection to device with ID " + deviceId + "\n" + e);
            }
        }
        if (ble.isConnected)
            Debug.Log("Connected to: " + targetDeviceName);
    }

    ulong ConvertLittleEndian(byte[] array)
    {
        int pos = 0;
        ulong result = 0;
        foreach (byte by in array)
        {
            result |= ((ulong)by) << pos;
            pos += 8;
        }
        return result;
    }

    public Quaternion get_quaternion()
    {
        return quaternion;
    }

    public float get_revolutions()
    {
        return revolutions;
    }
}
