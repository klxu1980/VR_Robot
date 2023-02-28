using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.MemoryMappedFiles;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;


class CSharedMemory
{
    // 基于共享内存完成VR和外部程序的数据交换
    // 共享内存的前1024字节用于向外部程序传递VR头盔和手柄的姿态数据，其余空间用于外部程序向VR传递图像数据
    private MemoryMappedViewAccessor __viewAccessor;
    private Mutex __mutex;
    public  List<byte[]> Images;

    public CSharedMemory(String MemName, int MemSize)
    {
        var mmf = MemoryMappedFile.CreateOrOpen(MemName, MemSize);
        __viewAccessor = mmf.CreateViewAccessor(0, MemSize);
        __mutex = new Mutex(false, MemName + "_mutex");

        Images = new List<byte[]>();
    }

    public void ReadImages(int Begin = 1024)
    {
        int Offset = Begin;
        int[] ImgCnt = new int[1];
        int[] ImgSize = new int[1];
        Images.Clear();

        __mutex.WaitOne();

        // 读取图像数量
        __viewAccessor.ReadArray<int>(Offset, ImgCnt, 0, 1);
        Offset += 4;

        // 依次读取图像
        for (int i = 0; i < ImgCnt[0]; ++i)
        {
            __viewAccessor.ReadArray<int>(Offset, ImgSize, 0, 1);

            byte[] ImgBuffer = new byte[ImgSize[0]];
            __viewAccessor.ReadArray<byte>(Offset + 4, ImgBuffer, 0, ImgSize[0]);

            Images.Add(ImgBuffer);
            Offset += ImgSize[0] + 4;
        }
        __mutex.ReleaseMutex();
    }

    public void WriteBytes(byte[] Data, int Begin = 0)
    {
        __mutex.WaitOne();
        __viewAccessor.WriteArray<byte>(Begin, Data, 0, Data.Length);
        __mutex.ReleaseMutex();
    }

    public void ReadBytes(byte[] Data, int Length, int Begin = 0)
    {
        __mutex.WaitOne();
        __viewAccessor.ReadArray<byte>(Begin, Data, 0, Length);
        __mutex.ReleaseMutex();
    }
}


public class ShareMem : MonoBehaviour
{
    public  RawImage    LeftEye;       // 左眼图像
    public  RawImage    RightEye;      // 右眼图像
    private InputDevice headSet;       // 头盔输入设备
    private InputDevice leftHand;      // 左手柄
    private InputDevice rightHand;     // 右手柄

    private CSharedMemory __SharedMem;
    private Texture2D     __Texture;
    private int           __RefCount;

    // Start is called before the first frame update
    void Start()
    {
        __SharedMem = new CSharedMemory("ShareForUnity", 2208 * 1242 * 3 + 1024);
        __Texture   = new Texture2D(1, 1);

        InitDevices();

        Debug.Log("Test started");
        __RefCount = 0;
    }

    void InitDevices()
    {
        var headDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.Head, headDevices);
        if (headDevices.Count == 1)
        {
            headSet = headDevices[0];
            Debug.Log("Find headset");
        }
        else
        {
            if (headDevices.Count == 0)
                Debug.Log("No headset can be found");
            else if (headDevices.Count > 1)
                Debug.Log("Find more than one head!");
        }

        var leftHands = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, leftHands);
        if (leftHands.Count == 1)
        {
            leftHand = leftHands[0];
            Debug.Log("Find left hand");
        }
        else
        {
            if (leftHands.Count == 0)
                Debug.Log("No left hand can be found");
            else if (leftHands.Count > 1)
                Debug.Log("Find more than one left hands!");
        }

        var rightHands = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, rightHands);
        if (rightHands.Count == 1)
        {
            rightHand = rightHands[0];
            Debug.Log("Find right hand");
        }
        else
        {
            if (rightHands.Count == 0)
                Debug.Log("No right hand can be found");
            else if (rightHands.Count > 1)
                Debug.Log("Find more than one right hands!");
        }
    }

    // Update is called once per frame
    void Update()
    {
        // 从共享内存中读取双目图像，在VR中显示
        __SharedMem.ReadImages();
        if (__SharedMem.Images.Count == 2)
        {
            //以原比例显示图像
            __Texture.LoadImage(__SharedMem.Images[0]);
            LeftEye.texture = __Texture;

            __Texture.LoadImage(__SharedMem.Images[1]);
            RightEye.texture = __Texture;

            Debug.Log(String.Format("Refresh times = {0}", ++__RefCount));
        }

        // 从头盔和手柄中读取位置和姿态数据
        int Begin = 0;
        byte[] PosGesture = new byte[128];

        Vector3 Position;
        Quaternion Rotation;
        headSet.TryGetFeatureValue(CommonUsages.devicePosition, out Position);
        headSet.TryGetFeatureValue(CommonUsages.deviceRotation, out Rotation);
        Begin = Vector3ToBytes(Position, PosGesture, Begin);
        Begin = QuaternionToBytes(Rotation, PosGesture, Begin);

        Vector3 PositionL;
        Quaternion RotationL;
        leftHand.TryGetFeatureValue(CommonUsages.devicePosition, out PositionL);
        leftHand.TryGetFeatureValue(CommonUsages.deviceRotation, out RotationL);
        Begin = Vector3ToBytes(PositionL, PosGesture, Begin);
        Begin = QuaternionToBytes(RotationL, PosGesture, Begin);

        Vector3 PositionR;
        Quaternion RotationR;
        rightHand.TryGetFeatureValue(CommonUsages.devicePosition, out PositionR);
        rightHand.TryGetFeatureValue(CommonUsages.deviceRotation, out RotationR);
        Begin = Vector3ToBytes(PositionR, PosGesture, Begin);
        Begin = QuaternionToBytes(RotationR, PosGesture, Begin);

        // 位姿数据写入共享内存
        __SharedMem.WriteBytes(PosGesture);
    }

    int Vector3ToBytes(Vector3 V, byte[] Bytes, int Begin)
    {
        byte[] bytes = BitConverter.GetBytes(V.x);
        Bytes[Begin] = bytes[0];
        Bytes[Begin + 1] = bytes[1];
        Bytes[Begin + 2] = bytes[2];
        Bytes[Begin + 3] = bytes[3];

        bytes = BitConverter.GetBytes(V.y);
        Bytes[Begin + 4] = bytes[0];
        Bytes[Begin + 5] = bytes[1];
        Bytes[Begin + 6] = bytes[2];
        Bytes[Begin + 7] = bytes[3];

        bytes = BitConverter.GetBytes(V.z);
        Bytes[Begin + 8] = bytes[0];
        Bytes[Begin + 9] = bytes[1];
        Bytes[Begin + 10] = bytes[2];
        Bytes[Begin + 11] = bytes[3];

        return Begin + 12;
    }

    int QuaternionToBytes(Quaternion V, byte[] Bytes, int Begin)
    {
        byte[] bytes = BitConverter.GetBytes(V.x);
        Bytes[Begin] = bytes[0];
        Bytes[Begin + 1] = bytes[1];
        Bytes[Begin + 2] = bytes[2];
        Bytes[Begin + 3] = bytes[3];

        bytes = BitConverter.GetBytes(V.y);
        Bytes[Begin + 4] = bytes[0];
        Bytes[Begin + 5] = bytes[1];
        Bytes[Begin + 6] = bytes[2];
        Bytes[Begin + 7] = bytes[3];

        bytes = BitConverter.GetBytes(V.z);
        Bytes[Begin + 8] = bytes[0];
        Bytes[Begin + 9] = bytes[1];
        Bytes[Begin + 10] = bytes[2];
        Bytes[Begin + 11] = bytes[3];

        bytes = BitConverter.GetBytes(V.w);
        Bytes[Begin + 12] = bytes[0];
        Bytes[Begin + 13] = bytes[1];
        Bytes[Begin + 14] = bytes[2];
        Bytes[Begin + 15] = bytes[3];

        return Begin + 16;
    }

    //回收内存的协程
    IEnumerator ToDestoryThis(Texture2D thisSprite)
    {
        yield return new WaitForSeconds(0.1f);
        //Destroy(thisSprite);
        Resources.UnloadUnusedAssets();
        System.GC.Collect();
    }
}
