using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.IO.MemoryMappedFiles;
using System.Drawing.Imaging;


namespace SharedMemTest
{
    class CSharedMemory
    {
        private MemoryMappedViewAccessor __viewAccessor;
        private Mutex __mutex;
        public List<byte[]> Images;

        public CSharedMemory(String MemName, int MemSize)
        {
            var mmf = MemoryMappedFile.CreateOrOpen(MemName, MemSize);
            __viewAccessor = mmf.CreateViewAccessor(0, MemSize);
            __mutex = new Mutex(false, MemName + "_mutex");

            Images = new List<byte[]>();
        }

        public void ReadImages(int Begin = 0)
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
}