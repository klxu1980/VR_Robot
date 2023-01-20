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
    class CImage
    {
        public int[] Shape;
        public byte[] Buffer;
        public CImage(int[] Shape, byte[] Buffer)
        {
            this.Shape = Shape;
            this.Buffer = Buffer;
        }
    }

    class CSharedMemory
    {
        private MemoryMappedViewAccessor __viewAccessor;
        private Mutex __mutex;
        public List<CImage> Images;

        public CSharedMemory(String MemName, int MemSize)
        {
            var mmf = MemoryMappedFile.CreateOrOpen(MemName, MemSize);
            __viewAccessor = mmf.CreateViewAccessor(0, MemSize);
            __mutex = new Mutex(false, MemName + "_mutex");

            Images = new List<CImage>();
        }

        public void ReadImages(int Begin=0)
        {
            int Offset = Begin;
            int[] ImgCnt = new int[1];
            int[] ImgShape = new int[3];
            Images.Clear();

            __mutex.WaitOne();
            
            // 读取图像数量
            __viewAccessor.ReadArray<int>(Offset, ImgCnt, 0, 1);
            Offset += 3;

            // 依次读取图像
            for(int i = 0; i < ImgCnt[0]; ++i)
            {                
                __viewAccessor.ReadArray<int>(Offset, ImgShape, 0, 3);
                int ImgSize = ImgShape[0] * ImgShape[1] * ImgShape[2];               

                byte[] ImgBuffer = new byte[ImgSize];
                __viewAccessor.ReadArray<byte>(Offset + 12, ImgBuffer, 0, ImgSize);

                Images.Add(new CImage(ImgShape, ImgBuffer));                
                Offset += ImgSize + 12;
            }
            __mutex.ReleaseMutex();

            Console.WriteLine(String.Format("Image count = {0}", Images.Count));
        }

        public void WriteBytes(byte[] Data, int Begin=0)
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