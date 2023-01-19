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

        public CSharedMemory(String MemName, int MemSize, bool OpenRead=true)
        {
            if(OpenRead)
            {
                var mmf = MemoryMappedFile.OpenExisting(MemName);
                __viewAccessor = mmf.CreateViewAccessor(0, MemSize);
                __mutex = new Mutex(false, MemName + "_mutex");
            }
            Images = new List<CImage>();
        }

        public void ReadImages()
        {
            __mutex.WaitOne();

            int Offset = 0;
            Images.Clear();
            while(true)
            {
                int[] ImgShape = new int[3];
                __viewAccessor.ReadArray<int>(Offset, ImgShape, 0, 3);
                int ImgSize = ImgShape[0] * ImgShape[1] * ImgShape[2];
                if (ImgSize == 0)
                    break;

                byte[] ImgBuffer = new byte[ImgSize];
                __viewAccessor.ReadArray<byte>(Offset + 12, ImgBuffer, 0, ImgSize);

                Images.Add(new CImage(ImgShape, ImgBuffer));                
                Offset += ImgSize + 12;
            }
            __mutex.ReleaseMutex();

            Console.WriteLine(String.Format("Image count = {0}", Images.Count));
        }
    }
}