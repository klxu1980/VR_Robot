using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SharedMemTest
{
    class Program
    {
        static void Main(string[] args)
        {
            CSharedMemory mem = new CSharedMemory("ShareForUnity1", 2208 * 1242 * 3 + 256);
            while(true)
            {
                mem.ReadImages();
                Thread.Sleep(200);
            }
        }
    }
}
