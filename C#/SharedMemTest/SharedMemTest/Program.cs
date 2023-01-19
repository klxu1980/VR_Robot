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
            CSharedMemory mem = new CSharedMemory("shared_memory1", 1024 * 1024 * 3 * 2);
            while(true)
            {
                mem.ReadImages();
                Thread.Sleep(200);
            }
        }
    }
}
