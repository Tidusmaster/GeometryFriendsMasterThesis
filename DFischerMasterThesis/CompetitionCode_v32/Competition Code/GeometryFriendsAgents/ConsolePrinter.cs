using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    public class ConsolePrinter
    {

        public ConsolePrinter() 
        {
        
        }

        public static void Print(string str)
        {
            System.Diagnostics.Debug.Write(str);
        }

        public static void PrintLine(string str)
        {
            System.Diagnostics.Debug.WriteLine("["+DateTime.Now+"] " + str);
        }



    }
}
