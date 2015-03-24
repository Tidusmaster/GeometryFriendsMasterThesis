//  Author(s):
//  João Catarino <joaopereiracatarino@gmail.com

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class Moves
    {
        //No action
        public const int NO_ACTION = 0;

        //Circle Actions
        public static int ROLL_LEFT = 1;
        public static int ROLL_RIGHT = 2;
        public static int JUMP = 3;
        public static int GROW = 4;

        //Square Actions
        public static int MOVE_LEFT = 5;
        public static int MOVE_RIGHT = 6;
        public static int MORPH_UP = 7;
        public static int MORPH_DOWN = 8;
    }
}
