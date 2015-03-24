using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class Node
    {
        private int x = 0;
        private int y = 0;
        private bool diamond = false;
        private bool leadsToFallDown = false;
        private bool pseudo = false;

        public Node(int x, int y, bool diamond)
        {
            this.x = x;
            this.y = y;
            this.diamond = diamond;
        }

        public override bool Equals(Object obj)
        {
            // Check for null values and compare run-time types.
            if (obj == null || GetType() != obj.GetType())
                return false;

            Node n = (Node)obj;
            return (x == n.x) && (y == n.y) && (diamond == n.diamond);
        }

        public override String ToString()
        {
            return "X: " + x + " Y: " + y + " Diamond: " + diamond;
        }

        public int getX()
        {
            return this.x;
        }
        public void setX(int x)
        {
            this.x = x;
        }

        public int getY()
        {
            return this.y;
        }

        public void setY(int y)
        {
            this.y = y;
        }

        public bool getDiamond()
        {
            return this.diamond;
        }

        public void setDiamond(bool diamond)
        {
            this.diamond = diamond;
        }

        public bool getLeadsToFallDown()
        {
            return leadsToFallDown;
        }

        public void setLeadsToFallDown(bool fallDown)
        {
            this.leadsToFallDown = fallDown;
        }

        public bool getPseudo()
        {
            return pseudo;
        }

        public void setPseudo(bool pseudo)
        {
            this.pseudo = pseudo;
        }

    }
}
