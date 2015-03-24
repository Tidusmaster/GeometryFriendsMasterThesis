using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class SubgoalAStarNode
    {

        public int nodeIndex;
        public int fScore;
        public int gScore;
        public SubgoalAStarNode cameFrom;
        public List<int> collectedDiamonds;

        public SubgoalAStarNode(int nodeIndex, int fScore, int gScore, SubgoalAStarNode cameFrom, List<int> collectedDiamonds)
        {
            this.nodeIndex = nodeIndex;
            this.fScore = fScore;
            this.gScore = gScore;
            this.cameFrom = cameFrom;
            this.collectedDiamonds = collectedDiamonds;
        }

        public override bool Equals(Object obj)
        {
            // Check for null values and compare run-time types.
            if (obj == null || GetType() != obj.GetType())
                return false;

            SubgoalAStarNode n = (SubgoalAStarNode)obj;
            if(nodeIndex != n.nodeIndex)
            {
                return false;
            }
            if(collectedDiamonds.Count != n.collectedDiamonds.Count)
            {
                return false;
            }
            for (int i = 0; i < collectedDiamonds.Count; i++)
            {
                if(collectedDiamonds[i] != n.collectedDiamonds[i])
                {
                    return false;
                }
            }
            return true;
        }

        public override String ToString()
        {
            return "Index: " + nodeIndex + " fScore: " + fScore + " gScore: " + gScore + " cameFrom: (" + cameFrom + ") collectedDiamonds: " + string.Join(",", collectedDiamonds.ToArray());
        }
    }
}
