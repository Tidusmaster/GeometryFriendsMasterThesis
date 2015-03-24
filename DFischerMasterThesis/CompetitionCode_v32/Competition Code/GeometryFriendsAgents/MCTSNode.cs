using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class MCTSNode
    {
        static Random r = new Random();
        static double epsilon = 0.01;

        public int nodeIndex = 0;

        public MCTSNode parent;
        public MCTSNode[] children;
        public double nVisits, totValue;

        public int collected;
        public bool diamond;
        public bool endState;

        bool output = false;

        public void selectAction() {
            LinkedList<MCTSNode> visited = new LinkedList<MCTSNode>();
            MCTS.visitedIndex = new List<int>();
            MCTSNode cur = this;
            visited.AddLast(this);
            while (!cur.isLeaf()) {
                if (cur.select()== null)
                {
                    return;
                }
                cur = cur.select();
                AddToIndexList(cur);
                visited.AddLast(cur);     
            }
            if(cur.endState)
            {
                return;
            }
            cur.expand();
            MCTSNode newNode = cur.select();
            if(newNode == null)
            {
                if (output)
                {
                    System.Diagnostics.Debug.WriteLine("new Node null error");
                }
                return;
            }
            AddToIndexList(newNode);
            visited.AddLast(newNode);         
            double value = rollOut(newNode);
            foreach (MCTSNode node in visited)
	        {
                node.updateStats(value);
	        }
        }

        public void expand()
        {
            //use adjacancy matrix with y = nodeIndex and find all children
            List<int> childIndex = new List<int>();
            for (int i = 0; i < MCTS.nodes.Count; i++)
            {
                if(MCTS.adjacencyMatrix[nodeIndex, i] != 0)
                {
                    childIndex.Add(i);
                }
            }
            children = new MCTSNode[childIndex.Count];
            for (int i = 0; i < childIndex.Count; i++)
            {
                MCTSNode child = new MCTSNode();
                child.parent = this;
                child.nodeIndex = childIndex[i];
                child.diamond = MCTS.nodes[child.nodeIndex].getDiamond();             
                child.collected = collected;
                if (child.diamond)
                {
                    if (!MCTS.visitedIndex.Contains(child.nodeIndex))
                    {
                        child.collected++;
                        if (child.collected == MCTS.diamonds)
                        {
                            child.endState = true;
                        }
                    }
                }
                
                children[i] = child;
            }
        }

        private MCTSNode select() {
            MCTSNode selected = null;
            double bestValue = Double.MinValue;
            foreach (MCTSNode c in children) {
                //Maybe change C (exploration parameter, sqrt2 or 1/sqrt2 ?)
                //With higher C explore more new nodes, with lower C exploit existing nodes
                double uctValue =
                        (c.totValue / (c.nVisits + epsilon)) +
                                Math.Sqrt(Math.Log(nVisits+1) / (c.nVisits + epsilon)) +
                                (r.NextDouble() * epsilon);
                if (uctValue > bestValue) {
                    selected = c;
                    bestValue = uctValue;
                }
            }
            // System.out.println("Returning: " + selected);
            return selected;
        }

        public bool isLeaf()
        {
            return children == null;
        }

        public double rollOut(MCTSNode mn)
        { 
            double value = 0;
            int counter = 1;
            while(!mn.endState && counter < (MCTS.nodes.Count*10))
            {
                mn.expand();
                MCTSNode randomNode = mn.select();
                AddToIndexList(randomNode);          
                if (randomNode == null)
                {
                    break;
                }
                mn = randomNode;          
                counter++;
            }
            if(mn.endState)
            {
                if (output)
                {
                    System.Diagnostics.Debug.WriteLine("End state found: " + mn);
                }
                //Stop if first endstate found?
                //MCTS.endStateFound = true;
            }
            value = (mn.collected +0.01)* 100 * 1000 / (counter-1);

            if(MCTS.bestCollected < mn.collected || (MCTS.bestValue < value && MCTS.bestCollected == mn.collected))
            {
                MCTS.SetBestRoute(mn);
                MCTS.bestCollected = mn.collected;
                MCTS.bestValue = value;
            }

            return value;

        }

        public void updateStats(double value)
        {
            nVisits++;
            totValue += value;
        }

        public bool childrenEmpty()
        {
            return children == null || children.Length == 0;
        }

        public void AddToIndexList(MCTSNode cur)
        {

            if (cur != null && cur.diamond)
                {
                    if (!MCTS.visitedIndex.Contains(cur.nodeIndex))
                    {
                        MCTS.visitedIndex.Add(cur.nodeIndex);
                    }
                }
        }

        public override String ToString()
        {
            return "Node index: " + nodeIndex + " nVisits: " + nVisits + " Value: " + totValue+ " Collected: "+ collected+ " End state: "+endState+ " Diamond: "+diamond;
        }

    }
}
