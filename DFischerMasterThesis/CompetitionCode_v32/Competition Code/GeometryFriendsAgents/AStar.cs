using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class AStar
    {
        List<int[]> closedList;
        List<int[]> openList;
        int[] cameFrom;
        int start;
        int goal;

        int completeDistance;

        bool output = false;

        public AStar(int start,int goal)
        {
            completeDistance = int.MaxValue;
            //Set start and goal node-index
            this.start = start;
            this.goal = goal;
            //List of nodes evaluated
            closedList = new List<int[]>();
            //The set of tentative nodes to be evaluated, initially containing the start node
            openList = new List<int[]>();
            //Array where the node-index came from: cameFrom[7] returns the index of the predecessor of node with index 7
            cameFrom = new int[SquareAgent.nodes.Count];
            for (int i = 0; i < SquareAgent.nodes.Count; i++)
            {
                cameFrom[i] = -1;
            }
            // Cost from start along best known path until current node
            int gScore = 0;
            // Estimated total cost from start to goal
            int fScore = gScore + HeuristicValue(start,goal);
            //Add start
            //An A* Node is an array with 3 values: node index, fScore value and gScore value
            openList.Add(new int[] {start, fScore, gScore});
        }

        //returns the euclidean distance between two nodes
        private int HeuristicValue(int start,int goal)
        {
 	        Node n1 = SquareAgent.nodes[start];
            Node n2 = SquareAgent.nodes[goal];
            int distance = (int)Math.Sqrt(Math.Pow(n1.getX() - n2.getX(), 2) + Math.Pow(n1.getY() - n2.getY(), 2));
            return distance;
        }

        public Queue<Node> Run()
        {
            //while openList is not empty
            while(openList.Count > 0) 
            {
                //Search for node in openList with lowsest fScore
                int[] current = LowestFScore();
                //If current is goal return route
                if(current[0] == goal)
                {
                    completeDistance = current[1];
                    return Route(current[0]);
                }
                //Add current to closedList
                closedList.Add(current);
                //Get all neighbors of current
                List<int> neighbors = Neighbors(current[0]);
                foreach (int neighbor in neighbors)
	            {
                    //if neighbor in closedList continue
                    if (GetOfList(closedList, neighbor) != null)
                    {
                        continue;
                    }
                    // calc gScore
                    int tentativeGScore = current[2]  + SquareAgent.directDistanceMap[current[0], neighbor];
                    //get neighbor in openList if exists
                    int[] neighborInList = GetOfList(openList, neighbor);
                    //if neighbor is not in openList or the neighbor in the openList has a higher gScore
                    if(neighborInList == null || (neighborInList != null && tentativeGScore < neighborInList[2]))
                    {
                        //Set new cameFrom predecessor
                        cameFrom[neighbor] = current[0];
                        int gScore = tentativeGScore;
                        //calc new fScore
                        int fScore = gScore + HeuristicValue(neighbor, goal);
                        //if neighbor is not in openList, add new node
                        if (neighborInList == null)
                        {
                            openList.Add(new int[] {neighbor, fScore, gScore});
                        }
                        //else replace the fScore and gScore of neighbor in openList
                        else
                        {
                            ReplaceFGInOpenList(neighbor, fScore, gScore);
                        }
                    }
	            }
            }
            if(output)
            {
                System.Diagnostics.Debug.WriteLine("AStar - no route found");
            }          
            //no route found
            return null;
        }

        //replaces fScore and gScore of neighbor in openList
        private void ReplaceFGInOpenList(int neighbor, int fScore, int gScore)
        {
            foreach (int[] node in openList)
            {
                if (node[0] == neighbor)
                {
                    node[1] = fScore;
                    node[2] = gScore;
                    break;
                }
            }
        }

        //returns neighbor node or null if it not exists in list
        private int[] GetOfList(List<int[]> list, int neighbor)
        {
            foreach (int[] node in list)
            {
                if (node[0] == neighbor)
                {
                    return node;
                }
            }
            return null;
        }

        //returns all neighbor indices of current
        private List<int> Neighbors(int current)
        {
            List<int> neighbors = new List<int>();
            for (int i = 0; i < SquareAgent.nodes.Count; i++)
            {
                if(current != i)
                {
                    if(SquareAgent.adjacencyMatrix[current, i] > 0)
                    {
                        neighbors.Add(i);
                    }
                }
            }
            return neighbors;
        }

        private Queue<Node> Route(int current)
        {
            Queue<Node> route = new Queue<Node>();
            List<int> routeReversed = new List<int>();
            //create a reversed route
            routeReversed.Add(current);
            while(cameFrom[current] != -1)
            {
                current = cameFrom[current];
                routeReversed.Add(current);
            }
            //create the correct route
            for (int i = routeReversed.Count-1; i >= 0; i--)
            {
                route.Enqueue(SquareAgent.nodes[routeReversed[i]]);
                if (output)
                {
                    System.Diagnostics.Debug.WriteLine("AStar - route found:" + routeReversed[i]);
                }
            }
            return route;
        }

        private int[] LowestFScore()
        {   // Searches for the lowest fScore in openList
            int[] lowest = openList[0];
            int index = 0;
            for (int i = 1; i < openList.Count; i++)
			{
			    if(lowest[1] > (openList[i])[1])
                {
                    lowest = openList[i];
                    index = i;
                }
			}
            //removes the node with the lowest fScore
            openList.RemoveAt(index);
 	        return lowest;
        }

        public int GetCompleteDistance()
        {
            return completeDistance;
        }
    }
}
