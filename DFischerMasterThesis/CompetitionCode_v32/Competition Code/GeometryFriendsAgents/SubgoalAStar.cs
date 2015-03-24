using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeometryFriendsAgents
{
    class SubgoalAStar
    {
        List<SubgoalAStarNode> closedList;
        List<SubgoalAStarNode> openList;
        int start;
        List<int> goals;
        int numberOfDiamonds;

        int timeToLowerNoD;

        bool output = false;

        public SubgoalAStar(int start, List<int> goals, int time, int diamonds)
        {
            //Set start and goal node-index
            this.start = start;
            this.goals = goals;
            this.timeToLowerNoD = time;
            numberOfDiamonds = goals.Count;
            if(diamonds > 0)
            {
                numberOfDiamonds = diamonds;
            }
            //List of nodes evaluated
            closedList = new List<SubgoalAStarNode>();
            //The set of tentative nodes to be evaluated, initially containing the start node
            openList = new List<SubgoalAStarNode>();
            // Cost from start along best known path until current node
            int gScore = 0;
            // Estimated total cost from start to goal
            int fScore = gScore; // + HeuristicValue(start,goal);
            //Add start
            //Create SubgoalAStarNode with node index, fScore value, gScore value, cameFrom node and the state of collected diamonds
            SubgoalAStarNode startNode = new SubgoalAStarNode(start, fScore, gScore, null, new List<int>() );
            openList.Add(startNode);
        }

        //returns the euclidean distance between two nodes
        //private int HeuristicValue(int start,int goal)
        //{
        //    Node n1 = SquareAgent.nodes[start];
        //    Node n2 = SquareAgent.nodes[goal];
        //    int distance = (int)Math.Sqrt(Math.Pow(n1.getX() - n2.getX(), 2) + Math.Pow(n1.getY() - n2.getY(), 2));
        //    return distance;
        //}

        public Queue<Node> Run()
        {
            System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
            //while openList is not empty
            while(openList.Count > 0) 
            {
                //Lower diamonds to collected by one if time elapsed
                if(sw.ElapsedMilliseconds > timeToLowerNoD)
                {
                    numberOfDiamonds--;
                    sw = System.Diagnostics.Stopwatch.StartNew();
                }
                //Search for node in openList with lowsest fScore
                SubgoalAStarNode current = LowestFScore();
                //If current is goal and the number of collected diamonds is equal to the given number of diamonds to collect the route is returned
                if(goals.Contains(current.nodeIndex))
                {
                    if(numberOfDiamonds == current.collectedDiamonds.Count)
                    {
                        return Route(current);
                    } 
                }
                //Add current to closedList
                closedList.Add(current);
                //Get all neighbors of current
                List<int> neighbors = Neighbors(current.nodeIndex);
                foreach (int neighbor in neighbors)
	            {
                    //if neighbor in closedList continue
                    if (GetOfList(closedList, neighbor, current) != null)
                    {
                        continue;
                    }
                    // calc gScore
                    int tentativeGScore = current.gScore  + SquareAgent.directDistanceMap[current.nodeIndex, neighbor];
                    //get neighbor in openList if exists
                    SubgoalAStarNode neighborInList = GetOfList(openList, neighbor, current);
                    //if neighbor is not in openList or the neighbor in the openList has a higher gScore
                    if(neighborInList == null || (neighborInList != null && tentativeGScore < neighborInList.gScore))
                    {
                        int gScore = tentativeGScore;
                        //calc new fScore
                        int fScore = gScore; // +HeuristicValue(neighbor, goal);
                        //if neighbor is not in openList, add new node
                        if (neighborInList == null)
                        {
                            //Copy collected diamonds so far
                            List<int> collDiamonds = new List<int>(current.collectedDiamonds);
                            SubgoalAStarNode newNode = new SubgoalAStarNode(neighbor, fScore, gScore, current, collDiamonds);
                            //If neighbor is a goal and not collected yet
                            if(goals.Contains(neighbor) && !newNode.collectedDiamonds.Contains(neighbor))
                            {
                                newNode.collectedDiamonds.Add(neighbor);
                            }
                            openList.Add(newNode);
                        }
                        //else delete old neighbor and add new one in openList
                        else
                        {
                            openList.Remove(neighborInList);
                            neighborInList.fScore = fScore;
                            neighborInList.gScore = gScore;
                            neighborInList.cameFrom = current;
                            //Copy collected diamonds so far
                            List<int> collDiamonds = new List<int>(current.collectedDiamonds);
                            neighborInList.collectedDiamonds = collDiamonds;
                            //If neighbor is a goal and not collected yet
                            if (goals.Contains(neighbor) && !neighborInList.collectedDiamonds.Contains(neighbor))
                            {
                                neighborInList.collectedDiamonds.Add(neighbor);
                            }
                            openList.Add(neighborInList);
                        }
                    }
	            }
            }
            if (output)
            {
                System.Diagnostics.Debug.WriteLine("Subgoal AStar - no route found");
            }
            //no route found
            return null;
        }

        //returns neighbor node or null if it not exists in list
        private SubgoalAStarNode GetOfList(List<SubgoalAStarNode> list, int neighbor, SubgoalAStarNode current)
        {
            foreach (SubgoalAStarNode node in list)
            {
                if (node.nodeIndex == neighbor)
                {
                    bool wrongOrder = false;
                    if (node.collectedDiamonds.Count != current.collectedDiamonds.Count)
                    {
                        continue;
                    }
                    for (int i = 0; i < node.collectedDiamonds.Count; i++)
                    {
                        if (node.collectedDiamonds[i] != current.collectedDiamonds[i])
                        {
                            wrongOrder = true ;
                            break;
                        }
                    }
                    if(wrongOrder)
                    {
                        continue;
                    }
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

        private Queue<Node> Route(SubgoalAStarNode current)
        {
            Queue<Node> route = new Queue<Node>();
            List<int> routeReversed = new List<int>();
            //create a reversed route
            routeReversed.Add(current.nodeIndex);
            while(current.cameFrom != null)
            {
                current = current.cameFrom;
                routeReversed.Add(current.nodeIndex);
            }
            //create the correct route
            for (int i = routeReversed.Count-1; i >= 0; i--)
            {
                route.Enqueue(SquareAgent.nodes[routeReversed[i]]);
                if (output)
                {
                    System.Diagnostics.Debug.WriteLine("Subgoal AStar - route found:" + routeReversed[i]);
                }
            }
            return route;
        }

        private SubgoalAStarNode LowestFScore()
        {   // Searches for the lowest fScore in openList
            SubgoalAStarNode lowest = openList[0];
            int index = 0;
            for (int i = 1; i < openList.Count; i++)
			{
			    if(lowest.fScore > openList[i].fScore)
                {
                    lowest = openList[i];
                    index = i;
                }
			}
            //removes the node with the lowest fScore
            openList.RemoveAt(index);
 	        return lowest;
        }
    }
}
