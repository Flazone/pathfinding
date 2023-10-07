using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public class Pathfinding
{
    public struct Node
    {
        public int2 Coord;
        public int2 Parent;
        public float G;
        public float H;
        public float F => G + H;
    }

    public static PathJob FindPath(Node startNode, Node endNode, int2 dimensions, NativeHashSet<int2> obstacles, ref NativeList<int2> path)
    {
        path = new NativeList<int2>(50, Allocator.TempJob);
        
        PathJob pathJob = new PathJob
        {
            Obstacles = obstacles,
            CalculatedPath = path,

            Dimensions = dimensions,
            StartNode = startNode,
            EndNode = endNode,
        };

        return pathJob;
    }
    
    [BurstCompile(CompileSynchronously = true)]
    public struct PathJob : IJob
    {
        [ReadOnly] public NativeHashSet<int2> Obstacles;
        public NativeList<int2> CalculatedPath;
        
        [ReadOnly] public int2 Dimensions;
        public Node StartNode;
        [ReadOnly] public Node EndNode;


        public void Execute()
        {
            CalculatedPath.Clear();

            // if start node is in an obstacle, return an empty path
            if (Obstacles.Contains(StartNode.Coord))
            {
                CalculatedPath.AddNoResize(StartNode.Coord);
                return;
            }         
            
            NativeHeap<Node, Min> minSet = new NativeHeap<Node, Min>(Allocator.Temp);
            NativeHashMap<int2, Node> openSet = new NativeHashMap<int2, Node>(1000, Allocator.Temp);
            NativeHashMap<int2, Node> exploredNodes = new NativeHashMap<int2, Node>(512, Allocator.Temp);

            Node currentNode = StartNode;
            StartNode.G = 0;
            StartNode.H = SqrDistance(StartNode.Coord, EndNode.Coord);
            minSet.Insert(StartNode);

            var offsets = new NativeArray<int2>(8, Allocator.Temp);
            offsets[0] = new int2(0, 1);
            offsets[1] = new int2(1, 1);
            offsets[2] = new int2(1, 0);
            offsets[3] = new int2(1, -1);
            offsets[4] = new int2(0, -1);
            offsets[5] = new int2(-1, -1);
            offsets[6] = new int2(-1, 0);
            offsets[7] = new int2(-1, 1);

            int counter = 0;

            // No more node to explore OR reached our goal
            while (minSet.Count != 0 && !currentNode.Coord.Equals(EndNode.Coord))
            {
                currentNode = minSet.Peek();

                for (int i = 0; i < offsets.Length; i++)
                {
                    var neighbourCoord = currentNode.Coord + offsets[i];

                    // Make sure the node isn't outside the grid
                    if (neighbourCoord.x < 0 || neighbourCoord.x >= Dimensions.x || neighbourCoord.y < 0 ||
                        neighbourCoord.y >= Dimensions.y)
                        continue;

                    // Continue if neighbour is an obstacle or an already searched node
                    if (Obstacles.Contains(neighbourCoord) || exploredNodes.ContainsKey(neighbourCoord))
                        continue;

                    // Avoid diagonal movement if there are obstacles on both sides
                    // Preventing the path from passing between two obstacles
                    bool diagonal = offsets[i].x != 0 && offsets[i].y != 0;
                    if (diagonal)
                    {
                        int xOffset = currentNode.Coord.x + offsets[i].x;
                        int yOffset = currentNode.Coord.y + offsets[i].y;

                        if (Obstacles.Contains(new int2(xOffset, currentNode.Coord.y)) &&
                            Obstacles.Contains(new int2(currentNode.Coord.x, yOffset)))
                        {
                            continue;
                        }
                    }

                    Node neighbour = new Node
                    {
                        Coord = neighbourCoord,
                        Parent = currentNode.Coord,
                        G = currentNode.G + SqrDistance(currentNode.Coord, neighbourCoord),
                    };

                    // if neighbour already open, but with an higher score than now 
                    if (openSet.ContainsKey(neighbour.Coord))
                    {
                        if (neighbour.G < openSet[neighbour.Coord].G)
                        {
                            openSet[neighbour.Coord] = neighbour;
                        }
                    }
                    else
                    {
                        neighbour.H = SqrDistance(neighbourCoord, EndNode.Coord);
                        openSet.Add(neighbour.Coord, neighbour);
                        minSet.Insert(neighbour);
                    }
                }

                // Remove the node we came from
                minSet.Pop();
                exploredNodes.TryAdd(currentNode.Coord, currentNode);

                if (++counter > 1000)
                    break;
            }
            
            
            // End node reached, calculate path
            if (exploredNodes.ContainsKey(EndNode.Coord)) // && nodes2.ContainsKey(startNode.Coord))
            {
                ReconstructPath(EndNode.Coord, StartNode.Coord, exploredNodes, ref CalculatedPath);
            }
            // Target node never reached
            // Calculating a fallback path to the closest node
            else
            {
                int2 closestNodeCoord = int2.zero;
                float bestScore = float.MaxValue;
          
                float hWeight = 1.0f;
                float parentWeight = 0.5f;
            
                // Determine the best node using the H score and the path length (parent count)
                // Can be wrong sometimes, but it's a good compromise between performance and results
                // We could have a 100% accurate result by calculating an additional path, coming from the end node to the start node  
                foreach (var i in exploredNodes)
                {
                    int parentAmount = GetNodeParentAmount(i.Key, exploredNodes);
                    float score = (i.Value.H * hWeight) - (parentAmount * parentWeight);

                    if (score < bestScore)
                    {
                        closestNodeCoord = i.Key;
                        bestScore = score;
                    }
                }

                ReconstructPath(closestNodeCoord, StartNode.Coord, exploredNodes, ref CalculatedPath);
                CalculatedPath.AddNoResize(closestNodeCoord);
            }
        }
        
        
        private void ReconstructPath(int2 endNodeCoord, int2 startNodeCoord, NativeHashMap<int2, Node> nodes, ref NativeList<int2> calculatedPath)
        {
            calculatedPath.Clear();
            int2 currentCoord = endNodeCoord;

            if (nodes.Count != 0)
            {
                while (!currentCoord.Equals(startNodeCoord))
                {
                    currentCoord = nodes[currentCoord].Parent;
                    calculatedPath.AddNoResize(currentCoord);
                }
            }

            calculatedPath.AddNoResize(endNodeCoord);

            // Reverse the path in-place
            for (int i = 0, j = calculatedPath.Length - 1; i < j; i++, j--)
            {
                (calculatedPath[i], calculatedPath[j]) = (calculatedPath[j], calculatedPath[i]);
            }
        }
        
        private int GetNodeParentAmount(int2 nodeCoord, NativeHashMap<int2, Node> exploredNodes)
        {
            int count = 0;
            int2 currentCoord = nodeCoord;

            while (!currentCoord.Equals(int2.zero) && exploredNodes.ContainsKey(currentCoord))
            {
                currentCoord = exploredNodes[currentCoord].Parent;
                count++;
            }

            return count;
        }
        
        private float SqrDistance(int2 coordA, int2 coordB)
        {
            int a = coordB.x - coordA.x;
            int b = coordB.y - coordA.y;
            return Mathf.Sqrt(a * a + b * b);
        }
        
        private struct Min : IComparer<Node>
        {
            public int Compare(Node x, Node y)
            {
                return x.F.CompareTo(y.F);
            }
        }
    }
}