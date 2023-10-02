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
    
    public static HashSet<int2> FindPath(Node startNode, Node endNode, int2 dimensions, NativeHashSet<int2> obstacles)
    {
        NativeHashMap<int2, Node> nodes1 = new NativeHashMap<int2, Node>(1000, Allocator.TempJob);
        NativeHashMap<int2, Node> nodes2 = new NativeHashMap<int2, Node>(1000, Allocator.TempJob);
        NativeArray<JobHandle> jobHandleArray = new NativeArray<JobHandle>(2, Allocator.TempJob);

        for (int i = 0; i < 2; i++) 
        {
            PathJob pathJob = new PathJob
            {
                Obstacles = obstacles,
                SearchedNodes = i == 0 ? nodes1 : nodes2,
            
                Dims = dimensions,
                StartNode = i == 0 ? startNode : endNode,
                EndNode = i == 0 ? endNode : startNode,
            };
            jobHandleArray[i] = pathJob.Schedule();
        }

        JobHandle.CompleteAll(jobHandleArray);
        
        HashSet<int2> calculatedPath;
        
        // End node reached, calculate path
        if (nodes1.ContainsKey(endNode.Coord) && nodes2.ContainsKey(startNode.Coord))
        {
            calculatedPath = ReconstructPath(endNode.Coord, startNode.Coord, nodes1);
        } 
        // If start == end, return the start node
        else if (nodes1.IsEmpty)
        {
            calculatedPath = new HashSet<int2> {startNode.Coord};
            return calculatedPath;
        }
        // Target nodes never reached for both jobs
        // Calculating the closest node meeting point
        else
        {
            int2 result = int2.zero;
            float closestDistance = int.MaxValue;
            
            foreach (var i in nodes1)
            {
                foreach (var j in nodes2)
                {
                    var distance = SqrDistance(i.Key, j.Key);
                    if (distance < closestDistance)
                    {
                        result = i.Key;
                        closestDistance = distance;
                    }
                }
            }

            calculatedPath = ReconstructPath(result, startNode.Coord, nodes1);
            calculatedPath.Add(result);
        }

        nodes1.Dispose();
        nodes2.Dispose();
        jobHandleArray.Dispose();
        
        return calculatedPath;
    }

    private static HashSet<int2> ReconstructPath(int2 arrivalNode, int2 departureNode, NativeHashMap<int2, Node> nodes)
    {
        HashSet<int2> calculatedPath = new HashSet<int2>();
        var currentCoord = arrivalNode;

        if (nodes.Count != 0)
        {
            while (!currentCoord.Equals(departureNode))
            {
                currentCoord = nodes[currentCoord].Parent;
                calculatedPath.Add(currentCoord);
            }
        }

        calculatedPath.Add(arrivalNode);
        return calculatedPath;
    }
    
    private static float SqrDistance(int2 coordA, int2 coordB)
    {
        int a = coordB.x - coordA.x;
        int b = coordB.y - coordA.y;
        return Mathf.Sqrt(a * a + b * b);
    }

    [BurstCompile(CompileSynchronously = true)]
    private struct PathJob : IJob
    {
        [ReadOnly] public NativeHashSet<int2> Obstacles;
        public NativeHashMap<int2, Node> SearchedNodes; //TODO: use a MinHeap
        public int2 Dims;
        
        public Node StartNode;
        public Node EndNode;

        public void Execute()
        {
            NativeHashMap<int2, Node> openSet = new NativeHashMap<int2, Pathfinding.Node>(1000, Allocator.Temp);

            Node currentNode = StartNode;
            StartNode.G = 0;
            StartNode.H = SqrDistance(StartNode.Coord, EndNode.Coord);
            openSet.Add(StartNode.Coord, StartNode);
            
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
            while (!openSet.IsEmpty && !currentNode.Coord.Equals(EndNode.Coord))
            {
                currentNode = openSet[GetLowestCostNode(openSet)];
                SearchedNodes.TryAdd(currentNode.Coord, currentNode);
                
                for (int i = 0; i < offsets.Length; i++)
                {
                    var neighbourCoord = currentNode.Coord + offsets[i];

                    // Make sure the node isn't outside the grid
                    if (neighbourCoord.x < 0 || neighbourCoord.x >= Dims.x || neighbourCoord.y < 0 || neighbourCoord.y >= Dims.y)
                        continue;
                    
                    // Continue if neighbour is an obstacle or an already searched node
                    if (Obstacles.Contains(neighbourCoord) || SearchedNodes.ContainsKey(neighbourCoord))
                        continue;

                    Node neighbour = new Node
                    {
                        Coord = neighbourCoord,
                        Parent = currentNode.Coord,
                        G = currentNode.G + SqrDistance(currentNode.Coord, neighbourCoord),
                    };

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
                    }
                }
                
                openSet.Remove(currentNode.Coord);

                counter++;
                if (counter > 1000)
                {
                    break;
                }
            }

            openSet.Dispose();
            offsets.Dispose();
        }

        private int2 GetLowestCostNode(NativeHashMap<int2, Node> openSet)
        {
            Node result = new Node();
            float fCost = int.MaxValue;

            foreach (var kvp in openSet)
            {
                var node = kvp.Value;
                if (node.F < fCost)
                {
                    result = node;
                    fCost = result.F;
                }
            }

            return result.Coord;
        }
    }
}