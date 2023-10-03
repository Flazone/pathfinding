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
    
    public static NativeList<int2> FindPath(Node startNode, Node endNode, int2 dimensions, HashSet<int2> obstacles)
    {
        NativeHashMap<int2, Node> nodes1 = new NativeHashMap<int2, Node>(1000, Allocator.TempJob);
        NativeHashMap<int2, Node> nodes2 = new NativeHashMap<int2, Node>(1000, Allocator.TempJob);
        NativeArray<JobHandle> jobHandleArray = new NativeArray<JobHandle>(2, Allocator.TempJob);
        
        NativeList<int2> obs = new NativeList<int2>(obstacles.Count, Allocator.TempJob);
        foreach (var obstacle in obstacles)
            obs.AddNoResize(obstacle);
        
        for (int i = 0; i < 2; i++) 
        {
            PathJob pathJob = new PathJob
            {
                Obstacles = obs,
                SearchedNodes = i == 0 ? nodes1 : nodes2,
            
                Dims = dimensions,
                StartNode = i == 0 ? startNode : endNode,
                EndNode = i == 0 ? endNode : startNode,
            };
            jobHandleArray[i] = pathJob.Schedule();
        }
        JobHandle.CompleteAll(jobHandleArray);
        
        NativeList<int2> calculatedPath = new NativeList<int2>(30,Allocator.Persistent);
        calculatedPath.AddNoResize(startNode.Coord);
        
        // End node reached, calculate path
        if (nodes1.ContainsKey(endNode.Coord))// && nodes2.ContainsKey(startNode.Coord))
        {
            ReconstructPath(endNode.Coord, startNode.Coord, nodes1, ref calculatedPath);
        } 
        // Target nodes never reached
        // Calculating the closest node meeting point
        else if (!nodes1.IsEmpty)
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
            
            ReconstructPath(result, startNode.Coord, nodes1, ref calculatedPath);
            calculatedPath.AddNoResize(result);
        }

        nodes1.Dispose();
        nodes2.Dispose();
        obs.Dispose();
        jobHandleArray.Dispose();

        return calculatedPath;
    }

    private static void ReconstructPath(int2 endNodeCoord, int2 startNodeCoord, NativeHashMap<int2, Node> nodes, ref NativeList<int2> calculatedPath)
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
    
    private static float SqrDistance(int2 coordA, int2 coordB)
    {
        int a = coordB.x - coordA.x;
        int b = coordB.y - coordA.y;
        return Mathf.Sqrt(a * a + b * b);
    }
    
    [BurstCompile(CompileSynchronously = true)]
    private struct PathJob : IJob
    {
        [ReadOnly] public NativeList<int2> Obstacles;
        public NativeHashMap<int2, Node> SearchedNodes;
        [ReadOnly] public int2 Dims;
        
        [ReadOnly] public Node StartNode;
        [ReadOnly] public Node EndNode;

        public void Execute()
        {
            NativeHeap<Node, Min> minSet = new NativeHeap<Node, Min>(Allocator.Temp);
            NativeHashMap<int2, Node> openSet = new NativeHashMap<int2, Node>(1000, Allocator.Temp);

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

                if (++counter > 500)
                    break;
            }
        }
    }
    
    private struct Min : IComparer<Node>
    {
        public int Compare(Node x, Node y)
        {
            return x.F.CompareTo(y.F);
        }
    }
}