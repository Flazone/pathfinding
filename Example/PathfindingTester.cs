using System;
using System.Diagnostics;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

public class PathfindingTester : MonoBehaviour
{
    private Grid _grid;
    [SerializeField] private Grid.GridSettings _gridSettings;
    
    [SerializeField] private Transform _startPosition;
    [SerializeField] private Transform _targetPosition;
    [SerializeField] private LayerMask _obstacleLayerMask;

    private NativeHashSet<int2> _obstacles;
    private NativeList<int2> _calculatedPath;
    private Stopwatch _stopwatch;

    private bool _repainted;

    private void Start()
    {
        _grid = new Grid(_gridSettings);
        _stopwatch = new Stopwatch();
        
        GetComponent<GridDebug>().SetGrid(_grid);
        
        _repainted = true;
    }

    private void Update()
    {
        // Waiting for the gizmos to repaint before disposing
        if (!_repainted) 
            return;
        _repainted = false;
        
        _obstacles = GetObstacles();
        
        _stopwatch.Restart();
        TestPathfinding();
        _stopwatch.Stop();

        Debug.Log((_stopwatch.Elapsed.TotalMilliseconds + $"ms"));
    }

    private void OnDestroy()
    {
        _obstacles.Dispose();
        _calculatedPath.Dispose();
    }

    
    /// <summary>
    /// In case of real usage, instead of scheduling and completing the job right away,
    /// a system could enqueue all the path requests and schedule them in parallel -
    /// taking advantage of the Job system.
    /// </summary>
    private void TestPathfinding()
    {
        var startCoord = _grid.WorldPositionToCell(_startPosition.position).Coord;
        Pathfinding.Node startNode = new Pathfinding.Node
        {
            Coord = new int2(startCoord.x, startCoord.y), 
            G = int.MaxValue, 
            H = int.MaxValue
        };
        
        var endCoord = _grid.WorldPositionToCell(_targetPosition.position).Coord;
        Pathfinding.Node endNode = new Pathfinding.Node
        {
            Coord = new int2(endCoord.x, endCoord.y), 
            G = int.MaxValue, 
            H = int.MaxValue
        };
        
        _calculatedPath.Dispose();
       
        var pathJob = Pathfinding.FindPath(startNode, endNode, new int2(_grid.Width, _grid.Height), _obstacles, ref _calculatedPath);
        pathJob.Schedule().Complete();
    }

    /// <summary>
    /// Grossly get all the obstacles in the scene and add them to the obstacles list
    /// </summary>
    private NativeHashSet<int2> GetObstacles()
    {
        _obstacles.Dispose();
        _obstacles = new NativeHashSet<int2>(100, Allocator.TempJob);
        
        for (int x = 0; x < _grid.Width; x++)
        {
            for (int y = 0; y < _grid.Height; y++)
            {
                Cell cell = _grid.GetCell(x, y);
                var pos = _grid.CellToWorldPosition(cell);
                
                if (Physics.Raycast(pos + Vector3.up * 100, Vector3.down, out var hit, 200, _obstacleLayerMask))
                {
                    var hitGO = hit.transform.gameObject;

                    if (hitGO.layer == LayerMask.NameToLayer("Water"))
                    {
                        _obstacles.Add(new int2(cell.X, cell.Y));
                    }
                }
            }
        }

        return _obstacles;
    }
    
    public void OnDrawGizmos()
    {
        if (_grid == null)
            return;

        if (!_obstacles.IsCreated)
            return;
        
        var cellSize = _grid.CellSize;

        foreach (int2 obstacle in _obstacles)
        {
            var obstaclePos = _grid.CellToWorldPosition(obstacle.x, obstacle.y);
            
            Gizmos.color = Color.red;
            Gizmos.DrawCube(obstaclePos + Vector3.up * 0.05f, new Vector3(cellSize,0,cellSize)); 
            Gizmos.DrawWireCube(obstaclePos, new Vector3(cellSize,0,cellSize));
        }
        
        Gizmos.color = Color.yellow;
        var cellPos = _grid.CellToWorldPosition(_grid.WorldPositionToCell(_startPosition.position));
        Gizmos.DrawCube(cellPos + Vector3.up * 0.05f, new Vector3(cellSize, 0, cellSize)); 
        
        cellPos = _grid.CellToWorldPosition(_grid.WorldPositionToCell(_targetPosition.position));
        Gizmos.DrawCube(cellPos + Vector3.up * 0.05f, new Vector3(cellSize, 0, cellSize));

        Gizmos.color = Color.cyan;

        foreach (int2 node in _calculatedPath)
        {
            cellPos = _grid.CellToWorldPosition(node.x, node.y);
            Gizmos.DrawCube(cellPos + Vector3.up * 0.05f, new Vector3(cellSize, 0, cellSize));
        }

        _repainted = true;
    }
}