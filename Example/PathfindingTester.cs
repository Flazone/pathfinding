using System.Collections.Generic;
using System.Diagnostics;
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

    private HashSet<int2> _obstacles;
    private HashSet<int2> _lastPath;

    private void Start()
    {
        _grid = new Grid(_gridSettings);
        GetComponent<GridDebug>().SetGrid(_grid);
    }

    private void Update()
    {
        TestPathfinding();
    }


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

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();

        _obstacles = GetObstacles();
        _lastPath = Pathfinding.FindPath(startNode, endNode, new int2(_grid.Width, _grid.Height), _obstacles);
        stopwatch.Stop();
        Debug.Log((stopwatch.Elapsed.TotalMilliseconds + $"ms"));
        
    }
    
    /// <summary>
    /// Grossly get all the obstacles in the scene and add them to the obstacles HashSet
    /// </summary>
    private HashSet<int2> GetObstacles()
    {
        _obstacles = new HashSet<int2>();
        
        for (int x = 0; x < _grid.Cells.GetLength(0); x++)
        {
            for (int y = 0; y < _grid.Cells.GetLength(1); y++)
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
        if (_grid == null || _obstacles == null)
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

        foreach (int2 node in _lastPath)
        {
            cellPos = _grid.CellToWorldPosition(node.x, node.y);
            Gizmos.DrawCube(cellPos + Vector3.up * 0.05f, new Vector3(cellSize, 0, cellSize));
        }
    }
}