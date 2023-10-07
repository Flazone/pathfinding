using System;
using UnityEngine;

public class Grid
{
    [Serializable]
    public struct GridSettings
    {
        public int Width;
        public int Height;
        public float CellSize;
        public Vector3 Origin;

        public GridSettings(int width, int height, float cellSize, Vector3 origin)
        {
            Width = width;
            Height = height;
            CellSize = cellSize;
            Origin = origin;
        }
    }
    
    private readonly int _width;
    public int Width => _width;
    
    private readonly int _height;
    public int Height => _height;

    private float _cellSize;
    public float CellSize => _cellSize;
    
    private readonly Vector3 _origin;

    private Cell[,] _cells;
    public Cell[,] Cells => _cells;
    
    public Grid(int width, int height, float cellSize, Vector3 origin)
    {
        _width = width;
        _height = height;
        _cellSize = cellSize;
        _origin = origin;

        CreateCells();
    }

    public Grid(GridSettings settings)
    {
        _width = settings.Width;
        _height = settings.Height;
        _cellSize = settings.CellSize;
        _origin = settings.Origin;

        CreateCells();
    }

    private void CreateCells()
    {
        if (_width <= 0)
            throw new ArgumentException();
        
        if (_height <= 0)
            throw new ArgumentException();
        
        if (_cellSize <= 0)
            throw new ArgumentException();
        
        _cells = new Cell[_width, _height];
        
        for (int x = 0; x < _cells.GetLength(0); x++)
        {
            for (int y = 0; y < _cells.GetLength(1); y++)
            {
                _cells[x, y] = new Cell() { Coord = new Vector2Int(x, y)};
            }
        }
    }
    
    public Cell GetCell(int x, int y)
    {
        x = Math.Clamp(x, 0, _width-1);
        y = Math.Clamp(y, 0, _height-1);

        return _cells[x, y];
    }
    
    public Vector3 CellToWorldPosition(int x, int y)
    {
        return CellToWorldPosition(GetCell(x, y));
    }

    public Vector3 CellToWorldPosition(Cell cell)
    {
        float offsetX = (cell.X + 0.5f) * _cellSize + _origin.x;
        float offsetY = 0;
        float offsetZ = (cell.Y + 0.5f) * _cellSize + _origin.z;
    
        return new Vector3(offsetX, offsetY, offsetZ);
    }

    public Cell WorldPositionToCell(Vector3 worldPosition)
    {
        int x = Mathf.FloorToInt((worldPosition - _origin).x / _cellSize);
        int y = Mathf.FloorToInt((worldPosition - _origin).z / _cellSize);
        
        return GetCell(x, y);
    }

    public Vector3 WorldPositionToCellPosition(Vector3 worldPosition)
    {
        var cell = WorldPositionToCell(worldPosition);
        return CellToWorldPosition(cell);
    }
}

public struct Cell
{
    public Vector2Int Coord;
    public int X => Coord.x;
    public int Y => Coord.y;
    private int _value;
    
    public void SetValue(int value)
    {
        _value = value;
    }

    public int GetValue()
    {
        return _value;
    }
}