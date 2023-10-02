using UnityEditor;
using UnityEngine;

public class GridDebug : MonoBehaviour
{
    private Grid _grid;
    [Space] [SerializeField] private bool _drawValues;
    [Space] [SerializeField, Range(0, 1)] private float _opacityMultiplier = 1.0f;
    
    [SerializeField] private Color _cellColor = Color.white;
    private GUIStyle _labelStyle;
    private Cell _hoveredCell;

    private void Start()
    {
        _labelStyle = new GUIStyle
        {
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            fontSize = 11,
            normal =
            {
                textColor = Color.white
            }
        };
    }

    
    public void SetGrid(Grid grid)
    {
        _grid = grid;
    }
    
    private void DrawCell(Cell cell)
    {
        var cellPosition = _grid.CellToWorldPosition(cell);

        Color color = _cellColor;
        color.a *= _opacityMultiplier;
        Gizmos.color = color;
        
        float cellSize = _grid.CellSize;
        Gizmos.DrawCube(cellPosition + Vector3.up * 0.05f, new Vector3(cellSize,0,cellSize)); 
        
        Gizmos.DrawWireCube(cellPosition, new Vector3(cellSize,0,cellSize));
        
        if (_drawValues)
        {
            Handles.Label(
                cellPosition + new Vector3(0,0,0), 
                new GUIContent(cell.Coord.x + "," + cell.Coord.y), 
                _labelStyle);     
        }
    }
    
    
    public void OnDrawGizmos()
    {
        if (_grid == null)
            return;

        if (_grid.Width > 200 || _grid.Height > 200)
            return;

        for (int x = 0; x < _grid.Cells.GetLength(0); x++)
        {
            for (int y = 0; y < _grid.Cells.GetLength(1); y++)
            {
                Cell cell = _grid.GetCell(x, y);
                DrawCell(cell);
            }
        }
    }
}