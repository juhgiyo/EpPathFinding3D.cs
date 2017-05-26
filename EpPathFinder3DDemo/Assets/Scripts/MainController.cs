using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using EpPathFinding3D.cs;
using System.IO;
using System.Runtime.InteropServices;

public class MainController : MonoBehaviour {
	
	//UI	
	public Dropdown m_ddDiagonalMode;
    public Toggle m_tRecursive, m_tOpenedCell, m_tTotalMap;    
    
    //cube & line
    public GameObject m_cube;
    public Dictionary<int, GameObject> m_cubeMap;
    public List<Testbed> m_testbedList = new List<Testbed>();

    string m_map;
    string m_startend;

    //pathfinding
    public JumpPointParam m_jumpParam;

    //colors
    Color m_hidden;
    Color m_basic;
    Color m_start;
    Color m_end;
    Color m_open;
    Color m_close;
    Color m_unwalkable;
    
    //cube methods
    private void buildCubes(int x, int y, int z)
    {   
		GameObject cubeset = new GameObject ();
		cubeset.name = "CubeSet";
		for (int i=0; i<z; i++)
		{
			for (int j=0; j<y; j++)
			{
				for (int k=0; k<x; k++)
				{					
					GameObject newcube = Instantiate(m_cube);
					newcube.transform.position = new Vector3(k-((x-1)/2.0f),i-((z-1)/2.0f), j - ((y - 1) / 2.0f));
                    newcube.GetComponent<MeshRenderer>().material.color = m_basic;
					int myindex = k*100 + j*10 + i;
					newcube.name = string.Format("Cube_{0}{1}{2}",k.ToString(),j.ToString(),i.ToString());
					newcube.transform.parent = cubeset.transform;
					m_cubeMap[myindex] = newcube;
				}
			}
		}
        return;
	}

	private void setCubeColor(GridPos g, Color c)
	{
		int index = g.x*100 + g.y*10 + g.z;
		GameObject selcube = m_cubeMap[index];
        selcube.GetComponent<Renderer>().material.color = c;
		return;
	}

	private GameObject drawLine(GridPos start, GridPos end)
	{
		int start_index = start.x * 100 + start.y * 10 + start.z;
		int end_index = end.x * 100 + end.y * 10 + end.z;

		Vector3 startpos = m_cubeMap[start_index].transform.position;
		Vector3 endpos = m_cubeMap[end_index].transform.position;

		GameObject newline = new GameObject();
		newline.transform.position = startpos;
		newline.AddComponent<LineRenderer>();
		LineRenderer lr = newline.GetComponent<LineRenderer>();
		lr.SetColors(Color.black, Color.black);
		lr.SetWidth(0.1f, 0.1f);
		lr.SetPosition(0, startpos);
		lr.SetPosition(1, endpos);

		return newline;
	}
    private void drawTrace(Testbed tb)
	{        
		GridPos start = new GridPos(), end;		
        tb.pTrace.name = string.Format("Trace_{0}-{1}", tb.start.ToString(), tb.end.ToString());
		for(int i=0; i<tb.result.Count; i++)
		{
            if (i == 0) start = tb.result[i];
            else
            {
                end = tb.result[i];
                GameObject newline = drawLine(start, end);
                newline.transform.parent = tb.pTrace.transform;
                tb.trace.Add(newline);
                start = end;
            }
		}	
	}

    // Use this for initialization
    void Start () {

        m_hidden = new Color(1.0f, 1.0f, 1.0f, 0.0f); //invisible
        m_basic = new Color(1.0f, 1.0f, 1.0f, 0.03f); //visible

        m_start = new Color(Color.green.r, Color.green.g, Color.green.b, 1.0f);
        m_end = new Color(Color.red.r, Color.red.g, Color.red.b, 1.0f);
        m_open = new Color(Color.blue.r, Color.blue.g, Color.blue.b, 0.3f);
        m_close = new Color(Color.cyan.r, Color.cyan.g, Color.cyan.b, 0.3f);

        m_unwalkable = new Color(Color.black.r, Color.black.g, Color.black.b, 1.0f);


        //// UI ////
        //loading & mapping
        m_cube = Resources.Load("Prefabs/Cube") as GameObject;
		GameObject.Find ("Canvas/btnClear").GetComponent<Button>().onClick.AddListener(OnbtnClearClicked);
		GameObject.Find ("Canvas/btnSearch").GetComponent<Button>().onClick.AddListener(OnbtnSearchClicked);
		GameObject.Find ("Canvas/btnLoadMap").GetComponent<Button>().onClick.AddListener(OnbtnLoadMapClicked);
		GameObject.Find("Canvas/btnLoadStartEnd").GetComponent<Button>().onClick.AddListener(OnbtnLoadStartEndClicked);
		m_ddDiagonalMode = GameObject.Find ("Canvas/DropdownMode").GetComponent<Dropdown>();
		m_tRecursive = GameObject.Find("Canvas/ToggleRecursive").GetComponent<Toggle>();
        m_tOpenedCell = GameObject.Find("Canvas/ToggleOpenedCell").GetComponent<Toggle>();
        m_tTotalMap = GameObject.Find("Canvas/ToggleTotalMap").GetComponent<Toggle>();

        m_tOpenedCell.onValueChanged.AddListener((value) => { OntOpenedCellValue(value); });
        m_tTotalMap.onValueChanged.AddListener((value) => { OntTotalMapValue(value); });



        ////cube setup////
        m_cubeMap = new Dictionary<int,GameObject>();
        int width = 10, length = 10, height = 10;
        buildCubes(width, length, height);

        ////Testbeds////
        //2 testbeds
		//m_testbedList.Add(new Testbed()); //default
		//m_testbedList.Add(new Testbed(new GridPos(1, 2, 3), new GridPos(9, 8, 7))); //custom

        ////path grid setup////
        //convert all matrix walkable
        bool[][][] movableMatrix = new bool [width][][];
		for(int widthTrav=0; widthTrav< width; widthTrav++)
		{
			movableMatrix[widthTrav]=new bool[length][];
			for(int lengthTrav=0; lengthTrav < length; lengthTrav++)
			{
				movableMatrix[widthTrav][lengthTrav]=new bool[height];
				for(int heightTrav=0; heightTrav < height;  heightTrav++)
				{ 
					movableMatrix[widthTrav][lengthTrav][heightTrav]=true; 
				}  
			}
		}
        BaseGrid searchGrid = new StaticGrid(width, length, height, movableMatrix);
		m_jumpParam = new JumpPointParam(searchGrid, new GridPos(), new GridPos(), true,0);                
	}
	// Update is called once per frame
	void Update ()
	{		
	}

    private void findPath()
    {
		m_jumpParam.UseRecursive = m_tRecursive.isOn;
		m_jumpParam.DiagonalMovement = (DiagonalMovement)m_ddDiagonalMode.value;
		foreach (Testbed i in m_testbedList)
        {
			m_jumpParam.Reset(i.start, i.end);
			i.result = JumpPointFinder.FindPath(m_jumpParam);
            getOpenClose(i);
			drawResult(i);
        }
    }
    private void getOpenClose(Testbed tb)
    {
        for(int x=0; x<m_jumpParam.SearchGrid.width; x++)
        {
            for(int y=0; y<m_jumpParam.SearchGrid.length; y++)
            {
                for(int z=0; z<m_jumpParam.SearchGrid.height; z++)
                {                    
                    GridPos curPos = new GridPos(x, y, z);
                    Node curNode = m_jumpParam.SearchGrid.GetNodeAt(curPos);
                    if (curPos != tb.start && curPos != tb.end)
                    {
                        if (curNode.isOpened) tb.opened.Add(curPos);
                        else if (curNode.isClosed) tb.closed.Add(curPos);
                    }
                }
            }
        }            
    }
    private void drawResult(Testbed tb)
    {		
		drawTrace(tb);
        if(m_tOpenedCell.isOn) markOpenClose(tb);        
    }
    private void clearResult()
    {
        foreach (Testbed tb in m_testbedList)
        {
            clearOpenClose(tb);
            foreach (GameObject line in tb.trace)
                Destroy(line);
            tb.opened.Clear();
            tb.closed.Clear();
            tb.trace.Clear();
        }
        markAllStartEnd(); //sometimes, start/end cubes are gone (by other path's open/closed mark)
    }
    
    void showBasicCube()
    {
        foreach (GameObject cube in m_cubeMap.Values)
        {
            Color curColor = cube.GetComponent<MeshRenderer>().material.color;
            if (curColor == m_hidden)
                cube.GetComponent<MeshRenderer>().material.color = m_basic;
        }
    }
    void hideBasicCube()
    {
        foreach (GameObject cube in m_cubeMap.Values)
        {
            Color curColor = cube.GetComponent<MeshRenderer>().material.color;
            if (curColor == m_basic)
                cube.GetComponent<MeshRenderer>().material.color = m_hidden;
        }
    }
    private void markOpenClose(Testbed tb)
    {        
        foreach(GridPos pos in tb.opened)
            setCubeColor(pos, m_open);
        foreach (GridPos pos in tb.closed)
            setCubeColor(pos, m_close);
    }
    private void clearOpenClose(Testbed tb)
    {
        foreach (GridPos cube in tb.opened)
        {
            if (m_tTotalMap.isOn) setCubeColor(cube, m_basic);
            else setCubeColor(cube, m_hidden);
        }
        foreach (GridPos cube in tb.closed)
        {
            if (m_tTotalMap.isOn) setCubeColor(cube, m_basic);
            else setCubeColor(cube, m_hidden);
        }
    }

    

    void OntOpenedCellValue(bool isOn)
    {
        if (isOn)
            foreach (Testbed tb in m_testbedList)
                markOpenClose(tb);
        else
            foreach (Testbed tb in m_testbedList)
                clearOpenClose(tb);
        markAllStartEnd(); //sometimes, start/end cubes are gone (by other path's open/closed mark)
    }
    void OntTotalMapValue(bool isOn)
    {
        if (isOn) showBasicCube();
        else hideBasicCube();
    }

    void OnbtnClearClicked()
    {
        clearResult();
    }
    void OnbtnSearchClicked()
    {
        clearResult();
        findPath();
    }

    void OnbtnLoadMapClicked()
    {
        Debug.Log("LoadMap button Clicked");

        OpenFileName.Open("Select a map file.", (f) => {
            Debug.Log(" Selected file with full path: " + f);
            m_map = File.ReadAllText(f);
            clearMap();
            drawMap();
        });
    }
    void drawMap()
    {
        clearResult();
        string[] cubes = m_map.Split('\n');
        foreach (string cube in cubes)
        {
            string[] pos = cube.Split(',');
            int x = int.Parse(pos[0]);
            int y = int.Parse(pos[1]);
            int z = int.Parse(pos[2]);
            GridPos target = new GridPos(x, y, z);
            m_jumpParam.SearchGrid.SetWalkableAt(target, false);
            setCubeColor(target, Color.black);
        }
    }
    private void clearMap()
    {
        //all convert to walkable
        for (int widthTrav = 0; widthTrav < m_jumpParam.SearchGrid.width; widthTrav++)
        {
            for (int lengthTrav = 0; lengthTrav < m_jumpParam.SearchGrid.length; lengthTrav++)
            {
                for (int heightTrav = 0; heightTrav < m_jumpParam.SearchGrid.height; heightTrav++)
                {
                    GridPos curPos = new GridPos(widthTrav, lengthTrav, heightTrav);
                    if (!m_jumpParam.SearchGrid.IsWalkableAt(curPos))
                    {
                        m_jumpParam.SearchGrid.SetWalkableAt(curPos, true);
                        setCubeColor(curPos, m_basic);
                    }
                }
            }
        }
    }

    void OnbtnLoadStartEndClicked()
    {
        Debug.Log("LoadStratEnd button Clicked");
        //clear previous startend set
        clearResult();
        foreach (Testbed tb in m_testbedList)
        {
            setCubeColor(tb.start, m_basic);
            setCubeColor(tb.end, m_basic);
        }
        m_testbedList.Clear();

        OpenFileName.Open("Select a map file.", (f) => {
            Debug.Log(" Selected file with full path: " + f);
            string mapText = File.ReadAllText(f);
            string[] tbs = mapText.Split('\n');            
            foreach (string layer in tbs)
            {
                string[] poses = layer.Split(' ');

                string[] startPosString = poses[0].Split(',');
                GridPos start = new GridPos(int.Parse(startPosString[0]), int.Parse(startPosString[1]), int.Parse(startPosString[2]));
                string[] endPosString = poses[1].Split(',');
                GridPos end = new GridPos(int.Parse(endPosString[0]), int.Parse(endPosString[1]), int.Parse(endPosString[2]));
                m_testbedList.Add(new Testbed(start, end));                
            }
            markAllStartEnd();
        });
    }
    void markAllStartEnd()
    {
        foreach(Testbed tb in m_testbedList)
        {
            setCubeColor(tb.start, m_start);
            setCubeColor(tb.end, m_end);
        }
    }
}
