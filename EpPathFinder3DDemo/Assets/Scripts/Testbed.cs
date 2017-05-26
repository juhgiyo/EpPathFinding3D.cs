using System;
using EpPathFinding3D.cs;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Testbed
{
	public GridPos start, end;
	public List<GridPos> result;
	public List<GameObject> trace;

	public Testbed()
	{
		start = new GridPos(0, 0, 0);
		end = new GridPos(3, 3, 3);
		result = new List<GridPos>();
		trace = new List<GameObject>();
	}
	public Testbed(GridPos istart, GridPos iend)
	{
		start = istart;
		end = iend;
		result = new List<GridPos>();
		trace = new List<GameObject>();
	}
}
