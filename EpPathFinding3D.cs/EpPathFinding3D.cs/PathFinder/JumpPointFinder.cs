/*! 
@file JumpPointFinder.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/eppathfinding3d.cs>
@date April 20, 2017
@brief Jump Point Search Algorithm Interface
@version 2.0

@section LICENSE

The MIT License (MIT)

Copyright (c) 2017 Woong Gyu La <juhgiyo@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

@section DESCRIPTION

An Interface for the Jump Point Search Algorithm Class.

*/
using C5;
using System;
using System.Collections.Generic;
using System.Collections;


namespace EpPathFinding3D.cs
{



    public class JumpPointParam : ParamBase
    {
 
        public JumpPointParam(BaseGrid iGrid, GridPos iStartPos, GridPos iEndPos, bool iAllowEndNodeUnWalkable = true, DiagonalMovement iDiagonalMovement = DiagonalMovement.Always, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iStartPos, iEndPos, iDiagonalMovement, iMode)
        {

            m_allowEndNodeUnWalkable = iAllowEndNodeUnWalkable;
            openList = new IntervalHeap<Node>();

            m_useRecursive = false;
        }

        public JumpPointParam(BaseGrid iGrid, bool iAllowEndNodeUnWalkable = true, DiagonalMovement iDiagonalMovement = DiagonalMovement.Always, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iDiagonalMovement, iMode)
        {
            m_allowEndNodeUnWalkable = iAllowEndNodeUnWalkable;

            openList = new IntervalHeap<Node>();
            m_useRecursive = false;
        }

        public JumpPointParam(JumpPointParam b):base(b)
        {
            m_heuristic = b.m_heuristic;
            m_allowEndNodeUnWalkable = b.m_allowEndNodeUnWalkable;

            openList = new IntervalHeap<Node>();
            openList.AddAll(b.openList);

            m_useRecursive = b.m_useRecursive;
        }



        internal override void _reset(GridPos iStartPos, GridPos iEndPos, BaseGrid iSearchGrid = null)
        {
            openList = new IntervalHeap<Node>();
            //openList.Clear();
        }

        public bool AllowEndNodeUnWalkable
        {
            get
            {
                return m_allowEndNodeUnWalkable;
            }
            set
            {
                m_allowEndNodeUnWalkable = value;
            }
        }

        public bool UseRecursive
        {
            get
            {
                return m_useRecursive;
            }
            set
            {
                m_useRecursive = value;
            }
        }

        protected bool m_allowEndNodeUnWalkable;

        protected bool m_useRecursive;


        //public List<Node> openList;
        public IntervalHeap<Node> openList;

    }
    public class JumpPointFinder
    {
        public static List<GridPos> GetFullPath(List<GridPos> routeFound)
        {
            if (routeFound == null)
                return null;
            List<GridPos> consecutiveGridList = new List<GridPos>();
            if (routeFound.Count > 1)
                consecutiveGridList.Add(new GridPos(routeFound[0]));
            for (int routeTrav = 0; routeTrav < routeFound.Count - 1; routeTrav++)
            {
                GridPos fromGrid = new GridPos(routeFound[routeTrav]);
                GridPos toGrid = routeFound[routeTrav + 1];
                int dX = toGrid.x - fromGrid.x;
                int dY = toGrid.y - fromGrid.y;
                int dZ = toGrid.z - fromGrid.z;
                
                int nDX = 0;
                int nDY = 0;
                int nDZ = 0;
                if (dX != 0)
                {
                    nDX = (dX / Math.Abs(dX));
                }
                if (dY != 0)
                {
                    nDY = (dY / Math.Abs(dY));
                }
                if (dZ != 0)
                {
                    nDZ = (dZ / Math.Abs(dZ));
                }
                while (fromGrid != toGrid)
                {
                    fromGrid.x += nDX;
                    fromGrid.y += nDY;
                    fromGrid.z += nDZ;
                    consecutiveGridList.Add(new GridPos(fromGrid));
                }

            }
            return consecutiveGridList;
        }
        public static List<GridPos> FindPath(JumpPointParam iParam)
        {

            IntervalHeap<Node> tOpenList = iParam.openList;
            Node tStartNode = iParam.StartNode;
            Node tEndNode = iParam.EndNode;
            Node tNode;
            bool revertEndNodeWalkable = false;

            // set the `g` and `f` value of the start node to be 0
            tStartNode.startToCurNodeLen = 0;
            tStartNode.heuristicStartToEndLen = 0;

            // push the start node into the open list
            tOpenList.Add(tStartNode);
            tStartNode.isOpened = true;

            if (iParam.AllowEndNodeUnWalkable && !iParam.SearchGrid.IsWalkableAt(tEndNode.x, tEndNode.y, tEndNode.z))
            {
                iParam.SearchGrid.SetWalkableAt(tEndNode.x, tEndNode.y, tEndNode.z, true);
                revertEndNodeWalkable = true;
            }

            // while the open list is not empty
            while (tOpenList.Count > 0)
            {
                // pop the position of node which has the minimum `f` value.
                tNode = tOpenList.DeleteMin();
                tNode.isClosed = true;

                if (tNode.Equals(tEndNode))
                {
                    if (revertEndNodeWalkable)
                    {
                        iParam.SearchGrid.SetWalkableAt(tEndNode.x, tEndNode.y, tEndNode.z, false);
                    }
                    return Node.Backtrace(tNode); // rebuilding path
                }

                identifySuccessors(iParam, tNode);
            }

            if (revertEndNodeWalkable)
            {
                iParam.SearchGrid.SetWalkableAt(tEndNode.x, tEndNode.y,tEndNode.z, false);
            }

            // fail to find the path
            return new List<GridPos>();
        }

        private static void identifySuccessors(JumpPointParam iParam, Node iNode)
        {
            HeuristicDelegate tHeuristic = iParam.HeuristicFunc;
            IntervalHeap<Node> tOpenList = iParam.openList;
            int tEndX = iParam.EndNode.x;
            int tEndY = iParam.EndNode.y;
            int tEndZ = iParam.EndNode.z;
            GridPos tNeighbor;
            GridPos tJumpPoint;
            Node tJumpNode;

            List<GridPos> tNeighbors = findNeighbors(iParam, iNode);
            for (int i = 0; i < tNeighbors.Count; i++)
            {
                tNeighbor = tNeighbors[i];
                if (iParam.UseRecursive)
                    tJumpPoint = jump(iParam, tNeighbor.x, tNeighbor.y, tNeighbor.z, iNode.x, iNode.y, iNode.z);
                else
                    tJumpPoint = jumpLoop(iParam, tNeighbor.x, tNeighbor.y, tNeighbor.z, iNode.x, iNode.y, iNode.z);
                if (tJumpPoint != null)
                {
                    tJumpNode = iParam.SearchGrid.GetNodeAt(tJumpPoint.x, tJumpPoint.y, tJumpPoint.z);
                    if (tJumpNode == null)
                    {
                        if (iParam.EndNode.x == tJumpPoint.x && iParam.EndNode.y == tJumpPoint.y && iParam.EndNode.z == tJumpPoint.z)
                            tJumpNode = iParam.SearchGrid.GetNodeAt(tJumpPoint);
                    }
                    if (tJumpNode.isClosed)
                    {
                        continue;
                    }
                    // include distance, as parent may not be immediately adjacent:
                    float tCurNodeToJumpNodeLen = tHeuristic(Math.Abs(tJumpPoint.x - iNode.x), Math.Abs(tJumpPoint.y - iNode.y), Math.Abs(tJumpPoint.z - iNode.z));
                    float tStartToJumpNodeLen = iNode.startToCurNodeLen + tCurNodeToJumpNodeLen; // next `startToCurNodeLen` value

                    if (!tJumpNode.isOpened || tStartToJumpNodeLen < tJumpNode.startToCurNodeLen)
                    {
                        tJumpNode.startToCurNodeLen = tStartToJumpNodeLen;
                        tJumpNode.heuristicCurNodeToEndLen = (tJumpNode.heuristicCurNodeToEndLen == null ? tHeuristic(Math.Abs(tJumpPoint.x - tEndX), Math.Abs(tJumpPoint.y - tEndY), Math.Abs(tJumpPoint.z - tEndZ)) : tJumpNode.heuristicCurNodeToEndLen);
                        tJumpNode.heuristicStartToEndLen = tJumpNode.startToCurNodeLen + tJumpNode.heuristicCurNodeToEndLen.Value;
                        tJumpNode.parent = iNode;

                        if (!tJumpNode.isOpened)
                        {
                            tOpenList.Add(tJumpNode);
                            tJumpNode.isOpened = true;
                        }
                    }
                }
            }
        }

        private class JumpSnapshot
        {
            public int iX;
            public int iY;
            public int iZ;
            public int iPx;
            public int iPy;
            public int iPz;
            public int tDx;
            public int tDy;
            public int tDz;
            public int stage;
            public JumpSnapshot()
            {

                iX = 0;
                iY = 0;
                iZ = 0;
                iPx = 0;
                iPy = 0;
                iPz = 0;
                tDx = 0;
                tDy = 0;
                tDz = 0;
                stage = 0;
            }
        }

        private static GridPos jumpLoop(JumpPointParam iParam, int iX, int iY, int iZ, int iPx, int iPy, int iPz)
        {
            GridPos retVal = null;
            Stack<JumpSnapshot> stack = new Stack<JumpSnapshot>();

            JumpSnapshot currentSnapshot = new JumpSnapshot();
            JumpSnapshot newSnapshot = null;
            currentSnapshot.iX = iX;
            currentSnapshot.iY = iY;
            currentSnapshot.iZ = iZ;
            currentSnapshot.iPx = iPx;
            currentSnapshot.iPy = iPy;
            currentSnapshot.iPz = iPz;

            currentSnapshot.stage = 0;

            stack.Push(currentSnapshot);
            while (stack.Count != 0)
            {
                currentSnapshot = stack.Pop();
                switch (currentSnapshot.stage)
                {
                    case 0:
                        if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ))
                        {
                            retVal = null;
                            continue;
                        }
                        else if (iParam.SearchGrid.GetNodeAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ).Equals(iParam.EndNode))
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.tDx = currentSnapshot.iX - currentSnapshot.iPx;
                        currentSnapshot.tDy = currentSnapshot.iY - currentSnapshot.iPy;
                        currentSnapshot.tDz = currentSnapshot.iZ - currentSnapshot.iPz;

                        if (iParam.DiagonalMovement == DiagonalMovement.Always || iParam.DiagonalMovement == DiagonalMovement.IfAtLeastOneWalkable)
                        {
                            // check for forced neighbors
                            // along the diagonal
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                            {
                                if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)))
                                {
                                    retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                    continue;
                                }
                            }
                            else
                            {
                                if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                        continue;
                                    }
                                }
                                else if (currentSnapshot.tDx != 0 && currentSnapshot.tDz != 0)
                                {

                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                        continue;
                                    }
                                }
                                else if (currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDx) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                        continue;
                                    }
                                }
                                // horizontally/vertically
                                else
                                {
                                    if (currentSnapshot.tDx != 0)
                                    {
                                        // moving along x
                                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ + 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ + 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - 1)))
                                        {
                                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                            continue;
                                        }
                                    }
                                    else if (currentSnapshot.tDy != 0)
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - 1)))
                                        {
                                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                            continue;
                                        }
                                    }
                                    else // currentSnapshot.tDz != 0
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - 1, currentSnapshot.iZ + currentSnapshot.tDz) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - 1, currentSnapshot.iZ)))
                                        {
                                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                            continue;
                                        }
                                    }
                                }
                            }
                            // when moving diagonally, must check for vertical/horizontal jump points
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                            {
                                currentSnapshot.stage = 1;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                            {
                                currentSnapshot.stage = 6;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDx != 0 && currentSnapshot.tDz != 0)
                            {
                                currentSnapshot.stage = 7;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                            {
                                currentSnapshot.stage = 8;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }

                            // when moving diagonally, must make sure one of the vertical/horizontal
                            // neighbors is open to allow the path
                            if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                 (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                 (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)) ||
                                 (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)) ||
                                 (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)) ||
                                 (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz))
                                )
                            {
                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                        }
                        else if (iParam.DiagonalMovement == DiagonalMovement.OnlyWhenNoObstacles)
                        {
                            // check for forced neighbors
                            // along the diagonal
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                            {
                                if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ))) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) ||
                                                                                                !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)))
                                    )
                                {
                                    retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                    continue;
                                }
                            }
                            else
                            {
                                if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                                {

                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ))) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1))) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1))))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                        continue;
                                    }
                                }
                                else if (currentSnapshot.tDx != 0 && currentSnapshot.tDz != 0)
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ))) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ))) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ))))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                        continue;
                                    }
                                }
                                else if (currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz))) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz))))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                        continue;
                                    }
                                }
                                // horizontally/vertically
                                else
                                {
                                    if (currentSnapshot.tDx != 0)
                                    {
                                        // moving along x
                                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1, currentSnapshot.iZ))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1, currentSnapshot.iZ))))
                                        {
                                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                            continue;
                                        }
                                    }
                                    else if (currentSnapshot.tDy != 0)
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ + 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ))) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ))) ||
                                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ - 1) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy, currentSnapshot.iZ))))
                                        {
                                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                            continue;
                                        }
                                    }
                                    else
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz)) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + 1, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - 1, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + 1, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1, currentSnapshot.iZ - currentSnapshot.tDz))) ||
                                            (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - 1, currentSnapshot.iZ) && (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY, currentSnapshot.iZ - currentSnapshot.tDz) || !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1, currentSnapshot.iZ - currentSnapshot.tDz))))
                                        {
                                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                            continue;
                                        }
                                    }
                                }
                            }
                            // when moving diagonally, must check for vertical/horizontal jump points
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                            {
                                currentSnapshot.stage = 10;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                            {
                                currentSnapshot.stage = 15;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDx != 0 && currentSnapshot.tDz != 0)
                            {
                                currentSnapshot.stage = 16;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDy != 0 && currentSnapshot.tDz != 0)
                            {
                                currentSnapshot.stage = 17;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }

                            // moving diagonally, must make sure both of the vertical/horizontal
                            // neighbors is open to allow the path
                            if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) &&
                                iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))
                            {
                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                        }
                        else // if(iParam.DiagonalMovement == DiagonalMovement.Never)
                        {
                            if (currentSnapshot.tDx != 0)
                            {
                                // moving along x
                                if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ))
                                {
                                    retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                    continue;
                                }
                            }
                            else if (currentSnapshot.tDy != 0)
                            {
                                if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ))
                                {
                                    retVal = new GridPos(iX, iY, iZ);
                                    continue;
                                }
                            }
                            else // if(currentSnapshot.tDz != 0)
                            {
                                if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))
                                {
                                    retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                                    continue;
                                }
                            }

                            //  must check for perpendicular jump points
                            if (currentSnapshot.tDx != 0)
                            {
                                currentSnapshot.stage = 19;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX;
                                newSnapshot.iY = currentSnapshot.iY + 1;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (currentSnapshot.tDy != 0)
                            {
                                currentSnapshot.stage = 22;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + 1;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else //if(currentSnapshot.tDz !=0)
                            {
                                currentSnapshot.stage = 25;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + 1;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iZ = currentSnapshot.iZ;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.iPz = currentSnapshot.iZ;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                        }
                        retVal = null;
                        break;
                    case 1:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 2;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 2:

                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 3;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 3:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 4;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 4:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 5;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 5:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 9;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 6:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 9;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 7:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 9;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 8:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 9;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;

                    case 9:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        // when moving diagonally, must check for vertical/horizontal jump points
                        // neighbors is open to allow the path
                        if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ)) ||
                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ)) ||
                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)) ||
                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)) ||
                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz)) ||
                             (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz))
                            )
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.iPz = currentSnapshot.iZ;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.iPz = currentSnapshot.iZ;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        retVal = null;
                        break;

                    case 10:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 11;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 11:

                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 12;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 12:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 13;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 13:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 14;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 14:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 18;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 15:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 18;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 16:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 18;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 17:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        currentSnapshot.stage = 18;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;


                    case 18:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }

                        // moving diagonally, must make sure both of the vertical/horizontal
                        // neighbors is open to allow the path
                        if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) &&
                        iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.iPz = currentSnapshot.iZ;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        retVal = null;
                        break;

                    case 19:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 20;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY - 1;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 20:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 21;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + 1;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 21:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 28;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ - 1;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 22:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 23;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX - 1;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 23:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 24;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ + 1;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 24:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 28;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ - 1;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 25:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 26;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX - 1;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 26:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 27;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + 1;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 27:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        currentSnapshot.stage = 28;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY - 1;
                        newSnapshot.iZ = currentSnapshot.iZ;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.iPz = currentSnapshot.iZ;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 28:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ);
                            continue;
                        }
                        // keep going
                        if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy, currentSnapshot.iZ) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY, currentSnapshot.iZ + currentSnapshot.tDz))
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iZ = currentSnapshot.iZ + currentSnapshot.tDz;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.iPz = currentSnapshot.iZ;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        retVal = null;
                        break;
                }
            }

            return retVal;
        }
        private static GridPos jump(JumpPointParam iParam, int iX, int iY, int iZ, int iPx, int iPy, int iPz)
        {
            if (!iParam.SearchGrid.IsWalkableAt(iX, iY, iZ))
            {
                return null;
            }
            else if (iParam.SearchGrid.GetNodeAt(iX, iY, iZ).Equals(iParam.EndNode))
            {
                return new GridPos(iX, iY, iZ);
            }

            int tDx = iX - iPx;
            int tDy = iY - iPy;
            int tDz = iZ - iPz;

            if (iParam.DiagonalMovement == DiagonalMovement.Always || iParam.DiagonalMovement == DiagonalMovement.IfAtLeastOneWalkable)
            {
                // check for forced neighbors
                // along the diagonal
                if(tDx != 0 && tDy != 0 && tDz != 0)
                {
                    if ((iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - 1)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - 1)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - tDy, iZ)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - tDz)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - tDz)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - tDy, iZ)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)))
                    {
                        return new GridPos(iX, iY, iZ);
                    }
                }
                else
                {
                    if (tDx != 0 && tDy != 0)
                    {
                        if ((iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)))
                        {
                            return new GridPos(iX, iY, iZ);
                        }
                    }
                    else if(tDx != 0 && tDz != 0)
                    {

                        if ((iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)))
                        {
                            return new GridPos(iX, iY, iZ);
                        }
                    }
                    else if (tDy !=0 && tDz != 0)
                    {
                        if ((iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ + tDx) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - tDz)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ)))
                        {
                            return new GridPos(iX, iY, iZ);
                        }
                    }
                    // horizontally/vertically
                    else
                    {
                        if (tDx != 0)
                        {
                            // moving along x
                            if ((iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ + 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ + 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - 1)))
                            {
                                return new GridPos(iX, iY, iZ);
                            }
                        }
                        else if (tDy != 0)
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ + 1)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ + 1)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ - 1)) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - 1)))
                            {
                                return new GridPos(iX, iY, iZ);
                            }
                        }
                        else // tDz != 0
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY + 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY - 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY + 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY - 1, iZ + tDz) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - 1, iZ)))
                            {
                                return new GridPos(iX, iY, iZ);
                            }
                        }
                    }
                }
                // when moving diagonally, must check for vertical/horizontal jump points
                if(tDx !=0 && tDy !=0 && tDz != 0)
                {
                    if(jump(iParam, iX + tDx, iY, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if(jump(iParam, iX, iY + tDy, iZ, iX, iY, iZ) !=null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX + tDx, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);

                    if (jump(iParam, iX, iY, iZ + tDz, iX, iY, iZ) !=null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX + tDx, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY + tDy, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);

                }
                else if (tDx != 0 && tDy != 0)
                {
                    if (jump(iParam, iX + tDx, iY, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                }
                else if (tDx != 0 && tDz != 0)
                {
                    if (jump(iParam, iX + tDx, iY, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                }
                else if (tDy != 0 && tDz != 0)
                {
                    if (jump(iParam, iX, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                }

                // moving diagonally, must make sure one of the vertical/horizontal
                // neighbors is open to allow the path
                if ( (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ) && iParam.SearchGrid.IsWalkableAt(iX+ tDx, iY + tDy, iZ)) ||
                     (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) && iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ)) ||
                     (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + tDz) && iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz)) ||
                     (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ + tDz) && iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz)) ||
                     (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ) && iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ + tDz)) ||
                     (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) && iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + tDz))
                    )
                {
                    return jump(iParam, iX + tDx, iY + tDy, iZ + tDz, iX, iY, iZ);
                }
                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                {
                    return jump(iParam, iX + tDx, iY + tDy, iZ + tDz, iX, iY, iZ);
                }
                
                return null;
            }
            else if (iParam.DiagonalMovement == DiagonalMovement.OnlyWhenNoObstacles)
            {
                // check for forced neighbors
                // along the diagonal
                if (tDx != 0 && tDy != 0 && tDz != 0)
                {
                    if ((iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ))) ||
                        (iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + tDz) && (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz))) ||
                        (iParam.SearchGrid.IsWalkableAt(iX+tDx, iY, iZ + tDz) && (!iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ) || !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz))) ||
                        (iParam.SearchGrid.IsWalkableAt(iX+tDx, iY +tDy, iZ+tDz) && (!iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ) || !iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ) || 
                                                                                    !iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + tDz) || !iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ+ tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ+tDz)))
                        )
                    {
                        return new GridPos(iX, iY, iZ);
                    }
                }
                else
                {
                    if (tDx != 0 && tDy != 0)
                    {

                        if ((iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ))) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - tDy, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + 1))) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - tDy, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - 1))) )
                        {
                            return new GridPos(iX, iY, iZ);
                        }
                    }
                    else if(tDx !=0 && tDz !=0)
                    {
                        if ((iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ + tDz) && (!iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz) || !iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ))) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1 , iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ))) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ))))
                        {
                            return new GridPos(iX, iY, iZ);
                        }
                    }
                    else if (tDy !=0 && tDz != 0)
                    {
                        if ((iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + tDz) && (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz))) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX + 1 , iY, iZ - tDz))) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ) || !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - tDz))))
                        {
                            return new GridPos(iX, iY, iZ);
                        }
                    }
                    // horizontally/vertically
                    else
                    {
                        if (tDx != 0)
                        {
                            // moving along x
                            if ((iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1, iZ))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1, iZ))))
                            {
                                return new GridPos(iX, iY, iZ);
                            }
                        }
                        else if (tDy != 0)
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY, iZ - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + 1)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ + 1) && (!iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ + 1) || !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ))) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy, iZ))) ||
                                 (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - 1) && (!iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy, iZ - 1) || !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy, iZ))))
                            {
                                return new GridPos(iX, iY, iZ);
                            }
                        }
                        else
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ - tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY + 1, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX + 1, iY + 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - tDz))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX + 1, iY - 1, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX + 1, iY - 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX + 1, iY, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - tDz))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + 1, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX - 1, iY + 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY + 1, iZ - tDz))) ||
                                (iParam.SearchGrid.IsWalkableAt(iX - 1, iY - 1, iZ) && (!iParam.SearchGrid.IsWalkableAt(iX - 1, iY - 1, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX - 1, iY, iZ - tDz) || !iParam.SearchGrid.IsWalkableAt(iX, iY - 1, iZ - tDz))))
                            {
                                return new GridPos(iX, iY, iZ);
                            }
                        }
                    }
                }
                


                // when moving diagonally, must check for vertical/horizontal jump points
                if (tDx != 0 && tDy != 0 && tDz != 0)
                {
                    if (jump(iParam, iX + tDx, iY, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX + tDx, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);

                    if (jump(iParam, iX, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX + tDx, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY + tDy, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);

                }
                else if (tDx != 0 && tDy != 0)
                {
                    if (jump(iParam, iX + tDx, iY, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                }
                else if (tDx != 0 && tDz != 0)
                {
                    if (jump(iParam, iX + tDx, iY, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                }
                else if (tDy != 0 && tDz != 0)
                {
                    if (jump(iParam, iX, iY + tDy, iZ, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ + tDz, iX, iY, iZ) != null)
                        return new GridPos(iX, iY, iZ);
                }

                // moving diagonally, must make sure both of the vertical/horizontal
                // neighbors is open to allow the path
                if (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ) && iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) && iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy, iZ) && 
                    iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ + tDz) && iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ + tDz) && iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz) )
                {
                    return jump(iParam, iX + tDx, iY + tDy, iZ + tDz, iX, iY, iZ);
                }
                else
                {
                    return null;
                }
            }
            else // if(iParam.DiagonalMovement == DiagonalMovement.Never)
            {
                if (tDx != 0)
                {
                    // moving along x
                    if (!iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ))
                    {
                        return new GridPos(iX, iY, iZ);
                    }
                }
                else if(tDy != 0)
                {
                    if (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ))
                    {
                        return new GridPos(iX, iY, iZ);
                    }
                }
                else // if(tDz != 0)
                {
                    if (!iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz))
                    {
                        return new GridPos(iX, iY, iZ);
                    }
                }

                //  must check for perpendicular jump points
                if (tDx != 0)
                {
                    if (jump(iParam, iX, iY + 1, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY - 1, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ + 1, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ - 1, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                }
                else if(tDy != 0)
                {
                    if (jump(iParam, iX + 1, iY, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX - 1, iY, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ + 1, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY, iZ - 1, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                }
                else //if(tDz !=0)
                {
                    if (jump(iParam, iX + 1, iY, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX - 1, iY, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY + 1, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                    if (jump(iParam, iX, iY - 1, iZ, iX, iY, iZ) != null) return new GridPos(iX, iY, iZ);
                }

                // keep going
                if (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY, iZ) && iParam.SearchGrid.IsWalkableAt(iX, iY + tDy, iZ) && iParam.SearchGrid.IsWalkableAt(iX, iY, iZ + tDz))
                {
                    return jump(iParam, iX + tDx, iY + tDy, iZ +tDz, iX, iY, iZ);
                }
                else
                {
                    return null;
                }
            }
        }

        private static List<GridPos> findNeighbors(JumpPointParam iParam, Node iNode)
        {
            Node tParent = (Node)iNode.parent;
            //var diagonalMovement = Util.GetDiagonalMovement(iParam.CrossCorner, iParam.CrossAdjacentPoint);
            int tX = iNode.x;
            int tY = iNode.y;
            int tZ = iNode.z;
            int tPx, tPy, tPz, tDx, tDy, tDz;
            List<GridPos> tNeighbors = new List<GridPos>();
            List<Node> tNeighborNodes;
            Node tNeighborNode;

            // directed pruning: can ignore most neighbors, unless forced.
            if (tParent != null)
            {
                tPx = tParent.x;
                tPy = tParent.y;
                tPz = tParent.z;
                // get the normalized direction of travel
                tDx = (tX - tPx) / Math.Max(Math.Abs(tX - tPx), 1);
                tDy = (tY - tPy) / Math.Max(Math.Abs(tY - tPy), 1);
                tDz = (tZ - tPz) / Math.Max(Math.Abs(tZ - tPz), 1);

                if (iParam.DiagonalMovement == DiagonalMovement.Always || iParam.DiagonalMovement == DiagonalMovement.IfAtLeastOneWalkable)
                {
                    // search diagonally
                    if(tDx != 0 && tDy!=0 && tDz != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                        }

                        // X && Y Dimension
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) || iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - 1))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - 1))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ));
                            }
                        }

                        // X && Z Dimension
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) || iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ+tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ+tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - tDz));
                            }
                        }

                        // Y && Z Dimension
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) || iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) )
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY - tDy, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX, tY - tDy, tZ + tDz));
                            }
                        }

                        // Bottom Left overs
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY +tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz)))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ - tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ - tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) &&iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz)) )
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ - tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ - tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) )
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ - tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ - tDz));
                            }
                        }

                        // Top Left overs
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ + tDz))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY , tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) )
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY-tDy, tZ + tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX+tDy, tY, tZ + tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY , tZ))
                                )
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) || 
                                 (iParam.SearchGrid.IsWalkableAt(tX , tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) )
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY - tDy, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY - tDy, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                        {
                            if ((iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) || iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ +tDz) || iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) || iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) || iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) )
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ + tDz));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ + tDz));
                            }
                        }

                    }
                    else
                    {
                        if (tDx != 0 && tDy != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) || iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ));
                                }
                            }

                            if(!iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + 1));
                                    }
                                    
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ + 1))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ))
                                       )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ + 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ + 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ + 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ + 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ + 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ + 1));
                                    }
                                }
                            }

                            if (!iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - 1));
                                    }

                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ - 1))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ))
                                       )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ - 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ - 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ - 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ - 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ - 1));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ - 1));
                                    }
                                }
                            }

                        }
                        else if (tDx != 0 && tDz != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) || iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + tDz));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX - tDx, tY, tZ + tDz));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX - tDx, tY, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - tDz));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - tDz));
                                }
                            }

                            if (!iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + tDz));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ));
                                    }

                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ + tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz))
                                       )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + 1, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + 1, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ - tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ - tDz));
                                    }
                                }
                            }

                            if (!iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + tDz));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ));
                                    }

                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ + tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz))
                                       )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - 1, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY - 1, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY - 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ - tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ - tDz));
                                    }
                                }
                            }
                        }
                        else if(tDy != 0 && tDz != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) || iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + tDz));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - tDz));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY - tDy, tZ + tDz));
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    tNeighbors.Add(new GridPos(tX, tY - tDy, tZ + tDz));
                                }
                            }

                            if (!iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY, tZ + tDz));
                                    }

                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ + tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz))
                                       )
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ - tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ - tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY - tDy, tZ + tDx) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                                {
                                    if ( (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY - tDy, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY - tDy, tZ + tDz));
                                    }
                                }
                            }

                            if (!iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY, tZ + tDz));
                                    }

                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ + tDz))
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz))
                                       )
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ - tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                                {
                                    if ( (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ - tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ - tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY - tDy, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY - tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                                {
                                    if ( (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        )
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY - tDy, tZ + tDz));
                                    }
                                    else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY - tDy, tZ + tDz));
                                    }
                                }
                            }

                        }
                        // search horizontally/vertically
                        else
                        {
                           
                            if (tDx != 0)
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ));
                                    }
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ));
                                    }
                                }

                                if (!iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + 1));
                                        }

                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ + 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ + 1));
                                        }   
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                            )
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ + 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ + 1));
                                        }
                                    }
                                }

                                if (!iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - 1));
                                        }

                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ - 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ - 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ - 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ - 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                            )
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ - 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ - 1));
                                        }
                                    }
                                }

                            }
                            else if (tDy != 0)
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ));
                                    }
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ));
                                    }
                                }

                                if (!iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + 1));
                                        }

                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ + 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ + 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ + 1) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                            )
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ + 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ + 1));
                                        }
                                    }
                                }

                                if (!iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - 1));
                                        }

                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ - 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ - 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ - 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ - 1) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ - 1))
                                    {
                                        if ( (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ)) ||
                                            (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                            )
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ - 1));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ - 1));
                                        }
                                    }
                                }
                            }
                            else // tDz != 0
                            {
                                // X & Z
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));

                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + tDz));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + tDz));
                                    }
                                }
                                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + tDz));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + tDz));
                                    }
                                }

                                if (!iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY, tZ + tDz));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY, tZ + tDz));
                                        }

                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY + 1, tZ))
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + 1, tZ + tDz));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + 1, tZ + tDz));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY - 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY - 1, tZ))
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                            (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                            )
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY - 1, tZ + tDz));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY - 1, tZ + tDz));
                                        }
                                    }
                                }

                                if (!iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY, tZ + tDz));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY, tZ + tDz));
                                        }

                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY + 1, tZ))
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                        (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + 1, tZ + tDz));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + 1, tZ + tDz));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY - 1, tZ + tDz) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY - 1, tZ))
                                    {
                                        if ((iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz)) ||
                                            (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                            )
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY - 1, tZ + tDz));
                                        }
                                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY - 1, tZ + tDz));
                                        }
                                    }
                                }
                            }
                        }
                    }
                 
                }
                else if (iParam.DiagonalMovement == DiagonalMovement.OnlyWhenNoObstacles)
                {
                    // search diagonally
                    if (tDx != 0 && tDy != 0 && tDz != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                        }

                        // X && Y Dimension
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - 1))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - 1))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ));
                            }
                        }

                        // X && Z Dimension
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - tDz));
                            }
                        }

                        // Y && Z Dimension
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - tDz))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - tDz));
                            }
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY - tDy, tZ + tDz));
                            }
                        }

                        // Bottom Left overs
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX+tDx, tY + tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ- tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ - tDz));
                        }


                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                        {
                            tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ - tDz));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ - tDz));
                        }

                        // Top Left overs
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ + tDz));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ + tDz));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX - tDx, tY - tDy, tZ + tDz));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) &&
                            iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ + tDz));
                        }

                    }
                    else
                    {
                        if (tDx != 0 && tDy != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ + 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1))
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ + 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + 1))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ + 1));
                                    }
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + tDy, tZ - 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1))
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + tDy, tZ - 1));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy, tZ) &&
                                       iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ - 1))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - tDy, tZ - 1));
                                    }
                                }
                            }
                            if(iParam.SearchGrid.IsWalkableAt(tX,tY,tZ+1))
                            {
                                tNeighbors.Add(new GridPos(tX, tY, tZ + 1));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                            {
                                tNeighbors.Add(new GridPos(tX, tY, tZ - 1));
                            }


                        }
                        else if (tDx != 0 && tDz != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX - tDx, tY, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + tDz));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY + 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ - tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ - tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ - tDz));
                                    }
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + tDz));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - 1, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX - tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX - tDx, tY - 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ - tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ - tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ - tDz));
                                    }
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + 1, tZ));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY - 1, tZ));
                            }


                        }
                        else if (tDy != 0 && tDz != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                            {
                                tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY - tDy, tZ + tDz));
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) &&
                                       iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ - tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) &&
                                       iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ - tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY - tDy, tZ + tDx))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) &&
                                       iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY - tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY - tDy, tZ + tDz));
                                    }
                                }
                            }

                            if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + tDz) &&
                                       iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ - tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - tDz) &&
                                       iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ - tDz) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ - tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY - tDy, tZ + tDz))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - tDy, tZ + tDz) &&
                                       iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY - tDy, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY - tDy, tZ + tDz));
                                    }
                                }
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY , tZ))
                            {
                                tNeighbors.Add(new GridPos(tX + 1, tY, tZ));
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                            {
                                tNeighbors.Add(new GridPos(tX - 1, tY, tZ));
                            }
                        }
                        // search horizontally/vertically
                        else
                        {

                            if (tDx != 0)
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ + 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ + 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ + 1));
                                        }
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ - 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ - 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY + 1, tZ - 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1, tZ) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ - 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX + tDx, tY - 1, tZ - 1));
                                        }
                                    }
                                }

                                if(iParam.SearchGrid.IsWalkableAt(tX, tY+1, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + 1, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY - 1, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY, tZ + 1));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY, tZ - 1));
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + 1))
                                {
                                    if(iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY +1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ - 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ - 1));
                                    }
                                }

                            }
                            else if (tDy != 0)
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ + 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1 , tY + tDy, tZ) &&
                                         iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ + 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ + 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) &&
                                         iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ + 1));
                                        }
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ - 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy, tZ) &&
                                         iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ - 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + tDy, tZ - 1));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ - 1))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy, tZ) &&
                                         iParam.SearchGrid.IsWalkableAt(tX, tY + tDy, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ - 1))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + tDy, tZ - 1));
                                        }
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + 1, tY, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX - 1, tY, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY, tZ + 1));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY, tZ - 1));
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY, tZ + 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY, tZ + 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY, tZ - 1));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ - 1))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY, tZ - 1));
                                    }
                                }
                            }
                            else // tDz != 0
                            {
                                // X & Z
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));

                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY + 1, tZ + tDz));
                                    }
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX, tY - 1, tZ + tDz));
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY, tZ + tDz));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + 1, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY + 1, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY + 1, tZ + tDz));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY - 1, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY- 1, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX + 1, tY - 1, tZ + tDz));
                                        }
                                    }
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY, tZ + tDz));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + 1, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY + 1, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY + 1, tZ + tDz));
                                        }
                                    }

                                    if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY - 1, tZ + tDz))
                                    {
                                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ + tDz) &&
                                        iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ + tDz) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY - 1, tZ))
                                        {
                                            tNeighbors.Add(new GridPos(tX - 1, tY - 1, tZ + tDz));
                                        }
                                    }
                                }


                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX + 1, tY, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX - 1, tY, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY + 1, tZ));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                                {
                                    tNeighbors.Add(new GridPos(tX, tY - 1, tZ));
                                }

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY + 1, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY + 1, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY - 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX + 1, tY - 1, tZ));
                                    }
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY - 1, tZ))
                                {
                                    if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                                    {
                                        tNeighbors.Add(new GridPos(tX - 1, tY - 1, tZ));
                                    }
                                }
                            }
                        }
                    }

                }
                else // if(iParam.DiagonalMovement == DiagonalMovement.Never)
                {
                    if (tDx != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY, tZ));
                        }
                        
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY +1, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY - 1, tZ));
                        }
                        
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ +1));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ - 1));
                        }
                    }
                    else if (tDy != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY +tDy, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + tDy, tZ));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX + 1, tY, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX - 1, tY, tZ));
                        }
                        
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + 1))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ +1));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ - 1))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ - 1));
                        }
                    }
                    else // if (tDz != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY, tZ + tDz))
                        {
                            tNeighbors.Add(new GridPos(tX, tY, tZ + tDz));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX + 1, tY, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX - 1, tY, tZ));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + 1, tZ));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1, tZ))
                        {
                            tNeighbors.Add(new GridPos(tX, tY - 1, tZ));
                        }
                    }
                }
            }
            // return all neighbors
            else
            {
                tNeighborNodes = iParam.SearchGrid.GetNeighbors(iNode, iParam.DiagonalMovement);
                for (int i = 0; i < tNeighborNodes.Count; i++)
                {
                    tNeighborNode = tNeighborNodes[i];
                    tNeighbors.Add(new GridPos(tNeighborNode.x, tNeighborNode.y, tNeighborNode.z));
                }
            }

            return tNeighbors;
        }
    }
}
