/*! 
@file BaseGrid.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/eppathfinding3d.cs>
@date April 20, 2017
@brief BaseGrid Interface
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

An Interface for the BaseGrid Class.

*/
using System;
using System.Collections.Generic;
using System.Collections;

namespace EpPathFinding3D.cs
{
    public class Node : IComparable<Node>
    {
        public int x;
        public int y;
        public int z;
        public bool walkable;
        public float heuristicStartToEndLen; // which passes current node
        public float startToCurNodeLen;
        public float? heuristicCurNodeToEndLen;
        public bool isOpened;
        public bool isClosed;
        public Object parent;

        public Node(int iX, int iY, int iZ, bool? iWalkable = null)
        {
            this.x = iX;
            this.y = iY;
            this.z = iZ;
            this.walkable = (iWalkable.HasValue ? iWalkable.Value : false);
            this.heuristicStartToEndLen = 0;
            this.startToCurNodeLen = 0;
            // this must be initialized as null to verify that its value never initialized
            // 0 is not good candidate!!
            this.heuristicCurNodeToEndLen = null;
            this.isOpened = false;
            this.isClosed = false;
            this.parent = null;

        }

        public Node(Node b)
        {
            this.x = b.x;
            this.y = b.y;
            this.z = b.z;
            this.walkable = b.walkable;
            this.heuristicStartToEndLen = b.heuristicStartToEndLen;
            this.startToCurNodeLen = b.startToCurNodeLen;
            this.heuristicCurNodeToEndLen = b.heuristicCurNodeToEndLen;
            this.isOpened = b.isOpened;
            this.isClosed = b.isClosed;
            this.parent = b.parent;
        }

        public void Reset(bool? iWalkable = null)
        {
            if (iWalkable.HasValue)
                walkable = iWalkable.Value;
            this.heuristicStartToEndLen = 0;
            this.startToCurNodeLen = 0;
            // this must be initialized as null to verify that its value never initialized
            // 0 is not good candidate!!
            this.heuristicCurNodeToEndLen = null ;
            this.isOpened = false;
            this.isClosed = false;
            this.parent = null;
        }

        public int CompareTo(Node iObj)
        {
            float result = this.heuristicStartToEndLen - iObj.heuristicStartToEndLen;
            if (result > 0.0f)
                return 1;
            else if (result == 0.0f)
                return 0;
            return -1;
        }
 

        public static List<GridPos> Backtrace(Node iNode)
        {
            List<GridPos> path = new List<GridPos>();
            path.Add(new GridPos(iNode.x, iNode.y, iNode.z));
            while (iNode.parent != null)
            {
                iNode = (Node)iNode.parent;
                path.Add(new GridPos(iNode.x, iNode.y, iNode.z));
            }
            path.Reverse();
            return path;
        }


        public override int GetHashCode()
        {
            return x ^ y ^ z;
        }

        public override bool Equals(System.Object obj)
        {
            // If parameter is null return false.
            if (obj == null)
            {
                return false;
            }

            // If parameter cannot be cast to Point return false.
            Node p = obj as Node;
            if ((System.Object)p == null)
            {
                return false;
            }

            // Return true if the fields match:
            return (x == p.x) && (y == p.y) && (z==p.z);
        }

        public bool Equals(Node p)
        {
            // If parameter is null return false:
            if ((object)p == null)
            {
                return false;
            }

            // Return true if the fields match:
            return (x == p.x) && (y == p.y) && (z == p.z);
        }

        public static bool operator ==(Node a, Node b)
        {
            // If both are null, or both are same instance, return true.
            if (System.Object.ReferenceEquals(a, b))
            {
                return true;
            }

            // If one is null, but not both, return false.
            if (((object)a == null) || ((object)b == null))
            {
                return false;
            }

            // Return true if the fields match:
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        public static bool operator !=(Node a, Node b)
        {
            return !(a == b);
        }

    }

    public abstract class BaseGrid
    {

        public BaseGrid()
        {
            m_gridRect = new GridRect();
        }

        public BaseGrid(BaseGrid b)
        {
            m_gridRect = new GridRect(b.m_gridRect);
            width = b.width;
            length = b.length;
            height = b.height;

        }

        protected GridRect m_gridRect;
        public GridRect gridRect
        {
            get { return m_gridRect; }
        }

        public abstract int width { get; protected set; }

        public abstract int length { get; protected set; }

        public abstract int height { get; protected set; }

        public abstract Node GetNodeAt(int iX, int iY, int iZ);

        public abstract bool IsWalkableAt(int iX, int iY, int iZ);

        public abstract bool SetWalkableAt(int iX, int iY, int iZ, bool iWalkable);

        public abstract Node GetNodeAt(GridPos iPos);

        public abstract bool IsWalkableAt(GridPos iPos);

        public abstract bool SetWalkableAt(GridPos iPos, bool iWalkable);

        /*      UPPER                    MIDDLE                   LOWER
         * **********************   *******************   **********************
         * * tDU3 * tSU2 * tDU2 *   * tD3 * tS2 * tD2 *   * tDD3 * tSD2 * tDD2 *
         * **********************   *******************   **********************
         * * tSU3 * tS4  * tSU1 *   * tS3 * Cur * tS1 *   * tSD3 * tS5  * tSD1 *
         * **********************   *******************   **********************
         * * tDU0 * tSU0 * tDU1 *   * tD0 * tS0 * tD1 *   * tDD0 * tSD0 * tDD1 *
         * **********************   *******************   **********************
        */

        public List<Node> GetNeighbors(Node iNode, DiagonalMovement diagonalMovement)
        {
            int tX = iNode.x;
            int tY = iNode.y;
            int tZ = iNode.z;
            List<Node> neighbors = new List<Node>();
            bool tS0 = false, tD0 = false,
                tS1 = false, tD1 = false,
                tS2 = false, tD2 = false,
                tS3 = false, tD3 = false,
                tS4 = false, 
                tS5 = false, 
                
                tSU0 = false, 
                tSU1 = false,
                tSU2 = false,
                tSU3 = false,
                tDU0 = false,
                tDU1 = false,
                tDU2 = false,
                tDU3 = false,

                tSD0 = false,
                tSD1 = false,
                tSD2 = false,
                tSD3 = false,
                tDD0 = false,
                tDD1 = false,
                tDD2 = false,
                tDD3 = false;

            GridPos pos = new GridPos();
            if (this.IsWalkableAt(pos.Set(tX, tY - 1, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
                tS0 = true;
            }
            if (this.IsWalkableAt(pos.Set(tX + 1, tY, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
                tS1 = true;
            }
            if (this.IsWalkableAt(pos.Set(tX, tY + 1, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
                tS2 = true;
            }
            if (this.IsWalkableAt(pos.Set(tX - 1, tY, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
                tS3 = true;
            }
            if (this.IsWalkableAt(pos.Set(tX, tY, tZ+1)))
            {
                neighbors.Add(GetNodeAt(pos));
                tS4 = true;
            }
            if (this.IsWalkableAt(pos.Set(tX, tY, tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
                tS5 = true;
            }

            switch (diagonalMovement)
            {
                case DiagonalMovement.Always:
                    tD0 = true;
                    tD1 = true;
                    tD2 = true;
                    tD3 = true;
                    break;
                case DiagonalMovement.Never:
                    break;
                case DiagonalMovement.IfAtLeastOneWalkable:
                    tD0 = tS3 || tS0;
                    tD1 = tS0 || tS1;
                    tD2 = tS1 || tS2;
                    tD3 = tS2 || tS3;
                    break;
                case DiagonalMovement.OnlyWhenNoObstacles:
                    tD0 = tS3 && tS0;
                    tD1 = tS0 && tS1;
                    tD2 = tS1 && tS2;
                    tD3 = tS2 && tS3;
                    break;
                default:
                    break;
            }

            if (tD0 && this.IsWalkableAt(pos.Set(tX - 1, tY - 1, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tD0 = false;
            }
            if (tD1 && this.IsWalkableAt(pos.Set(tX + 1, tY - 1, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
            }else
            {
                tD1 = false;
            }
            if (tD2 && this.IsWalkableAt(pos.Set(tX + 1, tY + 1, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
            }else
            {
                tD2 = false;
            }
            if (tD3 && this.IsWalkableAt(pos.Set(tX - 1, tY + 1, tZ)))
            {
                neighbors.Add(GetNodeAt(pos));
            }else
            {
                tD3 = false;
            }

            switch (diagonalMovement)
            {
                case DiagonalMovement.Always:

                    tSU0 = true;
                    tSU1 = true;
                    tSU2 = true;
                    tSU3 = true;

                    tSD0 = true;
                    tSD1 = true;
                    tSD2 = true;
                    tSD3 = true;
                    break;
                case DiagonalMovement.Never:
                    break;
                case DiagonalMovement.IfAtLeastOneWalkable:
                    tSU0 = tS0 || tS4;
                    tSU1 = tS3 || tS4;
                    tSU2 = tS1 || tS4;
                    tSU3 = tS2 || tS4;

                    tSD0 = tS0 || tS4;
                    tSD1 = tS3 || tS4;
                    tSD2 = tS1 || tS4;
                    tSD3 = tS2 || tS4;

                    break;
                case DiagonalMovement.OnlyWhenNoObstacles:
                    tSU0 = tS0 && tS4;
                    tSU1 = tS3 && tS4;
                    tSU2 = tS1 && tS4;
                    tSU3 = tS2 && tS4;

                    tSD0 = tS0 && tS4;
                    tSD1 = tS3 && tS4;
                    tSD2 = tS1 && tS4;
                    tSD3 = tS2 && tS4;
                    break;
                default:
                    break;
            }

            if (tSU0 && this.IsWalkableAt(pos.Set(tX, tY - 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSU0 = false;
            }
            if (tSU1 && this.IsWalkableAt(pos.Set(tX + 1, tY, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSU1 = false;
            }
            if (tSU2 && this.IsWalkableAt(pos.Set(tX, tY + 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSU2 = false;
            }
            if (tSU3 && this.IsWalkableAt(pos.Set(tX - 1, tY, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSU3 = false;
            }

            if (tSD0 && this.IsWalkableAt(pos.Set(tX, tY - 1 , tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSD0 = false;
            }
            if (tSD1 && this.IsWalkableAt(pos.Set(tX + 1, tY, tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSD1 = false;
            }
            if (tSD2 && this.IsWalkableAt(pos.Set(tX, tY + 1, tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSD2 = false;
            }
            if (tSD3 && this.IsWalkableAt(pos.Set(tX - 1, tY, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            else
            {
                tSD3 = false;
            }

            switch (diagonalMovement)
            {
                case DiagonalMovement.Always:
                    tDU0 = true;
                    tDU1 = true;
                    tDU2 = true;
                    tDU3 = true;

                    tDD0 = true;
                    tDD1 = true;
                    tDD2 = true;
                    tDD3 = true;
                    break;
                case DiagonalMovement.Never:
                    break;
                case DiagonalMovement.IfAtLeastOneWalkable:
                    tDU0 = tS0 || tS3 || tSU0 || tSU3;// || tD0 || tS4;
                    tDU1 = tS0 || tS1 || tSU0 || tSU1;// || tD1 || tS4;
                    tDU2 = tS2 || tS3 || tSU2 || tSU3;// || tD3 || tS4;
                    tDU3 = tS1 || tS2 || tSU1 || tSU2;// || tD2 || tS4;

                    tDD0 = tS0 || tS3 || tSD0 || tSD3;// || tD0 || tS5;
                    tDD1 = tS0 || tS1 || tSD0 || tSD1;// || tD1 || tS5;
                    tDD2 = tS2 || tS3 || tSD2 || tSD3;// || tD3 || tS5;
                    tDD3 = tS1 || tS2 || tSD1 || tSD2;// || tD2 || tS5;

                    break;
                case DiagonalMovement.OnlyWhenNoObstacles:
                    tDU0 = tS0 && tS3 && tSU0 && tSU3 && tD0 && tS4;
                    tDU1 = tS0 && tS1 && tSU0 && tSU1 && tD1 && tS4;
                    tDU2 = tS2 && tS3 && tSU2 && tSU3 && tD3 && tS4;
                    tDU3 = tS1 && tS2 && tSU1 && tSU2 && tD2 && tS4;

                    tDD0 = tS0 && tS3 && tSD0 && tSD3 && tD0 && tS5;
                    tDD1 = tS0 && tS1 && tSD0 && tSD1 && tD1 && tS5;
                    tDD2 = tS2 && tS3 && tSD2 && tSD3 && tD3 && tS5;
                    tDD3 = tS1 && tS2 && tSD1 && tSD2 && tD2 && tS5;
                    break;
                default:
                    break;
            }
            
            if (tDU0 && this.IsWalkableAt(pos.Set(tX - 1, tY - 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            if (tDU1 && this.IsWalkableAt(pos.Set(tX + 1, tY - 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            if (tDU2 && this.IsWalkableAt(pos.Set(tX + 1, tY + 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            if (tDU3 && this.IsWalkableAt(pos.Set(tX - 1, tY + 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }

            if (tDD0 && this.IsWalkableAt(pos.Set(tX - 1, tY - 1, tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            if (tDD1 && this.IsWalkableAt(pos.Set(tX + 1, tY - 1, tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            if (tDD2 && this.IsWalkableAt(pos.Set(tX + 1, tY + 1, tZ - 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            if (tDD3 && this.IsWalkableAt(pos.Set(tX - 1, tY + 1, tZ + 1)))
            {
                neighbors.Add(GetNodeAt(pos));
            }
            return neighbors;
        }


        public abstract void Reset();

        public abstract BaseGrid Clone();

    }
}