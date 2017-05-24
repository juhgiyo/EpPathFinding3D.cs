/*! 
@file StaticGrid.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/eppathfinding3d.cs>
@date April 20, 2017
@brief StaticGrid Interface
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

An Interface for the StaticGrid Class.

*/
using System;
using System.Collections.Generic;
using System.Collections;

namespace EpPathFinding3D.cs
{
    public class StaticGrid : BaseGrid
    {
        public override int width { get; protected set; }
        public override int length { get; protected set; }

        public override int height { get; protected set; }

        private Node[][][] m_nodes;

        public StaticGrid(int iWidth, int iLength, int iHeight, bool[][][] iMatrix = null):base()
        {
            width = iWidth;
            length = iLength;
            height = iHeight;
            m_gridRect.minX = 0;
            m_gridRect.minY = 0;
            m_gridRect.minZ = 0;
            m_gridRect.maxX = iWidth-1;
            m_gridRect.maxY = iLength - 1;
            m_gridRect.maxZ = iHeight - 1;
            this.m_nodes = buildNodes(iWidth,iLength, iHeight, iMatrix);
        }

        public StaticGrid(StaticGrid b)
            : base(b)
        {
            bool[][][] tMatrix = new bool[b.width][][];
            for (int widthTrav = 0; widthTrav < b.width; widthTrav++)
            {
                tMatrix[widthTrav] = new bool[b.length][];
                for (int lengthTrav = 0; lengthTrav < b.length; lengthTrav++)
                {
                    tMatrix[widthTrav][lengthTrav] = new bool[b.height];
                    for (int heightTrav = 0; heightTrav < b.height; heightTrav++)
                    {
                        if (b.IsWalkableAt(widthTrav, lengthTrav, heightTrav))
                            tMatrix[widthTrav][lengthTrav][heightTrav] = true;
                        else
                            tMatrix[widthTrav][lengthTrav][heightTrav] = false;
                    }
                }
                
            }
            this.m_nodes = buildNodes(b.width, b.length, b.height, tMatrix);
        }
       
        private Node[][][] buildNodes(int iWidth, int iLength, int iHeight, bool[][][] iMatrix)
        {

            Node[][][] tNodes = new Node[iWidth][][];
            for (int widthTrav = 0; widthTrav < iWidth; widthTrav++)
            {
                tNodes[widthTrav] = new Node[iLength][];
                for (int lengthTrav = 0; lengthTrav < iLength; lengthTrav++)
                {
                    tNodes[widthTrav][lengthTrav] = new Node[iHeight];
                    for (int heightTrav = 0; heightTrav < iHeight; heightTrav++)
                    {
                        tNodes[widthTrav][lengthTrav][heightTrav] = new Node(widthTrav,lengthTrav, heightTrav, null);
                    }
                }
                
            }

            if (iMatrix == null)
            {
                return tNodes;
            }

            if (iMatrix.Length != iWidth || iMatrix[0].Length != iLength || iMatrix[0][0].Length != iHeight)
            {
                throw new System.Exception("Matrix size does not fit");
            }


            for (int widthTrav = 0; widthTrav < iWidth; widthTrav++)
            {
                for (int lengthTrav = 0; lengthTrav < iLength; lengthTrav++)
                {
                    for (int heightTrav = 0; heightTrav < iHeight; heightTrav++)
                    {
                        if (iMatrix[widthTrav][lengthTrav][heightTrav])
                        {
                            tNodes[widthTrav][lengthTrav][heightTrav].walkable = true;
                        }
                        else
                        {
                            tNodes[widthTrav][lengthTrav][heightTrav].walkable = false;
                        }
                    }
                }
            }
            return tNodes;
        }

        public override Node GetNodeAt(int iX, int iY, int iZ)
        {
            return this.m_nodes[iX][iY][iZ];
        }

        public override bool IsWalkableAt(int iX, int iY, int iZ)
        {
            return isInside(iX, iY, iZ) && this.m_nodes[iX][iY][iZ].walkable;
        }

        protected bool isInside(int iX, int iY, int iZ)
        {
            return (iX >= 0 && iX < width) && (iY >= 0 && iY < length) && (iZ >= 0 && iZ < height);
        }

        public override bool SetWalkableAt(int iX, int iY,int iZ, bool iWalkable)
        {
            this.m_nodes[iX][iY][iZ].walkable = iWalkable;
            return true;
        }

        protected bool isInside(GridPos iPos)
        {
            return isInside(iPos.x, iPos.y, iPos.z);
        }

        public override Node GetNodeAt(GridPos iPos)
        {
            return GetNodeAt(iPos.x, iPos.y, iPos.z);
        }

        public override bool IsWalkableAt(GridPos iPos)
        {
            return IsWalkableAt(iPos.x, iPos.y, iPos.z);
        }

        public override bool SetWalkableAt(GridPos iPos, bool iWalkable)
        {
            return SetWalkableAt(iPos.x, iPos.y, iPos.z, iWalkable);
        }

        public override void Reset()
        {
            Reset(null);
        }

        public void Reset(bool[][][] iMatrix)
        {
            for (int widthTrav = 0; widthTrav < width; widthTrav++)
            {
                for(int lengthTrav =0;lengthTrav < length; lengthTrav++)
                {
                    for (int heightTrav = 0; heightTrav < height; heightTrav++)
                    {
                        m_nodes[widthTrav][lengthTrav][heightTrav].Reset();
                    }
                }
            }

            if (iMatrix == null)
            {
                return;
            }
            if (iMatrix.Length != width || iMatrix[0].Length != length || iMatrix[0][0].Length != height)
            {
                throw new System.Exception("Matrix size does not fit");
            }

            for (int widthTrav = 0; widthTrav < width; widthTrav++)
            {
                for(int lengthTrav = 0; lengthTrav < length; lengthTrav++)
                {
                    for (int heightTrav = 0; heightTrav < height; heightTrav++)
                    {
                        if (iMatrix[widthTrav][lengthTrav][heightTrav])
                        {
                            m_nodes[widthTrav][lengthTrav][heightTrav].walkable = true;
                        }
                        else
                        {
                            m_nodes[widthTrav][lengthTrav][heightTrav].walkable = false;
                        }
                    }
                }
            }
        }

        public override BaseGrid Clone()
        {
            int tWidth = width;
            int tHeight = height;
            int tLength = length;
            Node[][][] tNodes = this.m_nodes;

            StaticGrid tNewGrid = new StaticGrid(tWidth, tLength, tHeight, null);

            Node[][][] tNewNodes = new Node[tWidth][][];
            for (int widthTrav = 0; widthTrav < tWidth; widthTrav++)
            {
                tNewNodes[widthTrav] = new Node[tLength][];
                for (int lengthTrav=0;lengthTrav<tLength; lengthTrav++)
                {
                    tNewNodes[widthTrav][lengthTrav] = new Node[tHeight];
                    for (int heightTrav = 0; heightTrav < tHeight; heightTrav++)
                    {
                        tNewNodes[widthTrav][lengthTrav][heightTrav] = new Node(widthTrav, lengthTrav,heightTrav, tNodes[widthTrav][lengthTrav][heightTrav].walkable);
                    }
                }
                
            }
            tNewGrid.m_nodes = tNewNodes;

            return tNewGrid;
        }
    }


}
