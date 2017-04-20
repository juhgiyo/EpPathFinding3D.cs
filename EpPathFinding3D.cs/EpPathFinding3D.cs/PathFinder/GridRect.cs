/*! 
@file GridRect.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/eppathfinding3d.cs>
@date April 20, 2017
@brief GridRect Interface
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

An Interface for the GridRect Struct.

*/
using System;
using System.Collections.Generic;
using System.Collections;

namespace EpPathFinding3D.cs
{
    public class GridRect
    {
        public int minX;
        public int minY;
        public int minZ;
        public int maxX;
        public int maxY;
        public int maxZ;
        public GridRect()
        {
            minX = 0;
            minY = 0;
            minZ = 0;
            maxX = 0;
            maxY = 0;
            maxZ = 0;

        }
        public GridRect(int iMinX, int iMinY, int iMinZ, int iMaxX, int iMaxY, int iMaxZ)
        {
            minX = iMinX;
            minY = iMinY;
            minZ = iMinZ;

            maxX = iMaxX;
            maxY = iMaxY;
            maxZ = iMaxZ;
        }

        public GridRect(GridRect b)
        {
            minX = b.minX;
            minY = b.minY;
            minZ = b.minZ;
            maxX = b.maxX;
            maxY = b.maxY;
            maxZ = b.maxZ;
        }

        public override int GetHashCode()
        {
            return minX ^ minY ^ minZ ^ maxX ^ maxY ^ maxZ;
        }

        public override bool Equals(System.Object obj)
        {
            // Unlikely to compare incorrect type so removed for performance
            //if (!(obj.GetType() == typeof(GridRect)))
            //    return false;
            GridRect p = (GridRect)obj;
            if (ReferenceEquals(null, p))
            {
                return false;
            }
            // Return true if the fields match:
            return (minX == p.minX) && (minY == p.minY) && (minZ == p.minZ) && (maxX == p.maxX) && (maxY == p.maxY) && (maxZ == p.maxZ);
        }

        public bool Equals(GridRect p)
        {
            if (ReferenceEquals(null, p))
            {
                return false;
            }
            // Return true if the fields match:
            return (minX == p.minX) && (minY == p.minY) && (minZ == p.minZ) && (maxX == p.maxX) && (maxY == p.maxY) && (maxZ == p.maxZ);
        }

        public static bool operator ==(GridRect a, GridRect b)
        {
            // If both are null, or both are same instance, return true.
            if (System.Object.ReferenceEquals(a, b))
            {
                return true;
            }
            if (ReferenceEquals(null, a))
            {
                return false;
            }
            if (ReferenceEquals(null, b))
            {
                return false;
            }
            // Return true if the fields match:
            return (a.minX == b.minX) && (a.minY == b.minY) && (a.minZ == b.minZ) && (a.maxX == b.maxX) && (a.maxY == b.maxY) && (a.maxZ == b.maxZ);
        }

        public static bool operator !=(GridRect a, GridRect b)
        {
            return !(a == b);
        }

        public GridRect Set(int iMinX, int iMinY, int iMinZ, int iMaxX, int iMaxY, int iMaxZ)
        {
            this.minX = iMinX;
            this.minY = iMinY;
            this.minZ = iMinZ;
            this.maxX = iMaxX;
            this.maxY = iMaxY;
            this.maxZ = iMaxZ;
            return this;
        }
    }
}