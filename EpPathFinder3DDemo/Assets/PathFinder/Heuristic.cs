/*! 
@file Heuristic.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/eppathfinding3d.cs>
@date April 20, 2017
@brief Heuristic Function Interface
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

An Interface for the Heuristic Function Class.

*/
using System;
using System.Collections.Generic;
using System.Collections;

namespace EpPathFinding3D.cs
{
    public enum HeuristicMode
    {
        MANHATTAN,
        EUCLIDEAN,
        CHEBYSHEV,

    };

    public class Heuristic
    {
        public static float Manhattan(int iDx, int iDy, int iDz)
        {
            return (float)iDx + iDy + iDz;
        }

        public static float Euclidean(int iDx, int iDy, int iDz)
        {
            float tFdx = (float)iDx;
            float tFdy = (float)iDy;
            float tFdz = (float)iDz;
            return (float)Math.Sqrt((double)(tFdx * tFdx + tFdy * tFdy + tFdz * tFdz));
        }

        public static float Chebyshev(int iDx, int iDy, int iDz)
        {
            return (float)Math.Max(iDx, Math.Max(iDy, iDz));
        }
    }
}
