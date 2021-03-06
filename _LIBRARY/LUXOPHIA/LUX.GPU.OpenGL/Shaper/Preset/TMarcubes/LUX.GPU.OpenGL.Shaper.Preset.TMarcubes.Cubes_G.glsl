﻿#version 430

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【共通定数】

layout( std140 ) uniform TViewerScal{ layout( row_major ) mat4 _ViewerScal; };
layout( std140 ) uniform TCameraProj{ layout( row_major ) mat4 _CameraProj; };
layout( std140 ) uniform TCameraPose{ layout( row_major ) mat4 _CameraPose; };
layout( std140 ) uniform TShaperPose{ layout( row_major ) mat4 _ShaperPose; };

layout( std140 ) uniform TGridSize { vec3  _GridSize;  };
layout( std140 ) uniform TThreshold{ float _Threshold; };

//------------------------------------------------------------------------------

uniform sampler3D _Grid;

/*
 -1     0    +1    +2    +3    +4    +5 = ItemPoins Coordinate
  +-----+-----+-----+-----+-----+-----+
  |     |     |     |     |     |     |
  1     2     3     4     5     6     7 = ElemPoinsN
  +-----o-----o-----o-----o-----o-----+
  |  1  |  2  |  3  |  4  |  5  |  6  | = ElemCellsN = ElemPoinsN-1
  |     |     |     |     |     |     |
  +-----o-----o-----o-----o-----o-----+
  |     |     |     |     |     |     |
  |     |     |     |     |     |     |    + : ElemPoin
  +-----o-----o-----o-----o-----o-----+
  |     |     |     |     |     |     |    o : ItemPoin
  |     |     |     |     |     |     |
  +-----o-----o-----o-----o-----o-----+
  |     |     |     |     |     |     |
  |     1     2     3     4     5     | = ItemPoinsN = ElemCellsN-1
  +-----o-----o-----o-----o-----o-----+
  |     |  1  |  2  |  3  |  4  |     | = ItemCellsN = ItemPoinsN-1
  |     |     |     |     |     |     |
  +-----+-----+-----+-----+-----+-----+
  0    1/6   2/6   3/6   4/6   5/6    1 = Texture Coordinate
*/

const ivec3 _ElemPoinsN = textureSize( _Grid, 0 );
const ivec3 _ElemCellsN = _ElemPoinsN - ivec3( 1 );
const ivec3 _ItemPoinsN = _ElemCellsN - ivec3( 1 );
const ivec3 _ItemCellsN = _ItemPoinsN - ivec3( 1 );

const vec3 _CellSize = _GridSize / _ItemCellsN;

//############################################################################## ■

layout( points ) in;

in TSenderVG
{
  ivec3 Pos;
}
_Sender[ 1 ];

const int X0 = _Sender[ 0 ].Pos.x;  const int X1 = X0 + 1;
const int Y0 = _Sender[ 0 ].Pos.y;  const int Y1 = Y0 + 1;
const int Z0 = _Sender[ 0 ].Pos.z;  const int Z1 = Z0 + 1;

//------------------------------------------------------------------------------

layout( line_strip, max_vertices = 24 ) out;

out TSenderGF
{
  vec4 Pos;
}
_Result;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【ルーチン】

float GetPoins( int X, int Y, int Z )
{
  return texelFetch( _Grid, ivec3( 1 ) + ivec3( X, Y, Z ), 0 ).x - _Threshold;
}

//------------------------------------------------------------------------------

void AddPoin( vec3 Pos )
{
  _Result.Pos = _ShaperPose * vec4( Pos * _CellSize - _GridSize / 2, 1 );

  gl_Position = _ViewerScal * _CameraProj * inverse( _CameraPose ) * _Result.Pos;

  EmitVertex();
}

//------------------------------------------------------------------------------

void AddEdge( vec3 Pos1, vec3 Pos2 )
{
  AddPoin( Pos1 );
  AddPoin( Pos2 );

  EndPrimitive();
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

const float G000 = GetPoins( X0, Y0, Z0 );
const float G001 = GetPoins( X1, Y0, Z0 );
const float G010 = GetPoins( X0, Y1, Z0 );
const float G011 = GetPoins( X1, Y1, Z0 );
const float G100 = GetPoins( X0, Y0, Z1 );
const float G101 = GetPoins( X1, Y0, Z1 );
const float G110 = GetPoins( X0, Y1, Z1 );
const float G111 = GetPoins( X1, Y1, Z1 );

//------------------------------------------------------------------------------

int CubeKind()
{
  int Result = 0;
  if ( G000 < 0 ) Result += 0x01;  // 00000001
  if ( G001 < 0 ) Result += 0x02;  // 00000010
  if ( G010 < 0 ) Result += 0x04;  // 00000100
  if ( G011 < 0 ) Result += 0x08;  // 00001000
  if ( G100 < 0 ) Result += 0x10;  // 00010000
  if ( G101 < 0 ) Result += 0x20;  // 00100000
  if ( G110 < 0 ) Result += 0x40;  // 01000000
  if ( G111 < 0 ) Result += 0x80;  // 10000000
  return Result;
}

////////////////////////////////////////////////////////////////////////////////

void main()
{
  const int K = CubeKind();

  if ( ( 0 < K ) && ( K < 255 ) )
  {
    AddEdge( vec3( X0, Y0, Z0 ), vec3( X1, Y0, Z0 ) );
    AddEdge( vec3( X0, Y1, Z0 ), vec3( X1, Y1, Z0 ) );
    AddEdge( vec3( X0, Y0, Z1 ), vec3( X1, Y0, Z1 ) );
    AddEdge( vec3( X0, Y1, Z1 ), vec3( X1, Y1, Z1 ) );

    AddEdge( vec3( X0, Y0, Z0 ), vec3( X0, Y1, Z0 ) );
    AddEdge( vec3( X0, Y0, Z1 ), vec3( X0, Y1, Z1 ) );
    AddEdge( vec3( X1, Y0, Z0 ), vec3( X1, Y1, Z0 ) );
    AddEdge( vec3( X1, Y0, Z1 ), vec3( X1, Y1, Z1 ) );

    AddEdge( vec3( X0, Y0, Z0 ), vec3( X0, Y0, Z1 ) );
    AddEdge( vec3( X1, Y0, Z0 ), vec3( X1, Y0, Z1 ) );
    AddEdge( vec3( X0, Y1, Z0 ), vec3( X0, Y1, Z1 ) );
    AddEdge( vec3( X1, Y1, Z0 ), vec3( X1, Y1, Z1 ) );
  }
}

//############################################################################## ■
