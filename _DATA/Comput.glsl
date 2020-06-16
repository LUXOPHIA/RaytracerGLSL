#version 430

//#extension GL_ARB_compute_variable_group_size : enable

//layout( local_size_variable ) in;
  layout( local_size_x = 10,
          local_size_y = 10,
          local_size_z =  1 ) in;

////////////////////////////////////////////////////////////////////////////////

  ivec3 _WorkGrupsN = ivec3( gl_NumWorkGroups );

//ivec3 _WorkItemsN = ivec3( gl_LocalGroupSizeARB );
  ivec3 _WorkItemsN = ivec3( gl_WorkGroupSize     );

  ivec3 _WorksN     = _WorkGrupsN * _WorkItemsN;

  ivec3 _WorkID     = ivec3( gl_GlobalInvocationID );

//############################################################################## ■

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【定数】

const float Pi         = 3.141592653589793;
const float Pi2        = Pi * 2;
const float P2i        = Pi / 2;
const float FLOAT_MAX  = 3.402823e+38;
const float FLOAT_EPS  = 1.1920928955078125E-7;
const float FLOAT_EPS1 = FLOAT_EPS * 1E1;
const float FLOAT_EPS2 = FLOAT_EPS * 1E2;

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【型】

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TdFloat

struct TdFloat
{
  float o;
  float d;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TdVec3

struct TdVec3
{
  TdFloat x;
  TdFloat y;
  TdFloat z;
};

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【ルーチン】

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（一般）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Pow2

float Pow2( in float X )
{
  return X * X;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% length2

float length2( in vec3 V )
{
  return Pow2( V.x ) + Pow2( V.y ) + Pow2( V.z );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MinI

int MinI( in vec3 V )
{
  if ( V.x <= V.y ) {
    if ( V.x <= V.z ) return 0; else return 2;
  } else {
    if ( V.y <= V.z ) return 1; else return 2;
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MaxI

int MaxI( in vec3 V )
{
  if ( V.x >= V.y ) {
    if ( V.x >= V.z ) return 0; else return 2;
  } else {
    if ( V.y >= V.z ) return 1; else return 2;
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rand

uvec4 _RandSeed;

uint rotl( in uint x, in int k )
{
  return ( x << k ) | ( x >> ( 32 - k ) );
}

float Rand()
{
  const uint Result = rotl( _RandSeed[ 0 ] * 5, 7 ) * 9;

  const uint t = _RandSeed[ 1 ] << 9;

  _RandSeed[ 2 ] ^= _RandSeed[ 0 ];
  _RandSeed[ 3 ] ^= _RandSeed[ 1 ];
  _RandSeed[ 1 ] ^= _RandSeed[ 2 ];
  _RandSeed[ 0 ] ^= _RandSeed[ 3 ];

  _RandSeed[ 2 ] ^= t;

  _RandSeed[ 3 ] = rotl( _RandSeed[ 3 ], 11 );

  return uintBitsToFloat( Result & 0x007FFFFFu | 0x3F800000u ) - 1;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RandBS2

float RandBS2()
{
  return Rand() - Rand();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RandBS4

float RandBS4()
{
  return RandBS2() + RandBS2();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RandCirc

vec2 RandCirc()
{
  vec2 Result;
  float T, R;

  T = Pi2 * Rand();
  R = sqrt( Rand() );

  Result.x = R * cos( T );
  Result.y = R * sin( T );

  return Result;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（演算子）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TdFloat

TdFloat Add( TdFloat A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o = A_.o + B_.o;
  Result.d = A_.d + B_.d;

  return Result;
}

TdFloat Add( float A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o = A_ + B_.o;
  Result.d =      B_.d;

  return Result;
}

TdFloat Add( TdFloat A_, float B_ )
{
  TdFloat Result;

  Result.o = A_.o + B_;
  Result.d = A_.d     ;

  return Result;
}

//------------------------------------------------------------------------------

TdFloat Sub( TdFloat A_, float B_ )
{
  TdFloat Result;

  Result.o = A_.o - B_;
  Result.d = A_.d     ;

  return Result;
}

TdFloat Sub( float A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o = A_ - B_.o;
  Result.d =     -B_.d;

  return Result;
}

TdFloat Sub( TdFloat A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o = A_.o - B_.o;
  Result.d = A_.d - B_.d;

  return Result;
}

//------------------------------------------------------------------------------

TdFloat Mul( TdFloat A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o = A_.o * B_.o;
  Result.d = A_.d * B_.o + A_.o * B_.d;

  return Result;
}

TdFloat Mul( float A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o = A_ * B_.o;
  Result.d = A_ * B_.d;

  return Result;
}

TdFloat Mul( TdFloat A_, float B_ )
{
  TdFloat Result;

  Result.o = A_.o * B_;
  Result.d = A_.d * B_;

  return Result;
}

//------------------------------------------------------------------------------

TdFloat Div( TdFloat A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o =                 A_.o          /       B_.o  ;
  Result.d = ( A_.d * B_.o - A_.o * B_.d ) / Pow2( B_.o );

  return Result;
}

TdFloat Div( float A_, TdFloat B_ )
{
  TdFloat Result;

  Result.o =  A_        /       B_.o  ;
  Result.d = -A_ * B_.d / Pow2( B_.o );

  return Result;
}

TdFloat Div( TdFloat A_, float B_ )
{
  TdFloat Result;

  Result.o = A_.o / B_;
  Result.d = A_.d / B_;

  return Result;
}

////////////////////////////////////////////////////////////////////////////////

TdFloat Pow2( TdFloat A_ )
{
  TdFloat Result;

  Result.o = Pow2( A_.o );
  Result.d = 2 * A_.o * A_.d;

  return Result;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TdVec3

TdVec3 Add( TdVec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Add( A_.x, B_.x );
  Result.y = Add( A_.y, B_.y );
  Result.z = Add( A_.z, B_.z );

  return Result;
}

TdVec3 Add( vec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Add( A_.x, B_.x );
  Result.y = Add( A_.y, B_.y );
  Result.z = Add( A_.z, B_.z );

  return Result;
}

TdVec3 Add( TdVec3 A_, vec3 B_ )
{
  TdVec3 Result;

  Result.x = Add( A_.x, B_.x );
  Result.y = Add( A_.y, B_.y );
  Result.z = Add( A_.z, B_.z );

  return Result;
}

//------------------------------------------------------------------------------

TdVec3 Sub( TdVec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Sub( A_.x, B_.x );
  Result.y = Sub( A_.y, B_.y );
  Result.z = Sub( A_.z, B_.z );

  return Result;
}

TdVec3 Sub( vec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Sub( A_.x, B_.x );
  Result.y = Sub( A_.y, B_.y );
  Result.z = Sub( A_.z, B_.z );

  return Result;
}

TdVec3 Sub( TdVec3 A_, vec3 B_ )
{
  TdVec3 Result;

  Result.x = Sub( A_.x, B_.x );
  Result.y = Sub( A_.y, B_.y );
  Result.z = Sub( A_.z, B_.z );

  return Result;
}

//------------------------------------------------------------------------------

TdVec3 Mul( TdVec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_.x, B_.x );
  Result.y = Mul( A_.y, B_.y );
  Result.z = Mul( A_.z, B_.z );

  return Result;
}

TdVec3 Mul( vec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_.x, B_.x );
  Result.y = Mul( A_.y, B_.y );
  Result.z = Mul( A_.z, B_.z );

  return Result;
}

TdVec3 Mul( TdVec3 A_, vec3 B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_.x, B_.x );
  Result.y = Mul( A_.y, B_.y );
  Result.z = Mul( A_.z, B_.z );

  return Result;
}

//------------------------------------------------------------------------------

TdVec3 Mul( TdFloat A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_, B_.x );
  Result.y = Mul( A_, B_.y );
  Result.z = Mul( A_, B_.z );

  return Result;
}

TdVec3 Mul( float A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_, B_.x );
  Result.y = Mul( A_, B_.y );
  Result.z = Mul( A_, B_.z );

  return Result;
}

TdVec3 Mul( TdFloat A_, vec3 B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_, B_.x );
  Result.y = Mul( A_, B_.y );
  Result.z = Mul( A_, B_.z );

  return Result;
}

//------------------------------------------------------------------------------

TdVec3 Mul( TdVec3 A_, float B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_.x, B_ );
  Result.y = Mul( A_.y, B_ );
  Result.z = Mul( A_.z, B_ );

  return Result;
}

TdVec3 Mul( vec3 A_, TdFloat B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_.x, B_ );
  Result.y = Mul( A_.y, B_ );
  Result.z = Mul( A_.z, B_ );

  return Result;
}

TdVec3 Mul( TdVec3 A_, TdFloat B_ )
{
  TdVec3 Result;

  Result.x = Mul( A_.x, B_ );
  Result.y = Mul( A_.y, B_ );
  Result.z = Mul( A_.z, B_ );

  return Result;
}

//------------------------------------------------------------------------------

TdVec3 Div( TdVec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Div( A_.x, B_.x );
  Result.y = Div( A_.y, B_.y );
  Result.z = Div( A_.z, B_.z );

  return Result;
}

TdVec3 Div( vec3 A_, TdVec3 B_ )
{
  TdVec3 Result;

  Result.x = Div( A_.x, B_.x );
  Result.y = Div( A_.y, B_.y );
  Result.z = Div( A_.z, B_.z );

  return Result;
}

TdVec3 Div( TdVec3 A_, vec3 B_ )
{
  TdVec3 Result;

  Result.x = Div( A_.x, B_.x );
  Result.y = Div( A_.y, B_.y );
  Result.z = Div( A_.z, B_.z );

  return Result;
}

//------------------------------------------------------------------------------

TdVec3 Div( TdVec3 A_, TdFloat B_ )
{
  TdVec3 Result;

  Result.x = Div( A_.x, B_ );
  Result.y = Div( A_.y, B_ );
  Result.z = Div( A_.z, B_ );

  return Result;
}

TdVec3 Div( vec3 A_, TdFloat B_ )
{
  TdVec3 Result;

  Result.x = Div( A_.x, B_ );
  Result.y = Div( A_.y, B_ );
  Result.z = Div( A_.z, B_ );

  return Result;
}

TdVec3 Div( TdVec3 A_, float B_ )
{
  TdVec3 Result;

  Result.x = Div( A_.x, B_ );
  Result.y = Div( A_.y, B_ );
  Result.z = Div( A_.z, B_ );

  return Result;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（幾何学）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% VecToSky

vec2 VecToSky( in vec3 Vec )
{
  vec2 Result;

  Result.x = ( Pi - atan( -Vec.x, -Vec.z ) ) / Pi2;
  Result.y =        acos(  Vec.y           ) / Pi ;

  return Result;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% HitAABB

bool HitSlab( in  float RayP, in  float RayV,
              in  float MinP, in  float MaxP,
              out float MinT, out float MaxT )
{
  if ( RayV < -FLOAT_EPS2 )
  {
    MinT = ( MaxP - RayP ) / RayV;
    MaxT = ( MinP - RayP ) / RayV;
  }
  else
  if ( +FLOAT_EPS2 < RayV )
  {
    MinT = ( MinP - RayP ) / RayV;
    MaxT = ( MaxP - RayP ) / RayV;
  }
  else
  if ( ( MinP < RayP ) && ( RayP < MaxP ) )
  {
    MinT = -FLOAT_MAX;
    MaxT = +FLOAT_MAX;
  }
  else return false;

  return true;
}

//------------------------------------------------------------------------------

bool HitAABB( in  vec4  RayP, in  vec4  RayV,
              in  vec3  MinP, in  vec3  MaxP,
              out float MinT, out float MaxT,
              out int   IncA, out int   OutA )
{
  vec3 T0, T1;

  if ( HitSlab( RayP.x, RayV.x, MinP.x, MaxP.x, T0.x, T1.x )
    && HitSlab( RayP.y, RayV.y, MinP.y, MaxP.y, T0.y, T1.y )
    && HitSlab( RayP.z, RayV.z, MinP.z, MaxP.z, T0.z, T1.z ) )
  {
    IncA = MaxI( T0 );  MinT = T0[ IncA ];
    OutA = MinI( T1 );  MaxT = T1[ OutA ];

    return ( MinT < MaxT );
  }

  return false;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（光学）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ToneMap

vec3 ToneMap( in vec3 Color, in float White )
{
  return clamp( Color * ( 1 + Color / White ) / ( 1 + Color ), 0, 1 );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GammaCorrect

vec3 GammaCorrect( in vec3 Color, in float Gamma )
{
  vec3 Result;

  float G = 1 / Gamma;

  Result.r = pow( Color.r, G );
  Result.g = pow( Color.g, G );
  Result.b = pow( Color.b, G );

  return Result;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Fresnel

float Fresnel( in vec3 Vec, in vec3 Nor, in float IOR )
{
  float N2, C, G2, F0;
  // float N2C, G;

  N2 = Pow2( IOR );
  C  = dot( Nor, -Vec );
  G2 = N2 + Pow2( C ) - 1;
  if ( G2 < 0 ) return 1;

  // N2C = N2 * C;
  // G   = sqrt( G2 );
  // return ( Pow2( (   C - G ) / (   C + G ) )
  //        + Pow2( ( N2C - G ) / ( N2C + G ) ) ) / 2;

  F0 = Pow2( ( IOR - 1 ) / ( IOR + 1 ) );
  return F0 + ( 1 - F0 ) * pow( 1 - C, 5 );
}

//############################################################################## ■

layout( rgba32ui ) uniform uimage2D _Seeder;

layout( std430 ) buffer TAccumN
{
  int _AccumN;
};

layout( rgba32f ) uniform image2D _Accumr;

writeonly uniform image2D _Imager;

layout( std430 ) buffer TCamera
{
  layout( row_major ) mat4 _Camera;
};

uniform sampler2D _Textur;

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【型】

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRay

struct TRay
{
  vec4 Pos;
  vec4 Vec;
  vec3 Wei;
  vec3 Emi;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THit

struct THit
{
  float t;
  int   Mat;
  vec4  Pos;
  vec4  Nor;
};

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【ルーチン】

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（物体）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ObjPlane

void ObjPlane( in TRay Ray, inout THit Hit )
{
  float t;

  if ( Ray.Vec.y < 0 )
  {
    t = ( Ray.Pos.y - -1.001 ) / -Ray.Vec.y;

    if ( ( 0 < t ) && ( t < Hit.t ) )
    {
      Hit.t   = t;
      Hit.Pos = Ray.Pos + t * Ray.Vec;
      Hit.Nor = vec4( 0, 1, 0, 0 );
      Hit.Mat = 3;
    }
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ObjSpher

void ObjSpher( in TRay Ray, inout THit Hit )
{
  float B, C, D, t;

  B = dot( Ray.Pos.xyz, Ray.Vec.xyz );
  C = length2( Ray.Pos.xyz ) - 1;

  D = Pow2( B ) - C;

  if ( D > 0 )
  {
    t = -B - sign( C ) * sqrt( D );

    if ( ( 0 < t ) && ( t < Hit.t ) )
    {
      Hit.t   = t;
      Hit.Pos = Ray.Pos + t * Ray.Vec;
      Hit.Nor = Hit.Pos;
      Hit.Mat = 2;
    }
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ObjRecta

void ObjRecta( in TRay Ray, inout THit Hit )
{
  const vec3 MinP = vec3( -1, -1, -1 );
  const vec3 MaxP = vec3( +1, +1, +1 );

  float MinT, MaxT, T;
  int   IncA, OutA;

  if ( HitAABB( Ray.Pos, Ray.Vec, MinP, MaxP, MinT, MaxT, IncA, OutA ) )
  {
    if ( FLOAT_EPS2 < MinT ) T = MinT;
    else
    if ( FLOAT_EPS2 < MaxT ) T = MaxT;
    else return;

    if ( T < Hit.t )
    {
      Hit.t   = T;
      Hit.Pos = Ray.Pos + Hit.t * Ray.Vec;

      Hit.Nor = vec4( 0 );
      Hit.Nor[ IncA ] = -sign( Ray.Vec[ IncA ] );

      Hit.Mat = 2;
    }
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ObjImpli

TdFloat MathFunc( TdVec3 P )
{
  return Sub( Add( Add( Mul( Pow2( P.x ), 4 ), Pow2( P.y ) ), Pow2( P.z ) ), 1 );
}

//------------------------------------------------------------------------------

vec3 MathGrad( vec3 Pos )
{
  vec3   Result;
  TdVec3 P;

  P.x.o = Pos.x;
  P.y.o = Pos.y;
  P.z.o = Pos.z;

  P.x.d = 1;
  P.y.d = 0;
  P.z.d = 0;

  Result.x = MathFunc( P ).d;

  P.x.d = 0;
  P.y.d = 1;
  P.z.d = 0;

  Result.y = MathFunc( P ).d;

  P.x.d = 0;
  P.y.d = 0;
  P.z.d = 1;

  Result.z = MathFunc( P ).d;

  return Result;
}

//------------------------------------------------------------------------------

bool HitFunc( in TRay Ray, in float T2d, inout float HitT, out vec3 HitP )
{
  const uint LoopN = 16;

  float   Tds, Td;
  uint    N;
  TdFloat T;
  TdVec3  P;
  TdFloat F;

  Tds = 0;

  T = TdFloat( HitT, 1 );

  for ( N = 1; N <= LoopN; N++ )
  {
    P = Add( Mul( Ray.Vec.xyz, T ), Ray.Pos.xyz );

    F = MathFunc( P );

    if ( abs( F.o ) < 0.001 )
    {
      HitT   = T.o;

      HitP.x = P.x.o;
      HitP.y = P.y.o;
      HitP.z = P.z.o;

      return true;
    }

    Td = -F.o / F.d;

    Tds += Td;

    if ( abs( Tds ) > T2d ) return false;

    T.o += Td;
  }

  return false;
}

//------------------------------------------------------------------------------

void ObjImpli( in TRay Ray, inout THit Hit )
{
  const vec3  MinP = vec3( -1, -1, -1 ) - FLOAT_EPS2;
  const vec3  MaxP = vec3( +1, +1, +1 ) + FLOAT_EPS2;
  const float Td   = 0.1;
  const float T2d  = 1.5 * Td/2;

  float MinT, MaxT, T0;
  int   IncA, OutA;
  float T;
  vec3  P;

  if ( HitAABB( Ray.Pos, Ray.Vec, MinP, MaxP, MinT, MaxT, IncA, OutA ) )
  {
    for ( T0 = max( 0, MinT ); T0 < MaxT + Td; T0 += Td )
    {
      T = T0;

      if ( HitFunc( Ray, T2d, T, P ) && ( 0 < T ) && ( T < MaxT ) && ( T < Hit.t ) )
      {
        Hit.t   = T;
        Hit.Pos = vec4( P, 1 );
        Hit.Nor = vec4( normalize( MathGrad( P ) ), 0 );
        Hit.Mat = 2;

        break;
      }
    }
  }
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（材質）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatSkyer

bool MatSkyer( inout TRay Ray, in THit Hit )
{
  Ray.Emi += texture( _Textur, VecToSky( Ray.Vec.xyz ) ).rgb;

  return false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatMirro

bool MatMirro( inout TRay Ray, in THit Hit )
{
  Ray.Pos = Hit.Pos + FLOAT_EPS2 * Hit.Nor;
  Ray.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );

  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatWater

bool MatWater( inout TRay Ray, in THit Hit )
{
  TRay  Result;
  float C, IOR, F;
  vec4  Nor;

  C = dot( Hit.Nor.xyz, -Ray.Vec.xyz );

  if( 0 < C )
  {
    IOR = 1.333 / 1.000;
    Nor = +Hit.Nor;
  }
  else
  {
    IOR = 1.000 / 1.333;
    Nor = -Hit.Nor;
  }

  F = Fresnel( Ray.Vec.xyz, Nor.xyz, IOR );

  if ( Rand() < F )
  {
    Ray.Pos = Hit.Pos + FLOAT_EPS2 * Nor;
    Ray.Vec = vec4( reflect( Ray.Vec.xyz, Nor.xyz ), 0 );
  } else {
    Ray.Pos = Hit.Pos - FLOAT_EPS2 * Nor;
    Ray.Vec = vec4( refract( Ray.Vec.xyz, Nor.xyz, 1 / IOR ), 0 );
  }

  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatDiffu

bool MatDiffu( inout TRay Ray, in THit Hit )
{
  vec3  AX, AY, AZ, DX, DY, V;
  mat3  M;
  float R, T;

  AZ = Hit.Nor.xyz;

  switch( MinI( abs( AZ ) ) )
  {
    case 0:
        DX = vec3( 1, 0, 0 );
        AY = normalize( cross( AZ, DX ) );
        AX = cross( AY, AZ );
      break;
    case 1:
        DY = vec3( 0, 1, 0 );
        AX = normalize( cross( DY, AZ ) );
        AY = cross( AZ, AX );
      break;
    case 2:
        DX = vec3( 0, 0, 1 );
        AY = normalize( cross( AZ, DX ) );
        AX = cross( AY, AZ );
      break;
  }

  M = mat3( AX, AY, AZ );

  V.z = sqrt( Rand() );

  R = sqrt( 1 - Pow2( V.z ) );
  T = Rand();

  V.x = R * cos( Pi2 * T );
  V.y = R * sin( Pi2 * T );

  Ray.Pos = Hit.Pos + FLOAT_EPS2 * Hit.Nor;
  Ray.Vec = vec4( M * V, 0 );

  return true;
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

void Raytrace( inout TRay Ray )
{
  int  L;
  THit Hit;

  for ( L = 1; L <= 8; L++ )
  {
    Hit = THit( FLOAT_MAX, 0, vec4( 0 ), vec4( 0 ) );

    ///// 物体

    ObjPlane( Ray, Hit );
    //ObjSpher( Ray, Hit );
    //ObjRecta( Ray, Hit );
    ObjImpli( Ray, Hit );

    ///// 材質

    switch( Hit.Mat )
    {
      case 0: if ( MatSkyer( Ray, Hit ) ) break; else return;
      case 1: if ( MatMirro( Ray, Hit ) ) break; else return;
      case 2: if ( MatWater( Ray, Hit ) ) break; else return;
      case 3: if ( MatDiffu( Ray, Hit ) ) break; else return;
    }
  }
}

//------------------------------------------------------------------------------

void main()
{
  uint N;
  vec4 E, S;
  TRay R;
  vec3 A, C, P;

  _RandSeed = imageLoad( _Seeder, _WorkID.xy );

  if ( _AccumN == 0 ) A = vec3( 0 );
                 else A = imageLoad( _Accumr, _WorkID.xy ).rgb;

  for( N = _AccumN+1; N <= _AccumN+16; N++ )
  {
    E = vec4( 0.02 * RandCirc(), 0, 1 );

    S.x =       4.0 * ( _WorkID.x + 0.5 + RandBS4() ) / _WorksN.x - 2.0;
    S.y = 1.5 - 3.0 * ( _WorkID.y + 0.5 + RandBS4() ) / _WorksN.y      ;
    S.z = -2;
    S.w = 1;

    R.Pos = _Camera * E;
    R.Vec = _Camera * normalize( S - E );
    R.Wei = vec3( 1 );
    R.Emi = vec3( 0 );

    Raytrace( R );

    C = R.Wei * R.Emi;

    A += ( C - A ) / N;
  }

  imageStore( _Accumr, _WorkID.xy, vec4( A, 1 ) );

  P = GammaCorrect( ToneMap( A, 10 ), 2.2 );

  imageStore( _Imager, _WorkID.xy, vec4( P, 1 ) );

  imageStore( _Seeder, _WorkID.xy, _RandSeed );
}

//############################################################################## ■
