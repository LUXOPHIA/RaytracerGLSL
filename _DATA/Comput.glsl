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
const float FLOAT_EPS3 = FLOAT_EPS * 1E3;
const float FLOAT_EPS4 = FLOAT_EPS * 1E4;

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【型】

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

int MinI( in float A, in float B, in float C )
{
  if ( A <= B ) {
    if ( A <= C ) return 0; else return 2;
  } else {
    if ( B <= C ) return 1; else return 2;
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MaxI

int MaxI( in float A, in float B, in float C )
{
  if ( A >= B ) {
    if ( A >= C ) return 0; else return 2;
  } else {
    if ( B >= C ) return 1; else return 2;
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
    IncA = MaxI( T0.x, T0.y, T0.z );  MinT = T0[ IncA ];
    OutA = MinI( T1.x, T1.y, T1.z );  MaxT = T1[ OutA ];

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

    if ( MaxT < Hit.t )
    {
      Hit.t   = T;
      Hit.Pos = Ray.Pos + Hit.t * Ray.Vec;

      Hit.Nor = vec4( 0 );
      Hit.Nor[ IncA ] = -sign( Ray.Vec[ IncA ] );

      Hit.Mat = 2;
    }
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ObjPrimi

float TorusDF( in vec3 Pos, in float CirR, in float PipR )
{
  vec2 Q;

  Q = vec2( length( Pos.yz ) - CirR, Pos.x );

  return length( Q ) - PipR;
}

float WarpTwist( inout vec3 Pos, in float RotA, in float TwiR )
{
  float C, S, AR;

  C = cos( RotA * Pos.y );
  S = sin( RotA * Pos.y );

  Pos.xz = mat2(  C, -S,
                 +S,  C ) * Pos.xz;

  AR = RotA * TwiR;

  return 1.0 / sqrt( ( 2.0 + AR * ( AR + sqrt( 4.0 + Pow2( AR ) ) ) ) / 2.0 );  // Lipschitz constant
}

float DistFunc( in vec3 Pos )
{
  return WarpTwist( Pos, 1.0, 1.0+1.0/3.0 ) * TorusDF( Pos, 1.0, 1.0/3.0 );
}

//------------------------------------------------------------------------------

vec3 GetNormal( in vec3 P )
{
  const vec3 Xd = vec3( FLOAT_EPS2, 0, 0 );
  const vec3 Yd = vec3( 0, FLOAT_EPS2, 0 );
  const vec3 Zd = vec3( 0, 0, FLOAT_EPS2 );

  vec3 Result;

  Result.x = DistFunc( P + Xd ) - DistFunc( P - Xd );
  Result.y = DistFunc( P + Yd ) - DistFunc( P - Yd );
  Result.z = DistFunc( P + Zd ) - DistFunc( P - Zd );

  return normalize( Result );
}

//------------------------------------------------------------------------------

bool ObjPrimi( in TRay Ray, inout THit Hit )
{
  vec3  P0, V0, N0, P, N;
  float D0, S0, A0, T, A, D;
  int   I;

  P0 = Ray.Pos.xyz;
  V0 = Ray.Vec.xyz;

  D0 = DistFunc( P0 );

  if ( abs( D0 ) < FLOAT_EPS4 )
  {
    N0 = GetNormal( P0 );
    S0 = sign( dot( N0, V0 ) );
    P0 = P0 + ( S0 * FLOAT_EPS4 - D0 ) * N0;
    D0 = DistFunc( P0 );
  }
  else S0 = sign( D0 );

  A0 = S0 * D0;

  T =  0;
  A = A0;

  for( I = 1; I <= 100; I++ )
  {
    T = T + A;
    P = P0 + V0 * T;
    D = DistFunc( P );
    A = S0 * D;

    if ( A < FLOAT_EPS3 )
    {
      if ( ( 0 < T ) && ( T < Hit.t ) )
      {
        N = GetNormal( P );
        P = P - D * N;

        Hit.t   = T;
        Hit.Pos = vec4( P, 1 );
        Hit.Nor = vec4( N, 0 );
        Hit.Mat = 2;

        return true;
      }

      return false;
    }
  }

  return false;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&（材質）

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatSkyer

TRay MatSkyer( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec = Ray.Vec;
  Result.Pos = Ray.Pos;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi + texture( _Textur, VecToSky( Ray.Vec.xyz ) ).rgb;

  return Result;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatMirro

TRay MatMirro( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );
  Result.Pos = Hit.Pos + FLOAT_EPS2 * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatWater

TRay MatWater( in TRay Ray, in THit Hit )
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
    Result.Vec = vec4( reflect( Ray.Vec.xyz, Nor.xyz ), 0 );
    Result.Pos = Hit.Pos + FLOAT_EPS2 * Nor;
    Result.Wei = Ray.Wei;
    Result.Emi = Ray.Emi;
  } else {
    Result.Vec = vec4( refract( Ray.Vec.xyz, Nor.xyz, 1 / IOR ), 0 );
    Result.Pos = Hit.Pos - FLOAT_EPS2 * Nor;
    Result.Wei = Ray.Wei;
    Result.Emi = Ray.Emi;
  }

  return Result;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MatDiffu

TRay MatDiffu( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec.y = sqrt( Rand() );

  float d = sqrt( 1 - Pow2( Result.Vec.y ) );
  float v = Rand();

  Result.Vec.x = d * cos( Pi2 * v );
  Result.Vec.z = d * sin( Pi2 * v );

  Result.Pos = Hit.Pos + FLOAT_EPS2 * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
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
    ObjPrimi( Ray, Hit );

    ///// 材質

    switch( Hit.Mat )
    {
      case 0: Ray = MatSkyer( Ray, Hit ); return;
      case 1: Ray = MatMirro( Ray, Hit ); break;
      case 2: Ray = MatWater( Ray, Hit ); break;
      case 3: Ray = MatDiffu( Ray, Hit ); break;
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
