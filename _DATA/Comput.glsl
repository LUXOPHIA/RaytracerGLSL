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

float Pow2( in float X )
{
  return X * X;
}

//------------------------------------------------------------------------------

float length2( in vec3 V )
{
  return Pow2( V.x ) + Pow2( V.y ) + Pow2( V.z );
}

//------------------------------------------------------------------------------

int MinI( in float A, in float B, in float C )
{
  if ( A <= B ) {
    if ( A <= C ) return 0; else return 2;
  } else {
    if ( B <= C ) return 1; else return 2;
  }
}

//------------------------------------------------------------------------------

int MaxI( in float A, in float B, in float C )
{
  if ( A >= B ) {
    if ( A >= C ) return 0; else return 2;
  } else {
    if ( B >= C ) return 1; else return 2;
  }
}

//------------------------------------------------------------------------------

vec2 VecToSky( in vec3 Vec )
{
  vec2 Result;

  Result.x = ( Pi - atan( -Vec.x, -Vec.z ) ) / Pi2;
  Result.y =        acos(  Vec.y           ) / Pi ;

  return Result;
}

//------------------------------------------------------------------------------

vec3 ToneMap( in vec3 Color, in float White )
{
  return clamp( Color * ( 1 + Color / White ) / ( 1 + Color ), 0, 1 );
}

//------------------------------------------------------------------------------

vec3 GammaCorrect( in vec3 Color, in float Gamma )
{
  vec3 Result;

  float G = 1 / Gamma;

  Result.r = pow( Color.r, G );
  Result.g = pow( Color.g, G );
  Result.b = pow( Color.b, G );

  return Result;
}

//------------------------------------------------------------------------------

float Fresnel( in vec3 Vec, in vec3 Nor, in float IOR )
{
  float N2, C, G2, F0;

  N2 = Pow2( IOR );
  C  = dot( Nor, -Vec );
  G2 = N2 + Pow2( C ) - 1;
  if ( G2 < 0 ) return 1;

  //float N2C, G;
  //N2C = N2 * C;
  //G   = sqrt( G2 );
  //return ( Pow2( (   C - G ) / (   C + G ) )
  //       + Pow2( ( N2C - G ) / ( N2C + G ) ) ) / 2;

  F0 = Pow2( ( IOR - 1 ) / ( IOR + 1 ) );
  return F0 + ( 1 - F0 ) * pow( 1 - C, 5 );
}

//------------------------------------------------------------------------------

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

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TdFloat

///////////////////////////////////////////////////////////////////////// 演算子

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

///////////////////////////////////////////////////////////////////////// 演算子

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

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 幾何学

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

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【外部変数】

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

//############################################################################## ■

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

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【内部変数】

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【物体】

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

//------------------------------------------------------------------------------

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

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【材質】

TRay MatSkyer( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec = Ray.Vec;
  Result.Pos = Ray.Pos;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi + texture( _Textur, VecToSky( Ray.Vec.xyz ) ).rgb;

  return Result;
}

//------------------------------------------------------------------------------

TRay MatMirro( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );
  Result.Pos = Hit.Pos + FLOAT_EPS2 * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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

//##############################################################################

TdFloat MathFunc( TdVec3 P )
{
  return Sub( Add( Add( Pow2( P.x ), Pow2( P.y ) ), Pow2( P.z ) ), 1 );
}

vec3 MathGrad( TdVec3 P )
{
  vec3 Result;

  P.x.d = 1;  P.y.d = 0;  P.z.d = 0;

  Result.x = MathFunc( P ).d;

  P.x.d = 0;  P.y.d = 1;  P.z.d = 0;

  Result.y = MathFunc( P ).d;

  P.x.d = 0;  P.y.d = 0;  P.z.d = 1;

  Result.z = MathFunc( P ).d;

  return Result;
}

//------------------------------------------------------------------------------

bool HitFunc( in TRay Ray, in float T2d, inout TdFloat T, out TdVec3 P )
{
  const uint LoopN = 16;

  float   Tds, Td;
  uint    N;
  TdFloat F;

  Tds = 0;

  for ( N = 1; N <= LoopN; N++ )
  {
    P = Add( Mul( Ray.Vec.xyz, T ), Ray.Pos.xyz );

    F = MathFunc( P );

    if ( abs( F.o ) < 0.001 ) return ( 0 < T.o );

    Td = -F.o / F.d;

    Tds += Td;

    if ( abs( Tds ) > T2d ) return false;

    T.o += Td;
  }

  return false;
}

void ObjImpli( in TRay Ray, inout THit Hit )
{
  const vec3  MinP = vec3( -1, -1, -1 ) - FLOAT_EPS2;
  const vec3  MaxP = vec3( +1, +1, +1 ) + FLOAT_EPS2;
  const float Td   = 0.1;
  const float T2d  = 1.5 * Td/2;

  float   MinT, MaxT, T0;
  int     IncA, OutA;
  TdFloat T;
  TdVec3  P;

  if ( HitAABB( Ray.Pos, Ray.Vec, MinP, MaxP, MinT, MaxT, IncA, OutA ) )
  {
    for ( T0 = max( 0, MinT ); T0 < MaxT + Td; T0 += Td )
    {
      T = TdFloat( T0, 1 );

      if ( HitFunc( Ray, T2d, T, P ) && ( 0 < T.o ) && ( T.o < MaxT ) && ( T.o < Hit.t ) )
      {
        Hit.t   = T.o;
        Hit.Pos = vec4( P.x.o, P.y.o, P.z.o, 1 );
        Hit.Nor = vec4( normalize( MathGrad( P ) ), 0 );
        Hit.Mat = 2;

        break;
      }
    }
  }
}

//------------------------------------------------------------------------------

void Raytrace( inout TRay Ray )
{
  int  L;
  THit Hit;

  for ( L = 1; L <= 8; L++ )
  {
    Hit = THit( FLOAT_MAX, 0, vec4( 0 ), vec4( 0 ) );

    ///// 物体

    ObjImpli( Ray, Hit );
    ObjPlane( Ray, Hit );

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
    E = vec4( 0, 0, 0, 1 );

    S.x = 4.0 * ( _WorkID.x + 0.5 ) / _WorksN.x - 2.0;
    S.y = 1.5 - 3.0 * ( _WorkID.y + 0.5 ) / _WorksN.y;
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
