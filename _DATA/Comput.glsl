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

const float Pi        = 3.141592653589793;
const float Pi2       = Pi * 2.0;
const float P2i       = Pi / 2.0;
const float FLOAT_MAX = 3.402823e+38;

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

int MinI( float A_, float B_, float C_ )
{
    if ( A_ <= B_ )
    {
        if ( A_ <= C_ ) return 0;
                   else return 2;
    }
    else
    {
        if ( B_ <= C_ ) return 1;
                   else return 2;
    }
}

int MinI( vec3 V_ )
{
    return MinI( V_.x, V_.y, V_.z );
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
  // float N = Pow2( IOR );
  // float C = dot( Vec, Nor );
  // float G = sqrt( N + Pow2( C ) - 1 );
  // float NC = N * C;
  // return ( Pow2( (  C + G ) / (  C - G ) )
  //        + Pow2( ( NC + G ) / ( NC - G ) ) ) / 2;

  float R = Pow2( ( IOR - 1 ) / ( IOR + 1 ) );
  float C = clamp( dot( Vec, Nor ), -1, 0 );
  return R + ( 1 - R ) * pow( 1 + C, 5 );
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

  return float( Result ) / 4294967296.0;
}

//------------------------------------------------------------------------------

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

layout( rgba32f ) uniform image3D _Voxels;

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

mat4 _ObjMove = mat4( 1 );
mat4 _ObjMovi = mat4( 1 );

void BeginMove( inout TRay Ray )
{
  _ObjMovi = inverse( _ObjMove );

  Ray.Pos = _ObjMovi * Ray.Pos;
  Ray.Vec = _ObjMovi * Ray.Vec;
}

void EndMove( inout THit Hit )
{
  mat3 ObjMovn = transpose( mat3( _ObjMovi ) );

  Hit.Pos     = _ObjMove * Hit.Pos    ;
  Hit.Nor.xyz =  ObjMovn * Hit.Nor.xyz;
}

////////////////////////////////////////////////////////////////////////////////

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
  BeginMove( Ray );

  float L, B, C, D, t;

  L = length( Ray.Vec.xyz );

  B = dot( Ray.Pos.xyz, Ray.Vec.xyz / L );
  C = length2( Ray.Pos.xyz ) - 1;

  D = Pow2( B ) - C;

  if ( D > 0 )
  {
    t = ( -B - sign( C ) * sqrt( D ) ) / L;

    if ( ( 0 < t ) && ( t < Hit.t ) )
    {
      Hit.t   = t;
      Hit.Pos = Ray.Pos + t * Ray.Vec;
      Hit.Nor = Hit.Pos;
      Hit.Mat = 2;

      EndMove( Hit );
    }
  }
}

//------------------------------------------------------------------------------

bool Slab( in float Pos, in float Vec,
           in float Cen, in float Siz,
           inout float MinT, inout float MaxT )
{
  float S, T0, T1;

  S = sign( Vec );

  T0 = ( Cen - S * Siz/2 - Pos ) / Vec;
  T1 = ( Cen + S * Siz/2 - Pos ) / Vec;

  if ( T1 < MaxT ) { MaxT = T1; }

  if ( MinT < T0 ) { MinT = T0; return true; } else { return false; }
}

void ObjRecta( in TRay Ray, inout THit Hit, in vec3 Cen, in vec3 Siz )
{
  float MinT, MaxT;
  vec3 Nor;

  MinT = -FLOAT_MAX;
  MaxT = +FLOAT_MAX;

  if ( Slab( Ray.Pos.x, Ray.Vec.x, Cen.x, Siz.x, MinT, MaxT ) ) Nor = vec3( -sign( Ray.Vec.x ), 0, 0 );
  if ( Slab( Ray.Pos.y, Ray.Vec.y, Cen.y, Siz.y, MinT, MaxT ) ) Nor = vec3( 0, -sign( Ray.Vec.y ), 0 );
  if ( Slab( Ray.Pos.z, Ray.Vec.z, Cen.z, Siz.z, MinT, MaxT ) ) Nor = vec3( 0, 0, -sign( Ray.Vec.z ) );

  if( ( MinT < MaxT ) && ( 0 < MinT ) && ( MinT < Hit.t ) )
  {
    Hit.t   = MinT;
    Hit.Pos = Ray.Pos + MinT * Ray.Vec;
    Hit.Nor = vec4( Nor, 0 );
    Hit.Mat = 1;
  }
}

bool HitRecta( in TRay Ray, out float HitT, in vec3 Cen, in vec3 Siz )
{
  float MinT, MaxT;

  MinT = -FLOAT_MAX;
  MaxT = +FLOAT_MAX;

  Slab( Ray.Pos.x, Ray.Vec.x, Cen.x, Siz.x, MinT, MaxT );
  Slab( Ray.Pos.y, Ray.Vec.y, Cen.y, Siz.y, MinT, MaxT );
  Slab( Ray.Pos.z, Ray.Vec.z, Cen.z, Siz.z, MinT, MaxT );

  if( MinT < MaxT ) { HitT = MinT; return true; } else return false;
}

//------------------------------------------------------------------------------

const vec3  _GridsC =  vec3(  0,  0,  0 );
const vec3  _GridsS =  vec3(  2,  2,  2 );
const ivec3 _GridsN = ivec3( 10, 10, 10 );

void ObjGrids( in TRay Ray, inout THit Hit )
{
  float HitT;

  if ( HitRecta( Ray, HitT, _GridsC, 0.9999 * _GridsS ) )
  {
    vec4 HitP = max( HitT, 0 ) * Ray.Vec + Ray.Pos;

    ivec3 Gv = ivec3( sign( Ray.Vec.xyz ) );

    ivec3 Gvs[ 3 ] = { { Gv.x,    0,    0 },
                       {    0, Gv.y,    0 },
                       {    0,    0, Gv.z } };

    vec3 Sd = _GridsS / _GridsN;

    vec3 Tv = Sd / abs( Ray.Vec.xyz );

    vec3 Tvs[ 3 ] = { { Tv.x,    0,    0 },
                      {    0, Tv.y,    0 },
                      {    0,    0, Tv.z } };

    vec3 G = ( HitP.xyz - _GridsC + _GridsS / 2 ) / Sd;

    ivec3 Gi = ivec3( floor( G ) );

    vec3 Gd = G - Gi;

    vec3 Ts;

    if ( isinf( Tv.x ) ) Ts.x = FLOAT_MAX;
                    else Ts.x = Tv.x * ( 0.5 + sign( Ray.Vec.x ) * ( 0.5 - Gd.x ) );

    if ( isinf( Tv.y ) ) Ts.y = FLOAT_MAX;
                    else Ts.y = Tv.y * ( 0.5 + sign( Ray.Vec.y ) * ( 0.5 - Gd.y ) );

    if ( isinf( Tv.z ) ) Ts.z = FLOAT_MAX;
                    else Ts.z = Tv.z * ( 0.5 + sign( Ray.Vec.z ) * ( 0.5 - Gd.z ) );

    float T0 = 0;

    while ( ( 0 <= Gi.x ) && ( Gi.x < _GridsN.x )
         && ( 0 <= Gi.y ) && ( Gi.y < _GridsN.y )
         && ( 0 <= Gi.z ) && ( Gi.z < _GridsN.z ) )
    {
      int K = MinI( Ts );

      float T1 = Ts[ K ];

      vec3 R = Sd / 2 * imageLoad( _Voxels, Gi ).rgb;
      vec3 C = _GridsC + _GridsS * ( ( Gi + 0.5 ) / _GridsN - vec3( 0.5 ) );

      _ObjMove = mat4( R.x,    0,    0,  0,
                         0,  R.y,    0,  0,
                         0,    0,  R.z,  0,
                       C.x,  C.y,  C.z,  1 );

      ObjSpher( Ray, Hit );

      T0 = T1;

      Gi += Gvs[ K ];
      Ts += Tvs[ K ];
    }
  }
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【材質】

float _EmitShift = 0.0001;

////////////////////////////////////////////////////////////////////////////////

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
  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//------------------------------------------------------------------------------

TRay MatWater( inout TRay Ray, in THit Hit )
{
  TRay Result;
  float IOR, F;
  vec4  Nor;

  if( dot( Ray.Vec.xyz, Hit.Nor.xyz ) < 0 )
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
    Result.Pos = Hit.Pos + _EmitShift * Nor;
    Result.Wei = Ray.Wei;
    Result.Emi = Ray.Emi;
  } else {
    Result.Vec = vec4( refract( Ray.Vec.xyz, Nor.xyz, 1 / IOR ), 0 );
    Result.Pos = Hit.Pos - _EmitShift * Nor;
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

  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//##############################################################################

void Raytrace( inout TRay Ray )
{
  THit Hit;

  for ( int L = 1; L <= 5; L++ )
  {
    Hit = THit( FLOAT_MAX, 0, vec4( 0 ), vec4( 0 ) );

    ///// 物体

    ObjGrids( Ray, Hit );

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
  vec4 E, S;
  TRay R;
  vec3 A, C, P;

  _RandSeed = imageLoad( _Seeder, _WorkID.xy );

  if ( _AccumN == 0 ) A = vec3( 0 );
                 else A = imageLoad( _Accumr, _WorkID.xy ).rgb;

  for( uint N = _AccumN+1; N <= _AccumN+16; N++ )
  {
    E = vec4( 0, 0, 0, 1 );
    E.xy += 0.05 * RandCirc();

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
