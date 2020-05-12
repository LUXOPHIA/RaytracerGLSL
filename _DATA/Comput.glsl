#version 430

//#extension GL_ARB_compute_variable_group_size : enable

//layout( local_size_variable ) in;
  layout( local_size_x = 10,
          local_size_y = 10,
          local_size_z =  1 ) in;                                               // スレッドグループサイズの固定設定

////////////////////////////////////////////////////////////////////////////////

  ivec3 _WorkGrupsN = ivec3( gl_NumWorkGroups );                                // スレッドグループ数

//ivec3 _WorkItemsN = ivec3( gl_LocalGroupSizeARB );
  ivec3 _WorkItemsN = ivec3( gl_WorkGroupSize     );                            // グループ内のスレッド数

  ivec3 _WorksN     = _WorkGrupsN * _WorkItemsN;                                // 全スレッド数

  ivec3 _WorkID     = ivec3( gl_GlobalInvocationID );                           // 現スレッドの座標

//############################################################################## ■

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【定数】

const float Pi  = 3.141592653589793;                                            // π
const float Pi2 = Pi * 2.0;                                                     // 2π
const float P2i = Pi / 2.0;                                                     // π/2

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【ルーチン】

float Pow2( in float X )                                                        // 二乗関数
{
  return X * X;
}

//------------------------------------------------------------------------------

float length2( in vec3 V )                                                      // ベクトル長の二乗関数
{
  return Pow2( V.x ) + Pow2( V.y ) + Pow2( V.z );
}

//------------------------------------------------------------------------------

vec2 VecToSky( in vec3 Vec )                                                    // ベクトル→球面座標変換関数
{
  vec2 Result;

  Result.x = ( Pi - atan( -Vec.x, -Vec.z ) ) / Pi2;
  Result.y =        acos(  Vec.y           ) / Pi ;

  return Result;
}

//------------------------------------------------------------------------------

vec3 ToneMap( in vec3 Color, in float White )                                   // トーンマッピング関数
{
  return clamp( Color * ( 1 + Color / White ) / ( 1 + Color ), 0, 1 );
}

//------------------------------------------------------------------------------

vec3 GammaCorrect( in vec3 Color, in float Gamma )                              // ガンマ補正関数
{
  vec3 Result;

  float G = 1 / Gamma;

  Result.r = pow( Color.r, G );
  Result.g = pow( Color.g, G );
  Result.b = pow( Color.b, G );

  return Result;
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【外部変数】

writeonly uniform image2D _Imager;                                              // イメージオブジェクトの宣言

layout( std430 ) buffer TCamera                                                 // SSBO の宣言
{
  layout( row_major ) mat4 _Camera;                                             // 4x4行列変数の宣言
};

uniform sampler2D _Textur;                                                      // テクスチャオブジェクトの宣言

//############################################################################## ■

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRay

struct TRay                                                                     // 光線の構造体宣言
{
  vec4 Pos;                                                                     // 出射点
  vec4 Vec;                                                                     // 出射ベクトル
  vec3 Wei;                                                                     // 重み
  vec3 Emi;                                                                     // 流入輝度
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THit

struct THit                                                                     // 光線衝突点の構造体宣言
{
  float t;                                                                      // 光線の出射点から衝突点までの距離
  int   Mat;                                                                    // 衝突点のマテリアルＩＤ
  vec4  Pos;                                                                    // 衝突点の位置
  vec4  Nor;                                                                    // 衝突点の法線
};

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【内部変数】

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【物体】

void ObjPlane( in TRay Ray, inout THit Hit )                                    // 無限平面と光線の交差判定関数
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
      Hit.Mat = 1;
    }
  }
}

//------------------------------------------------------------------------------

void ObjSpher( in TRay Ray, inout THit Hit )                                    // 球体と光線の交差判定関数
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
      Hit.Mat = 1;
    }
  }
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【材質】

float _EmitShift = 0.0001;                                                      // 反射シフト量

////////////////////////////////////////////////////////////////////////////////

TRay MatSkyer( in TRay Ray, in THit Hit )                                       // スカイドームのマテリアル関数
{
  TRay Result;

  Result.Vec = Ray.Vec;
  Result.Pos = Ray.Pos;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi + texture( _Textur, VecToSky( Ray.Vec.xyz ) ).rgb;

  return Result;
}

//------------------------------------------------------------------------------

TRay MatMirro( in TRay Ray, in THit Hit )                                       // 鏡面反射のマテリアル関数
{
  TRay Result;

  Result.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );
  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//##############################################################################

void Raytrace( inout TRay Ray )                                                 // レイトレース関数
{
  THit Hit;

  Hit = THit( 10000, 0, vec4( 0 ), vec4( 0 ) );                                 // 衝突点を無限遠に初期化

  ///// 物体

  ObjSpher( Ray, Hit );                                                         // 球体と光線の交差判定
  ObjPlane( Ray, Hit );                                                         // 平面と光線の交差判定

  ///// 材質

  switch( Hit.Mat )                                                             // 衝突した材質に応じて光線の再生産過程を分岐
  {
    case 0: Ray = MatSkyer( Ray, Hit ); return;                                 // 空に当たったら終了
    case 1: Ray = MatMirro( Ray, Hit ); break;                                  // 鏡面素材に当たったら光線を再生産
  }
}

//------------------------------------------------------------------------------

void main()
{
  vec4 E, S;
  TRay R;
  vec3 A, P;

  E = vec4( 0, 0, 0, 1 );                                                       // 視点位置座標

  S.x = 4.0 * ( _WorkID.x + 0.5 ) / _WorksN.x - 2.0;                            // スクリーン上のピクセル位置座標
  S.y = 1.5 - 3.0 * ( _WorkID.y + 0.5 ) / _WorksN.y;
  S.z = -2;
  S.w = 1;

  R.Pos = _Camera * E;                                                          // 光線の出射点を視点
  R.Vec = _Camera * normalize( S - E );                                         // スクリーンのピルセルに向かって光線を発射
  R.Wei = vec3( 1 );                                                            // 初期ウェイトは１
  R.Emi = vec3( 0 );                                                            // 初期輝度はゼロ

  Raytrace( R );                                                                // レイトレース

  A = R.Wei * R.Emi;                                                            // 最終的な流入輝度と光線のウェイトをかけ算

  P = GammaCorrect( ToneMap( A, 10 ), 2.2 );                                    // トーンマッピングとガンマ補正

  imageStore( _Imager, _WorkID.xy, vec4( P, 1 ) );                              // イメージオブジェクトに書き込み
}

//############################################################################## ■
