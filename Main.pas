unit Main;

interface //#################################################################### ■

uses
  System.SysUtils, System.Types, System.UITypes, System.Classes, System.Variants,
  FMX.Types, FMX.Controls, FMX.Forms, FMX.Graphics, FMX.Dialogs,
  FMX.Controls.Presentation, FMX.StdCtrls, FMX.Objects, FMX.TabControl, FMX.ScrollBox, FMX.Memo, LUX.FMX.Controls,
  Winapi.OpenGL, Winapi.OpenGLext,
  LUX, LUX.D1, LUX.D2, LUX.D3, LUX.D4, LUX.M4,
  LUX.Random.Xoshiro.B32,
  LUX.Random.Xoshiro.B32.P128,
  LUX.GPU.OpenGL,
  LUX.GPU.OpenGL.Atom.Buffer.StoBuf,
  LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D1,
  LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D2,
  LUX.GPU.OpenGL.Atom.Imager,
  LUX.GPU.OpenGL.Atom.Imager.D1.Preset,
  LUX.GPU.OpenGL.Atom.Imager.D2.Preset,
  LUX.GPU.OpenGL.Atom.Textur.D1.Preset,
  LUX.GPU.OpenGL.Atom.Textur.D2.Preset,
  LUX.GPU.OpenGL.Comput;

type
  TForm1 = class(TForm)                                                         // アプリウィンドウクラスの宣言
    TabControl1: TTabControl;                                                   // タブクラスのインスタ変数
      TabItem1: TTabItem;                                                       // タブページクラスのインスタ変数
        Image1: TImage;                                                         // 画像表示クラスのインスタ変数
      TabItem2: TTabItem;                                                       // タブページクラスのインスタ変数
        Memo1: TMemo;                                                           // テキスト表示クラスのインスタ変数
    Timer1: TTimer;                                                             // 定期実行クラスのインスタ変数
    procedure FormCreate(Sender: TObject);                                                               // アプリ起動イベントのメソッド
    procedure FormDestroy(Sender: TObject);                                                              // アプリ終了イベントのメソッド
    procedure Image1MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);  // マウスボタンＯＮイベントのメソッド
    procedure Image1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);                        // マウス移動イベントのメソッド
    procedure Image1MouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);    // マウスボタンＯＦＦイベントのメソッド
    procedure Timer1Timer(Sender: TObject);                                                              // 定義実行イベントのメソッド
  private
    { private 宣言 }
    _MouseS :TShiftState;                                                       // マウスボタンの状態集合の変数
    _MouseP :TSingle2D;                                                         // マウスポインタの座標の変数
    _MouseA :TSingle2D;                                                         // マウスポインタの累積座標の変数
  public
    { public 宣言 }
    _ImageX :Integer;                                                           // レンダリング画像の横ピクセル数の変数
    _ImageY :Integer;                                                           // レンダリング画像の縦ピクセル数の変数
    _Comput :TGLComput;                                                         // コンピュートシェーダクラスのインスタ変数
    _Imager :TGLCelIma2D_TAlphaColorF;                                          // 書き換え可能なイメージクラスのインスタ変数
    _Camera :TGLStoBuf<TSingleM4>;                                              // 行列配列クラスのインスタ変数
    _Textur :TGLCelTex2D_TAlphaColorF;                                          // 線形補間可能なテクスチャクラスのインスタ変数
    ///// メソッド
    procedure InitComput;                                                       // コンピュートシェーダの初期化メソッド
  end;

var
  Form1: TForm1;                                                                // アプリウィンドウクラスのインスタ変数

implementation //############################################################### ■

uses System.Math,
     LUX.Random.Xoshiro;

{$R *.fmx}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& private

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& public

/////////////////////////////////////////////////////////////////////// メソッド

procedure TForm1.InitComput;                                                    // コンピュートシェーダの初期化メソッド
begin
     _Comput.ItemsX := 10;                                                      // スレッド数をＸ方向に１０個ずつの束にする
     _Comput.ItemsY := 10;                                                      // スレッド数をＹ方向に１０個ずつの束にする
     _Comput.ItemsZ :=  1;                                                      // スレッド数をＺ方向に　１個ずつの束にする

     _Comput.WorksX := _ImageX;                                                 // Ｘ方向のスレッド数
     _Comput.WorksY := _ImageY;                                                 // Ｙ方向のスレッド数
     _Comput.WorksZ :=       1;                                                 // Ｚ方向のスレッド数

     _Comput.ShaderC.Source.LoadFromFile( '..\..\_DATA\Comput.glsl' );          // コンピュートシェーダのソースを読み込む

     with Memo1.Lines do                                                        // Memo1 の Lines プロパティに対して･･･
     begin
          Assign( _Comput.ShaderC.Errors );                                     // コンピュートシェーダからコンパイルメッセージをコピー

          if Count > 0 then TabControl1.TabIndex := 1;                          // メッセージが存在する場合はタブページの２ページ目を表示
     end;

     _Comput.Imagers.Add( '_Imager', _Imager );                                 // コンピュートシェーダにイメージクラスを登録
     _Comput.Buffers.Add( 'TCamera', _Camera );                                 // コンピュートシェーダに行列配列クラスを登録
     _Comput.Texturs.Add( '_Textur', _Textur );                                 // コンピュートシェーダにテクスチャクラスを登録
end;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

procedure TForm1.FormCreate(Sender: TObject);                                   // アプリ起動イベントのメソッド
begin
     Image1.AutoCapture := True;                                                // Image1 上でマウスポインタを常に補足

     _ImageX := 800;                                                            // レンダリング画像の横ピクセル数を設定
     _ImageY := 600;                                                            // レンダリング画像の縦ピクセル数を設定

     _Comput := TGLComput               .Create;                                // コンピュートシェーダの生成
     _Imager := TGLCelIma2D_TAlphaColorF.Create;                                // イメージの生成
     _Camera := TGLStoBuf<TSingleM4>    .Create( GL_DYNAMIC_DRAW );             // 行列配列の生成
     _Textur := TGLCelTex2D_TAlphaColorF.Create;                                // テクスチャの生成

     InitComput;                                                                // コンピュートシェーダの初期化

     _Imager.Grid.CellsX := _ImageX;                                            // イメージの横ピクセル数を設定
     _Imager.Grid.CellsY := _ImageY;                                            // イメージの横ピクセル数を設定

     _Textur.Imager.LoadFromFileHDR( '..\..\_DATA\Luxo-Jr_2000x1000.hdr' );     // テクスチャにＨＤＲＩを読み込む
end;

procedure TForm1.FormDestroy(Sender: TObject);                                  // アプリ終了イベントのメソッド
begin
     _Comput.Free;                                                              // コンピュートシェーダの廃棄
     _Imager.Free;                                                              // イメージの廃棄
     _Camera.Free;                                                              // 行列配列の廃棄
     _Textur.Free;                                                              // テクスチャの廃棄
end;

////////////////////////////////////////////////////////////////////////////////

procedure TForm1.Timer1Timer(Sender: TObject);                                  // 定期実行クラスのメソッド
begin
     _Camera[ 0 ] := TSingleM4.RotateY( DegToRad( -_MouseA.X ) )                //    Ｙ軸回転行列
                   * TSingleM4.RotateX( DegToRad( -_MouseA.Y ) )                // × Ｘ軸回転行列
                   * TSingleM4.Translate( 0, 0, 3 );                            // × 平行移動行列 を行列配列へ代入

     _Comput.Run;                                                               // コンピュートシェーダを実行

     _Imager.CopyTo( Image1.Bitmap );                                           // イメージを画像表示ＵＩへコピー
end;

//------------------------------------------------------------------------------

procedure TForm1.Image1MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);  // マウスボタンＯＮイベントのメソッド
begin
     _MouseS := Shift;                                                          // マウスボタンの状態集合を取得
     _MouseP := TSingle2D.Create( X, Y );                                       // マウスポインタ座標を取得
end;

procedure TForm1.Image1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);  // マウス移動イベントのメソッド
var
   P :TSingle2D;                                                                // 現在のマウスポインタ座標の変数
begin
     if ssLeft in _MouseS then                                                  // マウスボタンの状態集合に左ボタンが含まれていた場合
     begin
          P := TSingle2D.Create( X, Y );                                        // 現在のマウスポインタ座標を取得

          _MouseA := _MouseA + ( P - _MouseP );                                 // マウスポインタの移動差分を累積

          _MouseP := P;                                                         // 現在のマウスポインタ座標を過去の座標とする
     end;
end;

procedure TForm1.Image1MouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);  // マウスボタンＯＦＦイベントのメソッド
begin
     Image1MouseMove( Sender, Shift, X, Y );                                    // マウス移動イベントを実行

     _MouseS := [];                                                             // マウスボタンの状態集合の変数に空集合を代入
end;

end. //######################################################################### ■
