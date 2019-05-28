﻿unit Main;

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
  TForm1 = class(TForm)
    TabControl1: TTabControl;
      TabItem1: TTabItem;
        Image1: TImage;
      TabItem2: TTabItem;
        Memo1: TMemo;
    Timer1: TTimer;
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure Image1MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
    procedure Image1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);
    procedure Image1MouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
    procedure Timer1Timer(Sender: TObject);
  private
    { private 宣言 }
    _MouseS :TShiftState;
    _MouseP :TSingle2D;
    _MouseA :TSingle2D;
  public
    { public 宣言 }
    _ImageX :Integer;
    _ImageY :Integer;
    _Comput :TGLComput;
    _Seeder :TGLCelIma2D_TInt32u4D;
    _Imager :TGLCelIma2D_TAlphaColorF;
    _Camera :TGLStoBuf<TSingleM4>;
    _Textur :TGLCelTex2D_TAlphaColorF;
    ///// メソッド
    procedure InitComput;
    procedure InitSeeder;
  end;

var
  Form1: TForm1;

implementation //############################################################### ■

uses System.Math,
     LUX.Random.Xoshiro;

{$R *.fmx}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& private

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& public

/////////////////////////////////////////////////////////////////////// メソッド

procedure TForm1.InitComput;
begin
     _Comput.ItemsX := 10;
     _Comput.ItemsY := 10;
     _Comput.ItemsZ :=  1;

     _Comput.WorksX := _ImageX;
     _Comput.WorksY := _ImageY;
     _Comput.WorksZ :=       1;

     _Comput.ShaderC.Source.LoadFromFile( '..\..\_DATA\Comput.glsl' );

     with Memo1.Lines do
     begin
          Assign( _Comput.ShaderC.Errors );

          if Count > 0 then TabControl1.TabIndex := 1;
     end;

     _Comput.Imagers.Add( '_Seeder', _Seeder );
     _Comput.Imagers.Add( '_Imager', _Imager );
     _Comput.Buffers.Add( 'TCamera', _Camera );
     _Comput.Texturs.Add( '_Textur', _Textur );
end;

//------------------------------------------------------------------------------

procedure TForm1.InitSeeder;
var
   R :IRandom32XOS128;
   D :TGLCelPixIter2D<TInt32u4D>;
   X, Y :Integer;
begin
     _Seeder.Grid.CellsX := _ImageX;
     _Seeder.Grid.CellsY := _ImageY;

     R := TRandom32XOS128x64ss.Create;

     D := _Seeder.Grid.Map( GL_WRITE_ONLY );

     for Y := 0 to _ImageY-1 do
     for X := 0 to _ImageX-1 do D.Cells[ X, Y ] := R.DrawSeed;

     D.DisposeOf;
end;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

procedure TForm1.FormCreate(Sender: TObject);
begin
     Image1.AutoCapture := True;

     _ImageX := 800;
     _ImageY := 600;

     _Comput := TGLComput               .Create;
     _Seeder := TGLCelIma2D_TInt32u4D   .Create;
     _Imager := TGLCelIma2D_TAlphaColorF.Create;
     _Camera := TGLStoBuf<TSingleM4>    .Create( GL_DYNAMIC_DRAW );
     _Textur := TGLCelTex2D_TAlphaColorF.Create;

     InitComput;

     InitSeeder;

     _Imager.Grid.CellsX := _ImageX;
     _Imager.Grid.CellsY := _ImageY;

     _Textur.Imager.LoadFromFileHDR( '..\..\_DATA\Luxo-Jr_2000x1000.hdr' );
end;

procedure TForm1.FormDestroy(Sender: TObject);
begin
     _Comput.DisposeOf;
     _Imager.DisposeOf;
     _Camera.DisposeOf;
     _Textur.DisposeOf;
end;

////////////////////////////////////////////////////////////////////////////////

procedure TForm1.Timer1Timer(Sender: TObject);
begin
     _Camera[ 0 ] := TSingleM4.RotateY( DegToRad( -_MouseA.X ) )
                   * TSingleM4.RotateX( DegToRad( -_MouseA.Y ) )
                   * TSingleM4.Translate( 0, 0, 3 );

     _Comput.Run;

     _Imager.CopyTo( Image1.Bitmap );
end;

//------------------------------------------------------------------------------

procedure TForm1.Image1MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
begin
     _MouseS := Shift;
     _MouseP := TSingle2D.Create( X, Y );
end;

procedure TForm1.Image1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);
var
   P :TSingle2D;
begin
     if ssLeft in _MouseS then
     begin
          P := TSingle2D.Create( X, Y );

          _MouseA := _MouseA + ( P - _MouseP );

          _MouseP := P;
     end;
end;

procedure TForm1.Image1MouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
begin
     Image1MouseMove( Sender, Shift, X, Y );

     _MouseS := [];
end;

end. //######################################################################### ■
