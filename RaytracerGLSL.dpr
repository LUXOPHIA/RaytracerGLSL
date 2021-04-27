program RaytracerGLSL;

uses
  System.StartUpCopy,
  FMX.Forms,
  Main in 'Main.pas' {Form1},
  LUX.FMX.Forms in '_LIBRARY\LUXOPHIA\LUX\FMX\LUX.FMX.Forms.pas',
  LUX.GPU.OpenGL.Render_ in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Render_.pas',
  LUX.GPU.OpenGL.Scener in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Scener.pas',
  LUX.GPU.OpenGL.Shaper in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Shaper.pas',
  LUX.GPU.OpenGL.Camera in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Camera.pas',
  LUX.GPU.OpenGL.Inform in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Inform.pas',
  LUX.GPU.OpenGL.Matery in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Matery.pas',
  LUX.GPU.OpenGL in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.pas',
  LUX.GPU.OpenGL.Shaper.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Shaper\LUX.GPU.OpenGL.Shaper.Preset.pas',
  LUX.GPU.OpenGL.Matery.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Matery\LUX.GPU.OpenGL.Matery.Preset.pas',
  LUX.GPU.OpenGL.Atom.Engine in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Engine.pas',
  LUX.GPU.OpenGL.Atom.Framer in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Framer.pas',
  LUX.GPU.OpenGL.Atom in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.pas',
  LUX.GPU.OpenGL.Atom.Porter in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Porter.pas',
  LUX.GPU.OpenGL.Atom.Progra in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Progra.pas',
  LUX.GPU.OpenGL.Atom.Shader in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Shader.pas',
  LUX.GPU.OpenGL.Atom.Buffer in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Buffer.pas',
  LUX.GPU.OpenGL.Atom.Chaner in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Chaner.pas',
  LUX.GPU.OpenGL.Render in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\_FMX\LUX.GPU.OpenGL.Render.pas',
  LUX.GPU.OpenGL.Viewer in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\_FMX\LUX.GPU.OpenGL.Viewer.pas' {GLViewer: TFrame},
  LUX.GPU.OpenGL.Window in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\_FMX\LUX.GPU.OpenGL.Window.pas',
  LUX.Data.Tree in '_LIBRARY\LUXOPHIA\LUX\Data\LUX.Data.Tree.pas',
  LUX.GPU.OpenGL.Comput in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\LUX.GPU.OpenGL.Comput.pas',
  LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D1 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\PixBuf\LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D1.pas',
  LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D2 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\PixBuf\LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D2.pas',
  LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D3 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\PixBuf\LUX.GPU.OpenGL.Atom.Buffer.PixBuf.D3.pas',
  LUX.GPU.OpenGL.Atom.Buffer.StoBuf in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\LUX.GPU.OpenGL.Atom.Buffer.StoBuf.pas',
  LUX.GPU.OpenGL.Atom.Buffer.UniBuf in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\LUX.GPU.OpenGL.Atom.Buffer.UniBuf.pas',
  LUX.GPU.OpenGL.Atom.Buffer.VerBuf in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\LUX.GPU.OpenGL.Atom.Buffer.VerBuf.pas',
  LUX.GPU.OpenGL.Atom.Buffer.EleBuf in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\LUX.GPU.OpenGL.Atom.Buffer.EleBuf.pas',
  LUX.GPU.OpenGL.Atom.Buffer.PixBuf in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Buffer\LUX.GPU.OpenGL.Atom.Buffer.PixBuf.pas',
  LUX.FMX.Controls in '_LIBRARY\LUXOPHIA\LUX\FMX\LUX.FMX.Controls.pas',
  LUX.GPU.OpenGL.Atom.Textur in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Textur.pas',
  LUX.GPU.OpenGL.Atom.Textur.D2 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Textur\LUX.GPU.OpenGL.Atom.Textur.D2.pas',
  LUX.GPU.OpenGL.Atom.Textur.D3 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Textur\LUX.GPU.OpenGL.Atom.Textur.D3.pas',
  LUX.GPU.OpenGL.Atom.Textur.D1 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Textur\LUX.GPU.OpenGL.Atom.Textur.D1.pas',
  LUX.GPU.OpenGL.Matery.Textur.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Matery\Textur\_FMX\LUX.GPU.OpenGL.Matery.Textur.Preset.pas',
  LUX.GPU.OpenGL.Atom.Imager in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\LUX.GPU.OpenGL.Atom.Imager.pas',
  LUX.GPU.OpenGL.Atom.Imager.D2 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Imager\LUX.GPU.OpenGL.Atom.Imager.D2.pas',
  LUX.GPU.OpenGL.Atom.Imager.D3 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Imager\LUX.GPU.OpenGL.Atom.Imager.D3.pas',
  LUX.GPU.OpenGL.Atom.Imager.D1 in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Imager\LUX.GPU.OpenGL.Atom.Imager.D1.pas',
  LUX.GPU.OpenGL.Atom.Imager.D1.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Imager\D1\_FMX\LUX.GPU.OpenGL.Atom.Imager.D1.Preset.pas',
  LUX.GPU.OpenGL.Atom.Imager.D2.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Imager\D2\_FMX\LUX.GPU.OpenGL.Atom.Imager.D2.Preset.pas',
  LUX.GPU.OpenGL.Atom.Imager.D3.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Imager\D3\_FMX\LUX.GPU.OpenGL.Atom.Imager.D3.Preset.pas',
  LUX.GPU.OpenGL.Atom.Textur.D3.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Textur\D3\LUX.GPU.OpenGL.Atom.Textur.D3.Preset.pas',
  LUX.GPU.OpenGL.Atom.Textur.D2.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Textur\D2\LUX.GPU.OpenGL.Atom.Textur.D2.Preset.pas',
  LUX.GPU.OpenGL.Atom.Textur.D1.Preset in '_LIBRARY\LUXOPHIA\LUX.GPU.OpenGL\Atom\Textur\D1\LUX.GPU.OpenGL.Atom.Textur.D1.Preset.pas',
  LUX.Color in '_LIBRARY\LUXOPHIA\LUX\Color\LUX.Color.pas',
  LUX.Random.Xorshift in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.Xorshift.pas',
  LUX.Random.Xoshiro in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.Xoshiro.pas',
  LUX.Random.LCG in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.LCG.pas',
  LUX.Random in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.pas',
  LUX.Random.Xoshiro.B64.P512 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\B64\LUX.Random.Xoshiro.B64.P512.pas',
  LUX.Random.Xoshiro.B64.P256 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\B64\LUX.Random.Xoshiro.B64.P256.pas',
  LUX.Random.Xoshiro.B64.P128 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\B64\LUX.Random.Xoshiro.B64.P128.pas',
  LUX.Random.Xoshiro.B32.P64 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\B32\LUX.Random.Xoshiro.B32.P64.pas',
  LUX.Random.Xoshiro.B32.P128 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\B32\LUX.Random.Xoshiro.B32.P128.pas',
  LUX.Random.WELL.P512 in '_LIBRARY\LUXOPHIA\LUX\Random\WELL\LUX.Random.WELL.P512.pas',
  LUX.Random.WELL.P44497 in '_LIBRARY\LUXOPHIA\LUX\Random\WELL\LUX.Random.WELL.P44497.pas',
  LUX.Random.WELL.P19937 in '_LIBRARY\LUXOPHIA\LUX\Random\WELL\LUX.Random.WELL.P19937.pas',
  LUX.Random.WELL.P1024 in '_LIBRARY\LUXOPHIA\LUX\Random\WELL\LUX.Random.WELL.P1024.pas',
  LUX.Random.PCG.B64 in '_LIBRARY\LUXOPHIA\LUX\Random\PCG\LUX.Random.PCG.B64.pas',
  LUX.Random.PCG.B32 in '_LIBRARY\LUXOPHIA\LUX\Random\PCG\LUX.Random.PCG.B32.pas',
  LUX.Random.PCG.B16 in '_LIBRARY\LUXOPHIA\LUX\Random\PCG\LUX.Random.PCG.B16.pas',
  LUX.Random.PCG.B08 in '_LIBRARY\LUXOPHIA\LUX\Random\PCG\LUX.Random.PCG.B08.pas',
  LUX.Random.WELL in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.WELL.pas',
  LUX.Random.SFMT in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.SFMT.pas',
  LUX.Random.PCG in '_LIBRARY\LUXOPHIA\LUX\Random\LUX.Random.PCG.pas',
  LUX.Random.Xoshiro.B64 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\LUX.Random.Xoshiro.B64.pas',
  LUX.Random.Xoshiro.B32 in '_LIBRARY\LUXOPHIA\LUX\Random\Xoshiro\LUX.Random.Xoshiro.B32.pas',
  LUX.Random.SFMT.P44497 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P44497.pas',
  LUX.Random.SFMT.P86243 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P86243.pas',
  LUX.Random.SFMT.P132049 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P132049.pas',
  LUX.Random.SFMT.P216091 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P216091.pas',
  LUX.Random.SFMT.P607 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P607.pas',
  LUX.Random.SFMT.P1279 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P1279.pas',
  LUX.Random.SFMT.P2281 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P2281.pas',
  LUX.Random.SFMT.P4253 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P4253.pas',
  LUX.Random.SFMT.P11213 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P11213.pas',
  LUX.Random.SFMT.P19937 in '_LIBRARY\LUXOPHIA\LUX\Random\SFMT\LUX.Random.SFMT.P19937.pas',
  LUX.Data.Dictionary in '_LIBRARY\LUXOPHIA\LUX\Data\Dictionary\LUX.Data.Dictionary.pas',
  LUX.Data.Grid.T1 in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.T1.pas',
  LUX.Data.Grid.T2.D1 in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.T2.D1.pas',
  LUX.Data.Grid.T2 in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.T2.pas',
  LUX.Data.Grid.T3.D3 in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.T3.D3.pas',
  LUX.Data.Grid.T3 in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.T3.pas',
  LUX.Data.Grid in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.pas',
  LUX.Data.Grid.T1.D1 in '_LIBRARY\LUXOPHIA\LUX\Data\Grid\LUX.Data.Grid.T1.D1.pas',
  LUX.Color.Grid.D2.Preset in '_LIBRARY\LUXOPHIA\LUX\Color\_FMX\LUX.Color.Grid.D2.Preset.pas',
  LUX.Color.Grid.D3 in '_LIBRARY\LUXOPHIA\LUX\Color\_FMX\LUX.Color.Grid.D3.pas',
  LUX.Color.Grid.D1 in '_LIBRARY\LUXOPHIA\LUX\Color\_FMX\LUX.Color.Grid.D1.pas',
  LUX.Color.Grid.D1.Preset in '_LIBRARY\LUXOPHIA\LUX\Color\_FMX\LUX.Color.Grid.D1.Preset.pas',
  LUX.Color.Grid.D2 in '_LIBRARY\LUXOPHIA\LUX\Color\_FMX\LUX.Color.Grid.D2.pas',
  LUX.Color.Format.HDR in '_LIBRARY\LUXOPHIA\LUX\Color\LUX.Color.Format.HDR.pas',
  LUX.D5 in '_LIBRARY\LUXOPHIA\LUX\LUX.D5.pas',
  LUX.DN in '_LIBRARY\LUXOPHIA\LUX\LUX.DN.pas',
  LUX in '_LIBRARY\LUXOPHIA\LUX\LUX.pas',
  LUX.D1 in '_LIBRARY\LUXOPHIA\LUX\LUX.D1.pas',
  LUX.D2 in '_LIBRARY\LUXOPHIA\LUX\LUX.D2.pas',
  LUX.D2x2 in '_LIBRARY\LUXOPHIA\LUX\LUX.D2x2.pas',
  LUX.D2x4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D2x4.pas',
  LUX.D2x4x4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D2x4x4.pas',
  LUX.D3 in '_LIBRARY\LUXOPHIA\LUX\LUX.D3.pas',
  LUX.D3x3 in '_LIBRARY\LUXOPHIA\LUX\LUX.D3x3.pas',
  LUX.D3x4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D3x4.pas',
  LUX.D3x4x4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D3x4x4.pas',
  LUX.D4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D4.pas',
  LUX.D4x4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D4x4.pas',
  LUX.D4x4x4 in '_LIBRARY\LUXOPHIA\LUX\LUX.D4x4x4.pas',
  LUX.Curve.BSpline in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.BSpline.pas',
  LUX.Curve.CatRom in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.CatRom.pas',
  LUX.Curve.D2 in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.D2.pas',
  LUX.Curve in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.pas',
  LUX.Curve.Poly in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.Poly.pas',
  LUX.Curve.Bezier.D2 in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.Bezier.D2.pas',
  LUX.Curve.Bezier in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.Bezier.pas',
  LUX.Curve.BSpline.D2 in '_LIBRARY\LUXOPHIA\LUX\Curve\LUX.Curve.BSpline.D2.pas',
  LUX.FMX.Messaging.Win in '_LIBRARY\LUXOPHIA\LUX\FMX\LUX.FMX.Messaging.Win.pas',
  LUX.FMX.Pratform in '_LIBRARY\LUXOPHIA\LUX\FMX\LUX.FMX.Pratform.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TForm1, Form1);
  Application.Run;
end.
