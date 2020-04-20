# RaytracerGLSL

[OpenGL](https://ja.wikipedia.org/wiki/OpenGL) の [Compute Shader](https://www.khronos.org/opengl/wiki/Compute_Shader) を用いて **[レイトレーシング](https://ja.wikipedia.org/wiki/レイトレーシング)([Ray-tracing](https://en.wikipedia.org/wiki/Ray_tracing_(graphics)))** を実装する方法。金属や水面のフレネル反射や屈折を表現。[環境マッピング](https://ja.wikipedia.org/wiki/環境マッピング)([Environment Mapping](https://en.wikipedia.org/wiki/Reflection_mapping)) には [HDRI](https://ja.wikipedia.org/wiki/ハイダイナミックレンジイメージ) を用い、簡易な トーンマッピング([Tone Mapping](https://en.wikipedia.org/wiki/Tone_mapping)) も導入。

[**[ YouTube 4K ]**](https://youtu.be/NjPYuC4lKfo)　[**[ Vimeo 4K** (original) **]**](https://vimeo.com/270096538)
[![](https://github.com/LUXOPHIA/Raytracer_OpenGL/raw/master/--------/_SCREENSHOT/RaytracerGLSL.png)](https://youtu.be/NjPYuC4lKfo)

### Ray-tracing with Compute Shader
[GLSL](https://ja.wikipedia.org/wiki/GLSL) では関数の[再帰呼び出し](https://ja.wikipedia.org/wiki/再帰#再帰呼出し)が利用できないので、未計算レイを保持する [スタック](https://ja.wikipedia.org/wiki/スタック)([Stack](https://en.wikipedia.org/wiki/Stack_(abstract_data_type))) を独自に実装する必要がある。
ソースコードは以下を参照。
> https://github.com/LUXOPHIA/RaytracerGLSL/blob/master/_DATA/Comput.glsl

----

[![Delphi Starter](https://github.com/delphiusers/FreeDelphi/raw/master/FreeDelphi_350px.png)](https://www.embarcadero.com/jp/products/delphi/starter)
