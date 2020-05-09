#pragma once


using namespace System;

using namespace OpenTK;
using namespace OpenTK::Graphics;
using namespace OpenTK::Graphics::OpenGL;

public ref class OpenTK3D : public GameWindow
{
private:
	int VertexBufferObject;
protected:
	void OnLoad(EventArgs ^e) override{
		

		VertexBufferObject = GL::GenBuffer();
		//GL::BufferData(BufferTarget::ArrayBuffer, 100, verti, BufferUsageHint::StreamDraw);

		GL::ClearColor(0.6f, 0.6f, 0.1f, 1.0f);
		GL::MatrixMode(MatrixMode::Modelview);
		GL::LoadIdentity();









		GameWindow::OnLoad(e);
	}
	 void OnUnload(EventArgs ^e) override
	{
		GL::BindBuffer(BufferTarget::ArrayBuffer, 0);
		GL::DeleteBuffer(VertexBufferObject);
		GameWindow::OnUnload(e);
	}

	 void OnRenderFrame(FrameEventArgs ^e) override
	{
		GameWindow::OnRenderFrame(e);

		GL::Clear(ClearBufferMask::ColorBufferBit);
		GL::ClearColor(Color::CornflowerBlue);
		

		GL::Begin(PrimitiveType::Lines);

		GL::Vertex3(1, 0, 0);
	

		GL::Vertex3(-0.5f, 0.3f, 0.0f);
	
		GL::End();


		GL::Flush();
		Context->SwapBuffers();
		
	}
	 void OnResize(EventArgs ^e)override
	{
		GL::Viewport(0, 0, Width, Height);
		GameWindow::OnResize(e);
	}

public:
	OpenTK3D(int width, int height, String ^ title) : GameWindow(width, height, GraphicsMode::Default, title) { }


};

