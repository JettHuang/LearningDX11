// DX11 Include Files
//

#pragma once

#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>

#include <DirectXMath.h>
#include <DirectXColors.h>

#include <winerror.h>

using namespace DirectX;


// required libraries for linking DX11.0 with App
// d3d11.lib		(for DX11.0 core)
// d3dcompiler.lib	(for D3DCompile and D3DReflect)
// dxgi.lib			(for IDXGIAdapter1 and IDXGIOutput)
// dxguid.lib		(for IID_ID3D11ShaderReflection)

#pragma comment( lib, "d3d11.lib" )
#pragma comment( lib, "dxgi.lib" )
#pragma comment( lib, "d3dcompiler.lib" )
#pragma comment( lib, "dxguid.lib" )
