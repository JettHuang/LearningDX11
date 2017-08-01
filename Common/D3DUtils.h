// Utils
//

#pragma once

#include "DX11Includes.h"
#include "dxerr.h"
#include <cassert>
#include <vector>
#include <string>


// D3D Error checker
#if defined(DEBUG) || defined(_DEBUG)

#ifndef HR
#define HR(x)						\
	{								\
			HRESULT hr = (x);		\
			if(FAILED(hr))			\
			{						\
				DXTrace(__FILEW__, (DWORD)__LINE__, hr, L#x, true); \
			}														\
	}
#endif // !HR

#else

#ifndef HR
#define HR(x)		(x)
#endif // !HR

#endif

// convenience macro for releasing COM Objects
#define ReleaseCOM(x)	{ if(x) { x->Release(); x = 0; } }

// convenience macro for deleting Objects
#define SafeDelete(x)	{ delete x; x = 0; }


// Utility Class
class d3dHelper
{
public:
	static ID3D11ShaderResourceView* CreateTexture2DArraySRV(
		ID3D11Device* device, ID3D11DeviceContext* context, std::vector<std::wstring>& filenames);

	static ID3D11ShaderResourceView* CreateRandomTexture1DSRV(ID3D11Device* device);
};