// Utils
//

#pragma once

#include "DX11Includes.h"
#include <cassert>


// D3D Error checker
#if defined(DEBUG) || defined(_DEBUG)

#ifndef HR
#define HR(x)						\
	{								\
			HRESULT hr = (x);		\
			if(FAILED(hr))			\
			{						\
				assert(0);			\
			}						\
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
