#include "UserInput.h"
#include "Engine/UserOutput/UserOutput.h"
namespace eae6320
{
	namespace UserInput
	{
		namespace MouseMovement
		{
			int xPosCached = -1;
			int yPosCached = -1;
			
		}
		namespace KeyState {
			uint8_t lastFrameKeyState[30];
		}
	}
}