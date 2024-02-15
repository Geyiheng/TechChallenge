#ifndef Tech3Pass_h__
#define Tech3Pass_h__
#include <skill/PlayerTask.h>
class CTech3Pass : public CStatedTask{
public:
	CTech3Pass();
	~CTech3Pass();
	virtual void plan(const CVisionModule* pVision);
	virtual bool isEmpty() const { return false;}
	virtual CPlayerCommand* execute(const CVisionModule * pVision);

protected:
};

#endif //Tech3Pass_h__