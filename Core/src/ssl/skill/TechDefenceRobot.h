#ifndef TechDefence_h__
#define TechDefence_h__
#include <skill/PlayerTask.h>
class CTechDefence : public CStatedTask{
public:
	CTechDefence();
	~CTechDefence();
	virtual void plan(const CVisionModule* pVision);
	virtual bool isEmpty() const { return false;}
	virtual CPlayerCommand* execute(const CVisionModule * pVision);

protected:
};

#endif //TechDefence_h__