#ifndef Tech3Pass_h__
#define Tech3Pass_h__
#include <skill/PlayerTask.h>
#include <time.h>
class CTech3Pass : public CStatedTask{
public:
	CTech3Pass();
	~CTech3Pass();
	virtual void plan(const CVisionModule* pVision);
	virtual bool isEmpty() const { return false;}
	virtual CPlayerCommand* execute(const CVisionModule * pVision);
	CGeoPoint limitpos(CGeoPoint pos, int fla = 1);
	void passto(const CVisionModule* pVision);
	void passwho(const CVisionModule* pVision, int change = 0, int passer = -1);
	CGeoPoint passwho(const CGeoPoint& A1, const CGeoPoint& A2, const CGeoPoint& A3, const CGeoPoint& B1);
	bool passwhen(const CVisionModule* pVision);
	enum State {
		state_ready = 1,
		state_wait,
		state_pass
	};
	static int num, buff, ifstep2, ifstart, rotvelbuff, ifchange;
	//  forcekickbuff;//num为接球人, buff用于buffcnt, ifstep2和ifstart用于切换状态
	CGeoPoint circleCenter[3] = {CGeoPoint(150, 0), CGeoPoint(-75, 130), CGeoPoint(-75, -130)};//三个圆心
	CGeoPoint centre;//自身机器人圆心
protected:
};

#endif //Tech3Pass_h__