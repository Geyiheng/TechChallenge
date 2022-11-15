#ifndef _PASS_RANGE_LIST_H_
#define _PASS_RANGE_LIST_H_
#include <ValueRange.h>
#include "Vision/VisionModule.h"
#include <vector>
/**
* CShootRangeList.
* ����һ���ɴ���ĽǶ��� �б�
*
*/

class CPassRangeList {
public:
	CPassRangeList(const CVisionModule* pVision);
	const CValueRangeList& getLeftPassRange();
	const CValueRangeList& getRightPassRange();

private:
	// �����ϴεģ���ֹ�ظ�����
	CValueRangeList lastLeftPassList; // �ϴλ�������
	CValueRangeList lastRightPassList; // �ϴλ�������
	int lastCycle; // �ϴε�ʱ��
	CGeoPoint lastPoint;

};
#endif // _PASS_RANGE_LIST_H_