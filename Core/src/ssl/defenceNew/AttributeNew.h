#ifndef __ATTRIBUTE_NEW_H__
#define __ATTRIBUTE_NEW_H__

#include "Vision/VisionModule.h"
#include "ParamManagerNew.h"

using namespace std;

//��������� �����������۷���������ֵ�����Ե�����ֻ���ڳ�ʼ��ʱָ��
//��ԭDefenceInfo��ֲ����΢�޸ģ�name����Ŀǰû���ϣ��������Ҳû���Ͽ��Կ���ֱ��ɾ��
class CAttributeNew
{
public:
	CAttributeNew(const string name) :_name(name), _value(0) {}
	~CAttributeNew() {}

	//���۵Ľӿ�,num�Ƕ��ֳ��ĳ���
	virtual void evaluate(const CVisionModule* pVision, const int num) = 0;

	//��ȡ����ֵ
	double getValue() { return _value; }
	string getName() { return _name; }

protected:
	void setValue(double theValue) { _value = theValue; }

private:
	double _value;
	string _name;
};

#endif