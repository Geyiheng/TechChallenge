#ifndef __ATTRIBUTE_NEW_H__
#define __ATTRIBUTE_NEW_H__

#include "Global.h"

using namespace std;

//��������� �����������۷���������ֵ�����Ե�����ֻ���ڳ�ʼ��ʱָ��
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