#ifndef __ATTRIBUTE_SET_NEW_H__
#define __ATTRIBUTE_SET_NEW_H__

#include <vector>
#include <utility>
#include <GDebugEngine.h>
#include "AttributeNew.h"

//�������Լ�����Ȩ�õ�������Ȩ���Զ��Ӳ�����������ȡ
class CAttributeSetNew
{
	typedef std::pair<double, CAttributeNew*> TattributePair;//Ȩֵ�Ͷ�Ӧ������
public:
	CAttributeSetNew() {};
	~CAttributeSetNew()
	{
		for (auto attributePair : _attributePairList)
		{
			delete attributePair.second;
		}
	}

	double evaluate(const CVisionModule* pVision, int num, bool draw_debug = false)
	{
		double score = 0;
		string deb;
		for (auto& attributePair : _attributePairList)
		{
			attributePair.second->evaluate(pVision, num);
			score += attributePair.first * attributePair.second->getValue();
			deb = deb + to_string(100 * int(attributePair.first * attributePair.second->getValue())) + "#";
		}
		if (draw_debug && pVision->TheirPlayer(num).Valid())
			GDebugEngine::Instance()->gui_debug_msg(pVision->TheirPlayer(num).Pos() + Utils::Polar2Vector(40, Param::Math::PI / 4), deb.c_str());
		return score;
	}
protected:
	virtual void setAttributes() = 0;
	std::vector<TattributePair> _attributePairList;
};

//ʵ�������Լ����õĺ궨��

#define DEFINE_ATTRIBUTE_NEW(attr_class) \
class attr_class:CAttributeNew \
{ \
public:	\
	attr_class():CAttributeNew(#attr_class){}; \
	void evaluate(const CVisionModule* pVision,const int num); \
};

#define EVALUATE_ATTRIBUTE_NEW(attr_class) \
void attr_class::evaluate(const CVisionModule *pVision, const int num)

#define ADD_ATTRIBUTE_NEW(attr_class) \
_attributePairList.emplace_back(ParamManager::Instance()->factor_##attr_class,(CAttributeNew*)new attr_class)

#endif