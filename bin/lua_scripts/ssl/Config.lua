IS_TEST_MODE = true
IS_SIMULATION = CGetIsSimulation()
IS_YELLOW = CGetIsYellow()

if not IS_YELLOW then
	OPPONENT_NAME = "NormalKick"-- PureDefence8 NormalKick
else
	OPPONENT_NAME = "NormalKick"
end

gTestPlay = function ()
	if not IS_YELLOW then
		return "Test_TechDefence" -- Test_AdvanceV4 Test_NormalPass Test_play6 Test_Run
		--犯规多的时候用"Test_play8_ManyFoul"
		--犯规少的时候用"Test_play8_NotFoul"
		--AutoChange用"Test_play8_AUTO"
		-- NormalPlayPureDefence8   Test_play8_Strong_Tyh

	else
		return "Test_Tech3Pass" -- Test_Defence Test_PassEachOther Test_NormalMiddleDefend
	end
end
gNormalPlay = "Test_play8_AUTO"
gSwitchNum = {
	["normal"]  = 6,
	["backup"]  = 7,
	["state"]   = "normal"
}

-- 有0号在场门将就是0号，没有0号在场，看这里设置的号码
-- 建议比赛时只要上了0号，这里就写0号，否则万一0号视觉丢了，就会匹配其他车去当门将，犯规
gRoleFixNum = {
	["Kicker"]   = {7},
	-- ["Goalie"]   = {0},
	["Tier"]	 = {13},
	["Receiver"] = {12}
}

gSkill = {
	"TechDefence",
	"Tech3Pass",
	--走位
	"GoAndTurn",
	"DribbleTurn",
	"GoSupport",
	--位移
	"SpeedTest",
	"SmartGoto",
	"GoCmuRush",
	"SimpleGoto",
	"BezierRush",
	"GotoMatchPos",
	"RunMultiPos",
	"RunMultiPosV2",
	"RunMultiPosVel",
	"GoTechChalPos",
	--踢球
	"JustKick",
	"WaitKick",
	"TouchKick",
	"ShootBall",
	"ShootBallV2",
	"GoAndTurnKick",
	"GoAndTurnKickV3",
	"GoAndTurnKickV4",
	"DribbleTurnKickV2",
	--拿球
	"GetBall",
	"SlowGetBall",
	"StaticGetBall",
	--进攻
	"ChaseKick",
	"ChaseKickV2",
	"AdvanceBall",
	"AdvanceBallV3",
	"AdvanceBallV4",
	--防守
	"Marking",
	"MarkingTouch",
	"Goalie2013",
	"Goalie2022",
	"ProtectBall",
	--传球
	"ReceivePass",
	--展示
	"GoAroundBall",
	"GoAroundRobot",
	"GoPIDCircle",
	--放球
	"FetchBall",
	--停止
	"Stop",
	--点球
	"PenaltyKick",
	"PenaltyGoalie",
	"Break",
	"PenaltyKickV2",
	--测试
	"TimeDelayTest",

}

gRefPlayTable = {
	--开球进攻
	"Ref/KickOff/Ref_KickOff8",
	"Ref/KickOff/Ref_KickOff_normal",
	--开球防守
	"Ref/KickOffDef/Ref_KickOffDef8",
	"Ref/KickOffDef/Ref_KickOffDef_normal",

	--角球进攻
	"Ref/DirectCornerKick/Ref_DirectCornerPush8",
	"Ref/DirectCornerKick/Ref_DirectCornerPush_normal",
	"Ref/IndirectCornerKick/Ref_IndirectCornerPush8",
	"Ref/IndirectCornerKick/Ref_IndirectCornerPush_normal_chip",
	--前场进攻
	"Ref/DirectFrontKick/Ref_DirectFrontPush8",
	"Ref/DirectFrontKick/Ref_DirectFrontPush_normal",
	"Ref/IndirectFrontKick/Ref_IndirectFrontPush8",
	"Ref/IndirectFrontKick/Ref_IndirectFrontPush_normal_chip",
	--中场进攻
	"Ref/MiddleKick/Ref_MiddleKick_normal",
	--后场进攻
	"Ref/BackKick/Ref_BackPush8",
	"Ref/BackKick/Ref_BackKick",
	"Ref/BackKick/Ref_BackPush_normal_chip",

	--角球防守
	"Ref/CornerDef/Ref_CornerDef8",
	"Ref/CornerDef/Ref_CornerDef_normal",
	--中场防守
	"Ref/MiddleDef/Ref_MiddleDef8",
	"Ref/MiddleDef/Ref_MiddleDef_normal",
	--前场防守
	"Ref/FrontDef/Ref_FrontDef8",
	"Ref/FrontDef/Ref_FrontDef_normal",
	--后场防守
	"Ref/BackDef/Ref_BackDef8",
	"Ref/BackDef/Ref_BackDef_normal",

	--己方放球
	"Ref/BallPlace/Ref_OurBallPlaceV2",
	--对方放球
	"Ref/BallPlace/Ref_TheirBallPlace",

	--点球进攻
	"Ref/PenaltyKick/Ref_PenaltyKick8",
	--点球防守
	"Ref/PenaltyDef/Ref_PenaltyDef8",

	--停止站位
	"Ref/GameStop/Ref_StopV2_normal",
	"Ref/GameStop/Ref_StopV3",
	"Ref/GameStop/Ref_StopV3_normal",
	"Ref/GameStop/Ref_StopV2_Play8",
	--PureDefence8
	"Ref/GameStop/Ref_StopCornerDefPureDefence",
	--NormalKick
	"Ref/GameStop/Ref_Stop4CornerKick",
	"Ref/GameStop/Ref_Stop4FrontKick",
	"Ref/GameStop/Ref_Stop4MiddleKick",
	"Ref/GameStop/Ref_Stop4BackKick",
	"Ref/GameStop/Ref_Stop4CornerDef_play8",
	"Ref/GameStop/Ref_Stop4BackDef_play8",
	"Ref/GameStop/Ref_Stop4MiddleDef_play8",
	"Ref/GameStop/Ref_Stop4FrontDef_play8",

	--强制停止
	"Ref/Ref_HaltV1",
}

gBayesPlayTable = {
	"Nor/NormalPlayMark",
}

gTestPlayTable = {
	"Test_TechDefence",
	"Test_Tech3Pass",
	"Test_RefChallenge",
	"Test_Run",
	"Test_BezierRun",
	"Test_GetBall",
	"Test_ChaseKick",
	"Test_PassNormal",
	"Test_ReceivePass",
	"Test_PassEachOther",
	"Test_StaticGetBall",
	"Test_Speed",
	"Test_Advance",
	"Test_GoSupport",
	"Test_AdvanceV4",
	"Test_TouchKick",
	"Test_Defence",
	"Test_Goalie",
	"Test_MultiBack",
	"Test_ProtectBall",
	"Test_TimeDelayTest",
	"Test_GoTechChalPos",
	"Test_NormalPlay",
	"Test_NormalAttack",
	"Test_NormalDefend",
	"Test_AvoidFoulDefend",
	"Test_NormalMiddleDefend",
	"Test_Circle",
	"NormalPlay",
	"NormalPlayPureDefence8",
	"Test_defend6",
	"Test_play6ForTest",
	"Test_DSS",
	"Test_Run6",
	"Test_play8_AUTO",
	"Test_play8_ManyFoul",
	"Test_play8_NotFoul",
	"Test_play3",
	"Test_NormalPass",
	"Test_play8ForPower",
	"Test_play6",
	"Test_play8_Strong_Tyh",
	"Test_play8_Simple",
	"Test_physics"
}
