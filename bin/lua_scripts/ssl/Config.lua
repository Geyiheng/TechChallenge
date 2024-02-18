IS_TEST_MODE = true
IS_SIMULATION = CGetIsSimulation()
IS_YELLOW = CGetIsYellow()

IS_TEST_MODE = true
-- Test_Run Test_Break Test_ChaseNew Test_GetBall
-- Test_Run Test_Run6 Test_Speed
-- Test_Goalie Test_PassEachOther Test_NormalPass
-- Test_AdvanceV4 Test_MultiBack Test_GoSupport
if not IS_YELLOW then
	gTestPlay = "Test_Tech3Pass"
else
	gTestPlay = "Test_TechDefence"
end

-- PureDefence8 NormalKick
if not IS_YELLOW then
	OPPONENT_NAME = "NormalKick"
else
	OPPONENT_NAME = "NormalKick"
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
	Goalie   = {0},
	Kicker   = {1},
	Tier	 = {2},
	Receiver = {3}
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
	"Rush",
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
	"Ref/KickOff/Ref_KickOff_PureDefence8",
	"Ref/KickOff/Ref_KickOff_normal",
	--开球防守
	"Ref/KickOffDef/Ref_KickOffDef_PureDefence8",
	"Ref/KickOffDef/Ref_KickOffDef_normal",

	--角球进攻
	"Ref/CornerKick/Ref_CornerKick_PureDefence8",
	"Ref/CornerKick/Ref_CornerKick_normal",
	--前场进攻
	"Ref/FrontKick/Ref_FrontKick_PureDefence8",
	"Ref/FrontKick/Ref_FrontKick_normal",
	--中场进攻
	"Ref/MiddleKick/Ref_MiddleKick_normal",
	--后场进攻
	"Ref/BackKick/Ref_BackKick_PureDefence8",
	"Ref/BackKick/Ref_BackKick_normal",

	--角球防守
	"Ref/CornerDef/Ref_CornerDef_PureDefence8",
	"Ref/CornerDef/Ref_CornerDef_normal",
	--前场防守
	"Ref/FrontDef/Ref_FrontDef_PureDefence8",
	"Ref/FrontDef/Ref_FrontDef_normal",
	--中场防守
	"Ref/MiddleDef/Ref_MiddleDef_PureDefence8",
	"Ref/MiddleDef/Ref_MiddleDef_normal",
	--后场防守
	"Ref/BackDef/Ref_BackDef_PureDefence8",
	"Ref/BackDef/Ref_BackDef_normal",

	--己方放球
	"Ref/BallPlace/Ref_OurBallPlace",
	--对方放球
	"Ref/BallPlace/Ref_TheirBallPlace",

	--点球进攻
	"Ref/PenaltyKick/Ref_PenaltyKick_normal",
	"Ref/PenaltyKick/Ref_PenaltyKick_simple",
	--点球防守
	"Ref/PenaltyDef/Ref_PenaltyDef_normal",

	--停止站位
	"Ref/GameStop/Ref_Stop_normal",

	--强制停止
	"Ref/Ref_HaltV1",
}

gNorPlayTable = {
	"Nor/Nor_6",
	"Nor/Nor_6_defend",
	"Nor/Nor_8_PureDefence",
	"Nor/Nor_8_Power",
	"Nor/Nor_8_Strong_Tyh",
	"Nor/Nor_8_Simple"
}

gTestPlayTable = {
	"Test_TechDefence",
	"Test_Tech3Pass",
	"Test_RefChallenge",
	-- 基础移动相关
	"Test_Run",
	"Test_Run6",
	"Test_DSS",
	"Test_Circle",
	"Test_Speed",
	"Test_TimeDelayTest",
	-- 基础skill相关
	"Test_Rush",
	"Test_GetBall",
	"Test_ChaseKick",
	"Test_PassEachOther",
	"Test_Goalie",
	"Test_TouchKick",
	"Test_NormalPass",
	-- 组合相关
	"Test_ProtectBall",
	"Test_Advance",
	"Test_GoSupport",
	"Test_AdvanceV4",
	"Test_MultiBack",
	"Test_ProtectBall",
	"Test_TimeDelayTest",
	-- "Test_NormalPlay",
	-- "Test_NormalAttack",
	-- "Test_NormalDefend",
	-- "Test_AvoidFoulDefend",
	-- "Test_NormalMiddleDefend",
	-- -- "Test_Circle",
	-- "NormalPlay",
	-- "NormalPlayPureDefence8",
	-- "Test_defend6",
	-- "Test_play6ForTest",
	-- "Test_DSS",
	-- "Test_Run6",
	-- "Test_play8_AUTO",
	-- "Test_play8_ManyFoul",
	-- "Test_play8_NotFoul",
	-- "Test_play3",
	-- "Test_NormalPass",
	-- "Test_play8ForPower",
	-- "Test_play6",
	-- "Test_play8_Strong_Tyh",
	-- "Test_play8_Simple",
	-- "Test_physics"
}
