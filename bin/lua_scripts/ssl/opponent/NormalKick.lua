gOppoConfig = {
  ----------------------play---------------------------------------------------------------

	IndirectCornerKick  = {"Ref_IndirectCornerPush_normal_chip"},--80 --"Ref_CornerPush"

	DirectCornerKick  = {"Ref_DirectCornerPush_normal"},--80 --"Ref_CornerPush"

	IndirectFrontKick   = {"Ref_IndirectFrontPush_normal_chip"},--610 --"Ref_FrontPush"--对面

	DirectFrontKick   = {"Ref_DirectFrontPush_normal"},--610 --"Ref_FrontPush"--对面

	MiddleKick    = {"Ref_MiddleKick_normal"},

	BackKick    = {"Ref_BackKick"}, --"Ref_ImmortalKickV610" --"Ref_BackPush"  --{10, "Ref_ImmortalKickV1"}
	
-------------------------------Def---------------------------------------------------

	CornerDef   = "Ref_CornerDef_normal", -- 可能要用v4，防头球
	BackDef 	= "Ref_BackDef_normal",
	MiddleDef   = "Ref_MiddleDef_normal",--横向marktouch用v5，四车markfront用v10,保守打法用V2
	FrontDef    = "Ref_FrontDef_normal", --四车markfront用v9,若挑就用V7，若他们四车全上就用V11,保守打法用Ref_FrontDefV2

	PenaltyKick = "Ref_PenaltyKick8", --Ref_PenaltyKick2017V5
	PenaltyDef  = "Ref_PenaltyDef8", --Ref_PenaltyDefV3

	KickOff		= "Ref_KickOff_normal",
	KickOffDef  = "Ref_KickOffDef_normal",
	
	NorPlay     = "Test_play8_AUTO"
	--犯规多的时候用"Test_play8_ManyFoul"
	--犯规少的时候用"Test_play8_NotFoul"
	--AutoChange用"Test_play8_AUTO"

}


--     B        -250           M        0   F_A  100  F_B  200  F_C   350
-----------------|----------------------|--------|---------|-----------|---------



--                                      0    F_D   125    F_E
----------------------------------------|-----------|----------------------------
