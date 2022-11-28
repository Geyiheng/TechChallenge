local ALL_AVOID = flag.dodge_ball + flag.avoid_stop_ball_circle + flag.avoid_shoot_line
local ALL_NOT_AVOID = flag.not_avoid_their_vehicle + flag.not_avoid_our_vehicle + flag.not_dodge_penalty
local FLAG = ALL_AVOID + flag.dribbling
local TargetPos1  = CGeoPoint:new_local(-450,0)
local distThreshold = 50

gPlayTable.CreatePlay{

firstState = "advance",

["advance"] = {
	switch = function ()
		if true then
			return "advance";
		end
	end,
	Leader = task.advanceV4(),
    Special = task.goRightSupport(),
	Middle = task.goLeftSupport(),
    --Breaker = task.leftCenterBack(),
    --Crosser = task.rightCenterBack(),
    --match = ""
	match    = ""
},

name = "Test_AdvanceV4",
applicable ={
	exp = "a",
	a = true
},
attribute = "attack",
timeout = 99999
}
