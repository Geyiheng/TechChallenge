local ALL_AVOID = flag.dodge_ball + flag.avoid_stop_ball_circle + flag.avoid_shoot_line
local ALL_NOT_AVOID = flag.not_avoid_their_vehicle + flag.not_avoid_our_vehicle + flag.not_dodge_penalty
local FLAG = ALL_AVOID + flag.dribbling

local distThreshold = 50

gPlayTable.CreatePlay{

firstState = "chase",

["chase"] = {
	switch = function ()
		if bufcnt(ball.toPlayerHeadDist("Kicker") < 2, 50,200)then
			return "break";
		end
	end,
    
	Kicker = task.slowGetBall(),
    match = ""
},
["break"] = {
	switch = function ()
		if bufcnt(ball.toPlayerHeadDist("Kicker") > 10, "fast")then
			return "chase";
		end
	end,
    Kicker = task.Break(),
    match = ""
},

name = "Test_ChaseKick",
applicable ={
	exp = "a",
	a = true
},
attribute = "attack",
timeout = 99999

}