gPlayTable.CreatePlay{

firstState = "init",

["init"] = {
	switch = function ()
		if bufcnt(cond.isNormalStart()) then
			return "pass"
		end
	end,
	Goalie  = task.goCmuRush(CGeoPoint:new_local(-280,0),_,_,flag.dodge_ball),
	Leader = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,200/900*param.pitchWidth),_,_,flag.dodge_ball),
	Assister = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,150/900*param.pitchWidth),_,_,flag.dodge_ball),
	Middle = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-150/900*param.pitchWidth),_,_,flag.dodge_ball),
    Special = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,250/900*param.pitchWidth),_,_,flag.dodge_ball),
	Defender = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-250/900*param.pitchWidth),_,_,flag.dodge_ball),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-200/900*param.pitchWidth),_,_,flag.dodge_ball),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-100/900*param.pitchWidth),_,_,flag.dodge_ball),
	match = "[C][LB][DS][AM]"
},

["pass"] = {
	switch = function ()
		if player.kickBall("Goalie") then
			return "kick"
		end
	end,
	--Goalie  = task.penaltykick(),
	Goalie = task.chipPass(CGeoPoint:new_local(100, 0)),
	Leader = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,200/900*param.pitchWidth),_,_,flag.dodge_ball),
	Assister = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,150/900*param.pitchWidth),_,_,flag.dodge_ball),
	Middle = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-150/900*param.pitchWidth),_,_,flag.dodge_ball),
    Special = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,250/900*param.pitchWidth),_,_,flag.dodge_ball),
	Defender = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-250/900*param.pitchWidth),_,_,flag.dodge_ball),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-200/900*param.pitchWidth),_,_,flag.dodge_ball),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-100/900*param.pitchWidth),_,_,flag.dodge_ball),
	match = "[C][LB][DS][AM]"
},

["kick"] = {
	switch = function ()
		if ball.posX()>0.5*param.pitchLength then
			return "exit"
		end
	end,
	--Goalie  = task.penaltykick(),
	Goalie = task.chaseNew(),
	Leader = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,200/900*param.pitchWidth),_,_,flag.dodge_ball),
	Assister = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,150/900*param.pitchWidth),_,_,flag.dodge_ball),
	Middle = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-150/900*param.pitchWidth),_,_,flag.dodge_ball),
    Special = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,250/900*param.pitchWidth),_,_,flag.dodge_ball),
	Defender = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-250/900*param.pitchWidth),_,_,flag.dodge_ball),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-200/900*param.pitchWidth),_,_,flag.dodge_ball),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-param.pitchLength/3,-100/900*param.pitchWidth),_,_,flag.dodge_ball),
	match = "[C][LB][DS][AM]"
},

name = "Ref_PenaltyKick8",
applicable ={
	exp = "a",
	a = true
},
attribute = "attack",
timeout = 99999
}
