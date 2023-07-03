local function dir__dead()
	local function dir_dead_inside(runner)
		return (CGeoPoint:new_local(600/1200*param.pitchLength,50/900*param.pitchWidth)-ball.pos()):dir()
	end
	return dir_dead_inside
end

gPlayTable.CreatePlay{

firstState = "init",

["init"] = {
	switch = function ()
		if bufcnt(1, "fast", 50) then
			return "gotolong"
		end
	end,
	Goalie  = task.goCmuRush(CGeoPoint:new_local(-185/1200*param.pitchLength,0/900*param.pitchWidth),_,_,flag.allow_dss),
	Leader = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,200/900*param.pitchWidth),_,_,flag.allow_dss),
	Assister = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,150/900*param.pitchWidth),_,_,flag.allow_dss),
	Middle = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-150/900*param.pitchWidth),_,_,flag.allow_dss),
    Special = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,250/900*param.pitchWidth),_,_,flag.allow_dss),
	Defender = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-250/900*param.pitchWidth),_,_,flag.allow_dss),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-200/900*param.pitchWidth),_,_,flag.allow_dss),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-100/900*param.pitchWidth),_,_,flag.allow_dss),
	match = "[C][LB][DS][AM]"
},

["gotolong"] = {
	switch = function ()
		if bufcnt(player.toTargetDist("Goalie") < 15 , 10 , 120) then
			return "templong"
		end
	end,
	Goalie  = task.goCmuRush(CGeoPoint:new_local(-170/1200*param.pitchLength,0/900*param.pitchWidth),_,_,flag.allow_dss),
	Leader = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,200/900*param.pitchWidth),_,_,flag.allow_dss),
	Assister = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,150/900*param.pitchWidth),_,_,flag.allow_dss),
	Middle = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-150/900*param.pitchWidth),_,_,flag.allow_dss),
    Special = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,250/900*param.pitchWidth),_,_,flag.allow_dss),
	Defender = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-250/900*param.pitchWidth),_,_,flag.allow_dss),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-200/900*param.pitchWidth),_,_,flag.allow_dss),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-100/900*param.pitchWidth),_,_,flag.allow_dss),
	match = "[C][LB][DS][AM]"
},

["templong"] = {
	switch = function ()
		if bufcnt(cond.isNormalStart()) then
			return "kick"
		end
	end,
	Goalie  = task.goCmuRush(CGeoPoint:new_local(-160/1200*param.pitchLength,0/900*param.pitchWidth),_,_,flag.allow_dss),
	Leader = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,200/900*param.pitchWidth),_,_,flag.allow_dss),
	Assister = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,150/900*param.pitchWidth),_,_,flag.allow_dss),
	Middle = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-150/900*param.pitchWidth),_,_,flag.allow_dss),
    Special = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,250/900*param.pitchWidth),_,_,flag.allow_dss),
	Defender = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-250/900*param.pitchWidth),_,_,flag.allow_dss),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-200/900*param.pitchWidth),_,_,flag.allow_dss),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-100/900*param.pitchWidth),_,_,flag.allow_dss),
	match = "[C][LB][DS][AM]"
},

["kick"] = {
	switch = function ()
		if ball.posX()>500 then
			return "exit"
		end
	end,
	Goalie  = task.penaltykick(),
	Leader = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,200/900*param.pitchWidth),_,_,flag.allow_dss),
	Assister = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,150/900*param.pitchWidth),_,_,flag.allow_dss),
	Middle = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-150/900*param.pitchWidth),_,_,flag.allow_dss),
    Special = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,250/900*param.pitchWidth),_,_,flag.allow_dss),
	Defender = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-250/900*param.pitchWidth),_,_,flag.allow_dss),
	Breaker = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-200/900*param.pitchWidth),_,_,flag.allow_dss),
	Crosser = task.goCmuRush(CGeoPoint:new_local(-300/1200*param.pitchLength,-100/900*param.pitchWidth),_,_,flag.allow_dss),
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
