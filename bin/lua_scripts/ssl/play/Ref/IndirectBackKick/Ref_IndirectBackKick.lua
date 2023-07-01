local FreeKick_ImmortalStart_Pos = function ()
	local pos
	local ball2goal = CVector:new_local(CGeoPoint:new_local(450, 0) - ball.pos())
	pos = ball.pos() + Utils.Polar2Vector(80, ball2goal:dir())
	local tempPos = ball.pos()+ Utils.Polar2Vector(80, ball2goal:dir())
	for i = 1, 6 do
		if vision:TheirPlayer(i):Valid() then
			if vision:TheirPlayer(i):Pos():dist(tempPos) < 20 then
				local dir = (CGeoPoint:new_local(450, 30 * ball.antiY()) - vision:TheirPlayer(i):Pos()):dir()
				pos = vision:TheirPlayer(i):Pos() + Utils.Polar2Vector(20, dir)
				break
			end
		else
			pos = tempPos
		end
		
	end
	return pos
end

local Field_RobotFoward_Pos = function (role)
	local pos
	local UpdatePos = function ()
		local roleNum = player.num(role)
		if Utils.PlayerNumValid(roleNum) then
			local carPos = vision:OurPlayer(roleNum):Pos()
			pos = carPos + Utils.Polar2Vector(1, vision:OurPlayer(roleNum):Dir())
		end
		return pos
	end
	return UpdatePos
end

local Farpos = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refSyntYPos(CGeoPoint:new_local(-130,275))()
	else
		return ball.refAntiYPos(CGeoPoint:new_local(-130,275))()
	end
end

local Nearpos = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refSyntYPos(CGeoPoint:new_local(-60,110))()
	else
		return ball.refAntiYPos(CGeoPoint:new_local(-60,110))()
	end
end

local Shootingpos = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refSyntYPos(CGeoPoint:new_local(500,212))()
	else
		return ball.refAntiYPos(CGeoPoint:new_local(500,212))()
	end
end

local chippos = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		print(1111111)
		return ball.refAntiYPos(CGeoPoint:new_local(0,300))()
	else
		print(222222222)
		return  ball.refSyntYPos(CGeoPoint:new_local(0,300))()
	end
end

local helppos = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refAntiYPos(CGeoPoint:new_local(-270,390))()
	else
		return ball.refSyntYPos(CGeoPoint:new_local(-270,390))()
	end
end

local assipos = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refAntiYPos(CGeoPoint:new_local(95,190))()
	else
		return ball.refSyntYPos(CGeoPoint:new_local(95,190))()
	end
end

local getpos  = function()
	if  ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refAntiYPos(CGeoPoint:new_local(125,340))()
	else
		return ball.refSyntYPos(CGeoPoint:new_local(125,340))()
	end
end

local Blockpos = function()
	if ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 then
		return ball.refSyntYPos(CGeoPoint:new_local(90,-310))()
	else
		return ball.refAntiYPos(CGeoPoint:new_local(90,-310))()
	end
end

-- if(ball.posX() > -265 and ball.posY()<150 and ball.posY()> -150 ) then
-- 	 Farpos = ball.refSyntYPos(CGeoPoint:new_local(-130,275))
-- 	 Nearpos = ball.refSyntYPos(CGeoPoint:new_local(-60,110))
-- 	 Shootingpos = ball.refSyntYPos(CGeoPoint:new_local(500,212))
-- 	 chippos = ball.refAntiYPos(CGeoPoint:new_local(0,300))
-- 	 helppos = ball.refAntiYPos(CGeoPoint:new_local(-270,390))
-- 	 assipos = ball.refAntiYPos(CGeoPoint:new_local(95,190))
-- 	 getpos = ball.refAntiPos(CGeoPoint:new_local(125,340))
-- else
-- 	 Farpos = ball.refAntiYPos(CGeoPoint:new_local(-130,275))
-- 	 Nearpos = ball.refAntiYPos(CGeoPoint:new_local(-60,110))
-- 	 Shootingpos = ball.refAntiYPos(CGeoPoint:new_local(500,212))
-- 	 chippos = ball.refSyntYPos(CGeoPoint:new_local(0,300))
-- 	 helppos = ball.refSyntYPos(CGeoPoint:new_local(-270,390))
-- 	 assipos = ball.refSyntYPos(CGeoPoint:new_local(95,190))
-- 	 getpos = ball.refSyntYPos(CGeoPoint:new_local(125,340))
-- end
local FREEKICKPOS
local FIELDROBOTPOS
FREEKICKPOS = FreeKick_ImmortalStart_Pos



PreGoPos = function (role)
	return function ()
		return player.pos(role) + Utils.Polar2Vector(5, player.dir(role))
	end
end

gPlayTable.CreatePlay{

firstState = "startball",

["startball"] = {
	switch = function ()
		if bufcnt(player.toTargetDist("Breaker") < 30, "fast") then
			return "changepos"
		elseif bufcnt(true, 300) then
			return "exit"
		end
	end,
	Assister = task.staticGetBall(chippos,false),
	Breaker = task.goCmuRush(chippos),
	Leader   = task.rightBack(),
	Middle   = task.leftBack(),
	Special  = task.goCmuRush(Farpos),
	Defender = task.goCmuRush(Nearpos),
	Crosser	 = task.goCmuRush(helppos),
	Goalie   = task.goalie(),
	match    = "{A}[LM][BD][CS]"
},

["changepos"] = {
	switch = function ()
		if bufcnt(player.toTargetDist("Crosser") < 30 and player.toTargetDist("Defender") < 30,"fast") then
			return "chipball"
		elseif bufcnt(true, 225) then
			return "exit"
		end
	end,
	Assister = task.staticGetBall(chippos,false),
	Breaker = task.leftBack(),	
	Middle   = task.rightBack(),
	Special  = task.goCmuRush(getpos),	
	Defender = task.goCmuRush(Farpos),
	Crosser	 = task.goCmuRush(Nearpos),
	Leader   = task.goCmuRush(helppos),
	Goalie   = task.goalie(),
	match    = "{LMB}{A}[SD][C]"
},

["chipball"] = {
    switch = function ()
		if bufcnt(player.kickBall("Assister") or
				  player.toBallDist("Assister") > 20, "normal") then
			return "exit"
		elseif  bufcnt(true,75) then
			return "exit"
		end
	end,
	Assister = task.chipPass(chippos,400),
	Breaker = task.goCmuRush(getpos),
	Leader   = task.singleBack(),
	Special  = task.goCmuRush(Blockpos),
	Defender = task.leftBack(),
	Crosser	 = task.goCmuRush(assipos),
	Middle   = task.rightBack(),
	Goalie   = task.goalie(),
	match    = "{ALS}[C][DM][B]"
},



name = "Ref_IndirectBackKick",
applicable ={
	exp = "a",
	a   = true
},
attribute = "attack",
timeout = 99999
}
