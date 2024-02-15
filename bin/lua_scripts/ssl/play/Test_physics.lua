


gPlayTable.CreatePlay{

firstState = "touch",

["touch"] = {
	switch = function ()
		if ball.velMod()>5 then 
			print(ball.velMod())
			return "touch";
		end
	end,
	Leader = task.techdefence(),
	-- Assister=task.goCmuRush(CGeoPoint:new_local(0,0)),
	match = "[L]"
},



name = "Test_physics",
applicable ={
	exp = "a",
	a = true
},
attribute = "attack",  
timeout = 99999
}

