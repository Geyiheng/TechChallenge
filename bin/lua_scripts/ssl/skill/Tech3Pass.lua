function Tech3Pass(task)
	local idx=task.idx
	execute=function(runner)
		return CTech3Pass(runner,idx)
	end
	matchPos=function()
		return ball.pos()
	end
	return execute,matchPos
end

gSkillTable.CreateSkill{
	name = "Tech3Pass",
	execute= function(self)
		print("This is in skill"..self.name)
	end
}