function TechDefence(task)
	execute=function(runner)
		return CTechDefence(runner)
	end
	matchPos=function()
		return ball.pos()
	end
	return execute,matchPos
end

gSkillTable.CreateSkill{
	name = "TechDefence",
	execute= function(self)
		print("This is in skill"..self.name)
	end
}

